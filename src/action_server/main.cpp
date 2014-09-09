#include <vector>
#include <cmath>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common_headers.h>
#include <pcl/ModelCoefficients.h>
#include "passage_finder.h"

// base class for action server
#include <actionlib/server/simple_action_server.h>

// Action description files (check the /action dir)
#include <action_server/MoveAlongAction.h>
#include <action_server/SwitchWallAction.h>
#include <action_server/ApproachDoorAction.h>
#include <action_server/PassDoorAction.h>
#include <action_server/SwitchSideAction.h>

// Mutex is necessary to avoid control races between task handlers and cloud callback
// cloud callback (or callback) is needed to renew information about point cloud and external sensors
#include <boost/thread/mutex.hpp>

boost::shared_ptr<DaVinci>  davinci;

boost::shared_ptr<boost::mutex> mutex_ptr;
boost::shared_ptr<LocationServer> loc_srv;
boost::shared_ptr<MotionServer>   msn_srv;
boost::shared_ptr<MappingServer>  map_srv;
boost::shared_ptr<Advanced_Passage_finder> apf;

std::string input_topic;
std::string output_topic_vel;

ros::Subscriber sub;
ros::Publisher  pub_vel;

// These parameters are configured in 'airdrone_launch/param.yaml':
double vel_P = 0.0, vel_I = 0.0, vel_D = 0.0;
double ang_P = 0.0, ang_I = 0.0, ang_D = 0.0;

double target_dist     = 0.0;
double target_angl     = 0.0;
double movement_speed  = 0.0;
double angle_of_kinect = 0.0;
double apf_min_width   = 0.0;
double apf_min_dist    = 0.0;
double apf_max_dist    = 0.0;
double apf_min_angl    = 0.0;
double apf_max_angl    = 0.0;
double apf_better_q    = 0.0;
double move_epsilon    = 0.0;


double rot_epsilon = 0.3;
double wall_ang_eps = 5;



class ActionServer
{
public:
    ActionServer(ros::NodeHandle nh_) :
        as_move_along   (nh_, "MoveAlongAS",    boost::bind(&ActionServer::moveAlongCB,    this, _1), false),
        as_switch_wall  (nh_, "SwitchWallAS",   boost::bind(&ActionServer::switchWallCB,   this, _1), false),
        as_approach_door(nh_, "ApproachDoorAS", boost::bind(&ActionServer::approachDoorCB, this, _1), false),
        as_pass_door    (nh_, "PassDoorAS",     boost::bind(&ActionServer::passDoorCB,     this, _1), false),
        as_switch_side  (nh_, "SwitchSideAS",  boost::bind(&ActionServer::switchSideCB,   this, _1), false),
		on_left_side (true)
    {
        as_move_along   .start();
        as_switch_wall  .start();
        as_approach_door.start();
        as_pass_door    .start();
        as_switch_side .start();
    }

    ~ActionServer(void) {}


    void moveAlongCB (const action_server::MoveAlongGoalConstPtr   &goal)
    {
        action_server::MoveAlongResult      result_;
        action_server::MoveAlongFeedback feedback_;
        ros::Rate r(60);

        float vel = 0;

        if (this->on_left_side)
        	vel = -fabs(goal->vel);
        else
        	vel = fabs(goal->vel);

        while (true) {
            loc_srv->lock();
            msn_srv->track();
            msn_srv->ref_ang = target_angl; // Face along the wall

            //
            // Stop cases:
            //
            if (vel == 0) {
                // No velocity no movement. This is actually a misuse case
                result_.error = true;
                msn_srv->move_parallel(0); // Stop movement
                loc_srv->unlock();         // Releasing mutex
                as_move_along.setSucceeded(result_); // Instantly leave the function
                return;
            }

            if (loc_srv->obstacle_detected_left() && vel > 0) {
                // Found wall on the left and moving towards
                result_.left = true;
                msn_srv->move_parallel(0); // Stop movement
                loc_srv->unlock();         // Releasing mutex
                as_move_along.setSucceeded(result_); // Instantly leave the function
                return;
            }

            if (loc_srv->obstacle_detected_rght() && vel < 0) { // Found wall on the left and moving towards
                result_.rght = true;
                msn_srv->move_parallel(0); // Stop movement
                loc_srv->unlock();         // Releasing mutex
                as_move_along.setSucceeded(result_); // Instantly leave the function
                return;
            }

            if (apf->passages.size() > 0) {
                if (!isnan(apf->passages.at(0).cmd_left.x) &&
                    !isnan(apf->passages.at(0).cmd_left.y)) { // Found door in some wall
                    result_.found = true;
                    msn_srv->move_parallel(0); // Stop movement
                    msn_srv->set_angles_current(); // And rotation
                    loc_srv->unlock();         // Releasing mutex
                    as_move_along.setSucceeded(result_); // Instantly leave the function
                    return;
                }
                if (!isnan(apf->passages.at(0).cmd_rght.x) &&
                    !isnan(apf->passages.at(0).cmd_rght.y)) {
                    msn_srv->ref_ang  = target_angl;
                    msn_srv->ref_dist = target_dist;
                    msn_srv->move_parallel(vel);
                }
            }

            if (as_move_along.isPreemptRequested() || !ros::ok()) {
                msn_srv->move_parallel(0); // Stop movement
                loc_srv->unlock();         // Releasing mutex
                as_move_along.setPreempted();
                return;
            }

            //
            // Movement sustain
            //
            if (fabs(loc_srv->get_ref_wall()->angle - msn_srv->ref_ang) < 10 ) {
                msn_srv->ref_ang  = target_angl;
                msn_srv->ref_dist = target_dist;
                msn_srv->move_parallel(vel);
            }/*
            else if (fabs(loc_srv->get_ref_wall()->angle - msn_srv->ref_ang) > 30) {
                double A = loc_srv->get_ref_wall()->A;
                double B = loc_srv->get_ref_wall()->B;
                double C = loc_srv->get_ref_wall()->C;
                double p_x = - (A * C) / (A * A + B * B);
                double p_y = - (B * C) / (A * A + B * B);
                double p_x1 = loc_srv->get_ref_wall()->kin_inliers.at(0).x;
                double p_y1 = loc_srv->get_ref_wall()->kin_inliers.at(0).y;

                double distance = (p_x - p_x1) * (p_x - p_x1) + (p_y - p_y1) * (p_y - p_y1);

                msn_srv->ref_ang  = target_angl;
                msn_srv->ref_dist = target_dist;
                //ROS_INFO ("Distance: %f", distance);
                if (distance < goal->vel)
                    msn_srv->move_parallel(distance * 1 + 0.1 * loc_srv->get_ref_wall()->distance /
                                           (loc_srv->get_ref_wall()->angle * loc_srv->get_ref_wall()->angle));
                else
                    msn_srv->move_parallel(goal->vel);
            }*/
            else
            {
                msn_srv->move_parallel(0);
            }

            loc_srv->unlock();

            feedback_.vel = vel; // TODO: Publish current velocity (from sensors), not input
            as_move_along.publishFeedback(feedback_);
            r.sleep();
        }
    }

    void switchWallCB(const action_server::SwitchWallGoalConstPtr  &goal)
    {
        action_server::SwitchWallResult   result_;
        loc_srv->lock();

        if (goal->left) {
            if (loc_srv->obstacle_detected_left()) {
                loc_srv->track_wall(loc_srv->get_crn_wall_left());
                loc_srv->unlock();
                result_.success = true;
                as_switch_wall.setSucceeded(result_);
                return;
            }
            else
            {
                result_.success = false;
                ROS_ERROR("DEBUG leave left");
                as_switch_wall.setAborted(result_);
                return;
            }
        }
        if (goal->rght) {
            if (loc_srv->obstacle_detected_rght()) {
                loc_srv->track_wall(loc_srv->get_crn_wall_rght());
                loc_srv->unlock();
                result_.success = true;
                as_switch_wall.setSucceeded(result_);
                return;
            }
            else
            {
                result_.success = false;
                ROS_ERROR("DEBUG leave rght");
                as_switch_wall.setAborted(result_);
                return;
            }
        }
        if (goal->any) {
            if (loc_srv->obstacle_detected_left()) {
                loc_srv->track_wall(loc_srv->get_crn_wall_left());
                loc_srv->unlock();
                result_.success = true;
                as_switch_wall.setSucceeded(result_);
                return;
            }
            if (loc_srv->obstacle_detected_rght()) {
                loc_srv->track_wall(loc_srv->get_crn_wall_rght());
                loc_srv->unlock();
                result_.success = true;
                as_switch_wall.setSucceeded(result_);
                return;
            }
            else
            {
                result_.success = false;
                ROS_ERROR("DEBUG leave any");
                as_switch_wall.setAborted(result_);
                return;
            }
        }

        loc_srv->unlock();
    }

    //TODO: Add cases for NAN input
    bool move(pcl::PointXYZ target, double vel, double phi, double rot_vel)
    {
        if (isnan(target.x) || isnan(target.y) || isnan(phi)) {
            return false;
        }
        bool move_done = false, rot_done = false;
        ros::Rate r(60);
        double prev_angl = map_srv->get_global_angle();
        pcl::PointXYZ prev_pos = map_srv->get_global_positon();
        //pcl::PointXYZ target (dir.x, dir.y, 0);

        msn_srv->untrack();
        msn_srv->unlock();

        while (true)
        {
            msn_srv->lock();


            double current_angl      = map_srv->get_global_angle();
            pcl::PointXYZ current_pos = map_srv->get_global_positon();
            double delta_angl  = map_srv->diff(current_angl, prev_angl);
            pcl::PointXYZ shift = map_srv->diff(current_pos, prev_pos);
            prev_angl = current_angl;
            prev_pos  = current_pos;

            // "-" delta_angl here cuz we need to rotate target destination backwards, to compensate the positive drone rotation
            target.x = target.x - shift.x;
            target.y = target.y - shift.y;
            target = map_srv->rotate(target, -delta_angl);


            double len = sqrt (target.x * target.x + target.y * target.y);
            if (len < move_epsilon) {
                move_done = true;
            }
            else {
                msn_srv->buf_cmd.linear.x = target.x * vel / len;
                msn_srv->buf_cmd.linear.y = target.y * vel / len;
                msn_srv->buf_cmd.angular.z = 0;
                davinci->draw_point_cmd(target.x, target.y, 666, GOLD);
            }
            //TODO: conform sign of phi and rot_vel
            if (phi < rot_epsilon) {
                rot_done = true;
            }
            else {
                phi -= delta_angl;
                msn_srv->buf_cmd.angular.z = rot_vel;
            }

            if (rot_done && move_done) {
                msn_srv->unlock();
                break;
            }

            msn_srv->unlock();
            r.sleep();
        }
        return true;
    }

    void approachDoorCB(const action_server::ApproachDoorGoalConstPtr  &goal)
    {

        action_server::ApproachDoorResult   result_;
        ros::Rate r(60);

        msn_srv->lock();
        msn_srv->set_target_angle(0);

        if (apf->passages.size() == 0) {
            ROS_ERROR("No passage");
            as_approach_door.setAborted(result_);
                return;
        }

        if (!isnan(apf->passages.at(0).cmd_middle.x) && !isnan(apf->passages.at(0).cmd_middle.y)) {
        	ROS_INFO("Middle pass");
        	Line_param *pass_line = apf->get_best_line(apf->passages.at(0).kin_left, loc_srv->lm);
        	int offset_x = 0, offset_y = 0;
        	if (pass_line != NULL) {
        		offset_x = pass_line->ldir_vec.cmd.x * target_dist;
        		offset_y = pass_line->ldir_vec.cmd.y * target_dist;
        		ROS_INFO("Offset");
        	}
        	pcl::PointXYZ vec (apf->passages.at(0).cmd_middle.x + offset_x,
        					   apf->passages.at(0).cmd_middle.y + offset_y, 0);
        	int tmp = map_srv->track(vec) - 1;
        	if (this->move (vec, 0.6, 0, 0) == false) {
				ROS_ERROR("Move function caused error");
				as_approach_door.setAborted(result_);
				return;
			}
        	msn_srv->unlock();
			result_.success = true;
			as_approach_door.setSucceeded(result_);
			return;
        }

        double pass_x = apf->passages.at(0).cmd_left.x;
        double pass_y = apf->passages.at(0).cmd_left.y;
        if (isnan(pass_x) || isnan(pass_y)){
            ROS_ERROR("Wrong pass dist");
            as_approach_door.setAborted(result_);
            return;
        }


        double pass_dist = sqrt (pass_x * pass_x + pass_y * pass_y);
        ROS_INFO ("Pass: x %f\t y %f\t len %f", pass_x, pass_y, pass_dist);

        Line_param *pass_line = apf->get_best_line(apf->passages.at(0).kin_left, loc_srv->lm);
        davinci->draw_vec(pass_line->fdir_vec.kin.x, pass_line->fdir_vec.kin.y, 667, RED);

        if (pass_line == NULL) {
            ROS_ERROR("Pass_line wasn't found");
        }

        double along_dist = sqrt (pass_dist * pass_dist - pass_line->distance * pass_line->distance);


        pcl::PointXYZ vec ((pass_line->ldir_vec.cmd.x * along_dist + apf->passages.at(0).cmd_left.x - pass_line->fdir_vec.cmd.x) / 2,
                           (pass_line->ldir_vec.cmd.y * along_dist + apf->passages.at(0).cmd_left.y - pass_line->fdir_vec.cmd.y) / 2, 0);


        // New point tracking feature demo :)
        int pass_border_num = map_srv->track(pcl::PointXYZ(apf->passages.at(0).cmd_left.x, apf->passages.at(0).cmd_left.y, 0)) - 1;
        int start_pos_num = map_srv->track(vec) - 1;

/*
        msn_srv->unlock();
        while (true) {
            msn_srv->lock();
            msn_srv->track();
            if (apf->passages.size() > 0 &&
               fabs(apf->passages.at(0).left_ang) < wall_ang_eps) {
                msn_srv->unlock();
                break;
            }
            msn_srv->ref_ang  = -10;
            if (fabs(loc_srv->get_ref_wall()->angle - msn_srv->ref_ang) < wall_ang_eps ) {

                    msn_srv->ref_dist = target_dist;
                    msn_srv->move_parallel(-0.6);
                    if (loc_srv->get_ref_wall()->distance > target_dist) {
                        msn_srv->move_parallel(0);
                    }
            }
            msn_srv->unlock();
        }
        msn_srv->lock();
*/

        Passage pass;
        double vec_len = move_epsilon;
        int pass_pos = 0;
        int start_pos = 0;

        while (vec_len >= move_epsilon) {
			if (apf->passages.empty())
				break;
			pass = apf->passages.at(0);
			pass_line = apf->get_best_line(pass.kin_left, loc_srv->lm);
			if (pass_line != NULL) {
				ROS_ERROR("Pass_line wasn't found");
			}

			vec = pcl::PointXYZ((pass.cmd_left.x - pass_line->fdir_vec.cmd.x * target_dist) / 2,
								(pass.cmd_left.y - pass_line->fdir_vec.cmd.y * target_dist) / 2, 0);

			start_pos = map_srv->track(vec) - 1;

			if (this->move (vec, 0.4, 0, 0) == false) {
				ROS_ERROR("Move function caused error");
				as_approach_door.setAborted(result_);
				return;
			}

			vec_len = sqrt(vec.x * vec.x + vec.y*vec.y);
			pass_pos = map_srv->track(pcl::PointXYZ(pass.cmd_left.x, pass.cmd_left.y, 0)) - 1;
		}


        pcl::PointXYZ pass_vec (map_srv->tracked_points.at(pass_border_num).x,
                                map_srv->tracked_points.at(pass_border_num).y, 0);
        if (apf->passages.size() > 0) {
            pass_x = apf->passages.at(0).cmd_left.x;
            pass_y = apf->passages.at(0).cmd_left.y;
            if (!isnan(pass_x) || !isnan(pass_y)){
                pass_vec = pcl::PointXYZ (pass_x, pass_y, 0);
            }
        }

        //apf->passages.at(0).cmd_left.x;

        ROS_INFO("Pos: %f\t %f\n Vec: %f\t %f", vec.x, vec.y, pass_vec.x, pass_vec.y);

        //double pass_vec_len = sqrt (pass_vec.x * pass_vec.x + pass_vec.y * pass_vec.y);
        vec.x = (pass_vec.x + pass_vec.y) / sqrt(1.7);
        vec.y = (-pass_vec.x + pass_vec.y) / sqrt(1.7); // WTF? why '-'

        ROS_INFO("Vec: %f\t %f", vec.x, vec.y);

        int cur_pos_num = map_srv->track(vec) - 1;

        if (this->move (vec, 0.4, 70, 0.5) == false) {
            ROS_ERROR("Move function caused error");
            as_approach_door.setAborted(result_);
            return;
        }

        //pass_border = pcl::PointXYZ (map_srv->tracked_points.at(pass_border_num).x,
        //                             map_srv->tracked_points.at(pass_border_num).y, 0);
        pass_vec = pcl::PointXYZ (map_srv->tracked_points.at(pass_border_num).x - map_srv->tracked_points.at(start_pos_num).x,
                                  map_srv->tracked_points.at(pass_border_num).y - map_srv->tracked_points.at(start_pos_num).y, 0);

        if (apf->passages.size() > 0) {
            pass_x = apf->passages.at(0).cmd_left.x;
            pass_y = apf->passages.at(0).cmd_left.y;
            if (!isnan(pass_x) || !isnan(pass_y)){
                pass_vec = pcl::PointXYZ (pass_x - map_srv->tracked_points.at(start_pos_num).x,
                                          pass_y - map_srv->tracked_points.at(start_pos_num).y, 0);
            }
        }

        ROS_INFO("Pos: %f\t %f\n Vec: %f\t %f", map_srv->tracked_points.at(start_pos_num).x, map_srv->tracked_points.at(start_pos_num).y,
                                                               pass_vec.x, pass_vec.y);

        double pass_vec_len = sqrt (pass_vec.x * pass_vec.x + pass_vec.y * pass_vec.y);
        vec.x = map_srv->tracked_points.at(cur_pos_num).x + pass_vec.x;
        vec.y = map_srv->tracked_points.at(cur_pos_num).y + pass_vec.y;

        ROS_INFO("Vec: %f\t %f", vec.x, vec.y);

        map_srv->track(vec);

        if (this->move (vec, 0.5, 40, 0.5) == false) {
            ROS_ERROR("Move function caused error");
            as_approach_door.setAborted(result_);
            return;
        }

        msn_srv->unlock();
        result_.success = true;
        as_approach_door.setSucceeded(result_);
        return;
    }


    void passDoorCB(const action_server::PassDoorGoalConstPtr  &goal) {
        // TODO: Change for door on left sight
        action_server::PassDoorResult   result_;
        action_server::PassDoorFeedback feedback_;

        msn_srv->lock();


        msn_srv->unlock();
        result_.success = true;
        as_pass_door.setSucceeded(result_);
        return;
    }

    void switchSideCB(const action_server::SwitchSideGoalConstPtr  &goal){
    	action_server::SwitchSideResult result_;
    	msn_srv->lock();


    	this->on_left_side = !this->on_left_side;
    	target_angl *= -1;
    	//msn_srv->ref_ang = target_angl;

    	ROS_INFO("%d | %f | %f", this->on_left_side, msn_srv->ref_ang, target_angl);
    	msn_srv->unlock();
    	result_.success = true;
		as_switch_side.setSucceeded(result_);
		return;
    }

private:
    // NodeHandle instance must be created before this line. Otherwise strange error may occur.
    actionlib::SimpleActionServer<action_server::MoveAlongAction>    as_move_along;
    actionlib::SimpleActionServer<action_server::SwitchWallAction>   as_switch_wall;
    actionlib::SimpleActionServer<action_server::ApproachDoorAction> as_approach_door;
    actionlib::SimpleActionServer<action_server::PassDoorAction>     as_pass_door;
    actionlib::SimpleActionServer<action_server::SwitchSideAction>   as_switch_side;

    //
    bool on_left_side; // Reference side of the wall
};








void callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
    // Take control upon location and motion servers
    loc_srv->lock();

    if(cloud->points.size() < 10) {
        ROS_ERROR("AS: Cloud size = %lu", cloud->points.size());
    }

    loc_srv->spin_once(cloud);
    msn_srv->set_ref_wall(loc_srv->get_ref_wall());
    apf->renew(cloud);


    davinci->draw_line(loc_srv->get_crn_wall_left(), 1, BLUE);
    davinci->draw_line(loc_srv->get_ref_wall(),      0, GREEN);
    davinci->draw_line(loc_srv->get_crn_wall_rght(), 2, BLUE);

    for (int i = 0; i < apf->passages.size(); ++i) {
        davinci->draw_point(apf->passages.at(i).kin_left.x,   apf->passages.at(i).kin_left.y,   i*3 + 0, RED);
        davinci->draw_point(apf->passages.at(i).kin_middle.x, apf->passages.at(i).kin_middle.y, i*3 + 1, GREEN);
        davinci->draw_point(apf->passages.at(i).kin_rght.x,   apf->passages.at(i).kin_rght.y,   i*3 + 2, BLUE);

        davinci->draw_vec(apf->passages.at(i).kin_left.x,   apf->passages.at(i).kin_left.y,   i*3 + 0, RED);
        davinci->draw_vec(apf->passages.at(i).kin_middle.x, apf->passages.at(i).kin_middle.y, i*3 + 1, GREEN);
        davinci->draw_vec(apf->passages.at(i).kin_rght.x,   apf->passages.at(i).kin_rght.y,   i*3 + 2, BLUE);
    }

    msn_srv->spin_once();

    //map_srv->track(pcl::PointXYZ(0, 0, 0));
    davinci->draw_vec_cmd(msn_srv->base_cmd, 10, GOLD);
    // ------------
    //msn_srv->base_cmd.linear.x = 0;
    //msn_srv->base_cmd.linear.y = 0;
    //msn_srv->base_cmd.angular.z = 0.1;
    // ------------

    pub_vel.publish(msn_srv->base_cmd);
    msn_srv->clear_cmd();

    // Return control to task callback handlers
    loc_srv->unlock();
};



int main( int argc, char** argv )
{
    ros::init(argc, argv, "action_server");
    ros::NodeHandle nh;


    if (!nh.getParam("PID_ang_P", ang_P)) ROS_ERROR("Failed to get param 'PID_ang_P'");
    if (!nh.getParam("PID_ang_I", ang_I)) ROS_ERROR("Failed to get param 'PID_ang_I'");
    if (!nh.getParam("PID_ang_D", ang_D)) ROS_ERROR("Failed to get param 'PID_ang_D'");


    if (!nh.getParam("PID_vel_P", vel_P)) ROS_ERROR("Failed to get param 'PID_vel_P'");
    if (!nh.getParam("PID_vel_I", vel_I)) ROS_ERROR("Failed to get param 'PID_vel_I'");
    if (!nh.getParam("PID_vel_D", vel_D)) ROS_ERROR("Failed to get param 'PID_vel_D'");


    if (!nh.getParam("distance_to_wall", target_dist)) ROS_ERROR("Failed to get param 'distance_to_wall'");
    if (!nh.getParam("angle_to_wall",    target_angl)) ROS_ERROR("Failed to get param 'angle_to_wall'");
    if (!nh.getParam("movement_speed",   movement_speed)) ROS_ERROR("Failed to get param 'movement_speed'");
    if (!nh.getParam("angle_of_kinect",  angle_of_kinect)) ROS_ERROR("Failed to get param 'angle_of_kinect'");


    if (!nh.getParam("apf_min_width", apf_min_width)) ROS_ERROR("Failed to get param 'apf_min_width'");
    if (!nh.getParam("apf_min_dist",  apf_min_dist))  ROS_ERROR("Failed to get param 'apf_min_dist'");
    if (!nh.getParam("apf_max_dist",  apf_max_dist))  ROS_ERROR("Failed to get param 'apf_max_dist'");
    if (!nh.getParam("apf_min_angl",  apf_min_angl))  ROS_ERROR("Failed to get param 'apf_min_angl'");
    if (!nh.getParam("apf_max_angl",  apf_max_angl))  ROS_ERROR("Failed to get param 'apf_max_angl'");
    if (!nh.getParam("apf_better_q",  apf_better_q))  ROS_ERROR("Failed to get param 'apf_better_q'");
    if (!nh.getParam("move_epsilon",  move_epsilon))  ROS_ERROR("Failed to get param 'move_epsilon'");


    input_topic      = nh.resolveName("/shrinker/depth/laser_points");
    output_topic_vel = nh.resolveName("/cmd_vel_2");

    sub     = nh.subscribe<pcl::PointCloud<pcl::PointXYZ> > (input_topic,      1, callback);
    pub_vel = nh.advertise<geometry_msgs::Twist >           (output_topic_vel, 1 );


    davinci   = boost::shared_ptr<DaVinci>        (new DaVinci (nh));
    mutex_ptr = boost::shared_ptr<boost::mutex>   (new boost::mutex);
    loc_srv   = boost::shared_ptr<LocationServer> (new LocationServer(mutex_ptr));
    msn_srv   = boost::shared_ptr<MotionServer>   (new MotionServer  (mutex_ptr));
    map_srv   = boost::shared_ptr<MappingServer>  (new MappingServer (nh, "/ground_truth_to_tf/pose"));
    apf       = boost::shared_ptr<Advanced_Passage_finder> (new Advanced_Passage_finder());

    msn_srv->set_pid_ang(ang_P, ang_I, ang_D);
    msn_srv->set_pid_vel(vel_P, vel_I, vel_D);

    ActionServer action_server(nh);



    ros::spin ();

    return 0;
};
