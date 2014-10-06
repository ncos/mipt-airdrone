#include <vector>
#include <cmath>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point32.h>

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
#include <action_server/ApproachWallAction.h>
#include <action_server/MiddlePassAction.h>
#include <action_server/TakeoffAction.h>
#include <action_server/LandingAction.h>

// Message files
#include <ransac_slam/LineMap.h>

// Mutex is necessary to avoid control races between task handlers and cloud callback
// cloud callback (or callback) is needed to renew information about point cloud and external sensors
#include <boost/thread/mutex.hpp>

boost::shared_ptr<DaVinci>  davinci;

boost::shared_ptr<boost::mutex> mutex_ptr;
boost::shared_ptr<LocationServer> loc_srv;
boost::shared_ptr<MotionServer>   msn_srv;
boost::shared_ptr<MappingServer>  map_srv;
boost::shared_ptr<Advanced_Passage_finder> apf;
boost::shared_ptr<Passage_type> pt;

ros::Subscriber sub;
ros::Subscriber landing_sub;
ros::Publisher  pub_vel;

// These parameters are configured in 'airdrone_launch/param.yaml':
double vel_P = 0.0, vel_I = 0.0, vel_D = 0.0;
double ang_P = 0.0, ang_I = 0.0, ang_D = 0.0;

double target_dist     = 0.0;
double target_angl     = 0.0;
double target_height   = 0.0;
double movement_speed  = 0.0;
double move_epsilon    = 0.0;
double rot_epsilon     = 0.0;
double wall_ang_eps    = 0.0;
double angle_to_pass   = 0.0;
std::string fixed_frame;
std::string base_footprint_frame;
std::string base_stabilized_frame;
std::string input_lm_topic;
std::string output_vel_topic;
std::string pointcloud_frame;
std::string landing_marker_topic;


class ActionServer
{
public:
    ActionServer(ros::NodeHandle nh_) :
        as_move_along   (nh_, "MoveAlongAS",    boost::bind(&ActionServer::moveAlongCB,    this, _1), false),
        as_switch_wall  (nh_, "SwitchWallAS",   boost::bind(&ActionServer::switchWallCB,   this, _1), false),
        as_approach_door(nh_, "ApproachDoorAS", boost::bind(&ActionServer::approachDoorCB, this, _1), false),
        as_pass_door    (nh_, "PassDoorAS",     boost::bind(&ActionServer::passDoorCB,     this, _1), false),
        as_switch_side  (nh_, "SwitchSideAS",   boost::bind(&ActionServer::switchSideCB,   this, _1), false),
        as_approach_wall(nh_, "ApproachWallAS", boost::bind(&ActionServer::approachWallCB, this, _1), false),
        as_middle_pass  (nh_, "MiddlePassAS",   boost::bind(&ActionServer::middlePassCB,   this, _1), false),
        as_takeoff      (nh_, "TakeoffAS",      boost::bind(&ActionServer::takeoffCB,      this, _1), false),
        as_landing      (nh_, "LandingAS",      boost::bind(&ActionServer::landingCB,      this, _1), false),
		on_left_side (true)
    {
        as_move_along   .start();
        as_switch_wall  .start();
        as_approach_door.start();
        as_pass_door    .start();
        as_switch_side  .start();
        as_approach_wall.start();
        as_middle_pass  .start();
        as_takeoff      .start();
        as_landing      .start();
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

        if (!this->on_left_side) {
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
        if (this->on_left_side) {
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


    void approachDoorCB(const action_server::ApproachDoorGoalConstPtr  &goal)
    {

        action_server::ApproachDoorResult   result_;
        ros::Rate r(60);

        msn_srv->lock();
        msn_srv->set_target_angle(0);

        if (apf->passages.size() == 0) {
            ROS_ERROR("No passage in AD");
            as_approach_door.setAborted(result_);
                return;
        }

        pcl::PointXYZ pass_point_cmd = this->on_left_side ? apf->passages.at(0).cmd_left :
                                                            apf->passages.at(0).cmd_rght;
        if (isnan(pass_point_cmd.x) || isnan(pass_point_cmd.y)){
            ROS_ERROR("Wrong pass dist");
            as_approach_door.setAborted(result_);
            return;
        }

        pcl::PointXYZ pass_point_kin = this->on_left_side ? apf->passages.at(0).kin_left :
                                                            apf->passages.at(0).kin_rght;
        pcl::PointXYZ pass_point_kin_op = this->on_left_side ? apf->passages.at(0).kin_rght :
                                                               apf->passages.at(0).kin_left;
        Line_param *pass_line = apf->get_best_line(pass_point_kin, loc_srv->lm);
        davinci->draw_vec(pass_line->fdir_vec.kin.x, pass_line->fdir_vec.kin.y, 667, RED);

        if (pass_line == NULL) {
            ROS_ERROR("Pass_line wasn't found");
        }


        double pass_point_ang = 0;

        msn_srv->unlock();
        while (1) {
            msn_srv->lock();
            msn_srv->track();
            if (apf->passages.empty())
				break;
            pass_point_cmd = this->on_left_side ? apf->passages.at(0).cmd_left :
                                                  apf->passages.at(0).cmd_rght;
            pass_point_kin = this->on_left_side ? apf->passages.at(0).kin_left :
                                                  apf->passages.at(0).kin_rght;
            pass_point_ang = this->on_left_side ? apf->passages.at(0).left_ang :
                                                  apf->passages.at(0).rght_ang;
            pass_point_kin_op = this->on_left_side ? apf->passages.at(0).kin_rght :
                                                     apf->passages.at(0).kin_left;
            Line_param *pass_line_op = apf->get_best_line(pass_point_kin_op, loc_srv->lm);
            double vel = this->on_left_side ? -0.4 : 0.4;
			pass_line = apf->get_best_line(pass_point_kin, loc_srv->lm);
            msn_srv->ref_ang  = target_angl;
            msn_srv->ref_dist = target_dist;
			if (pass_line != NULL && !isnan(pass_point_cmd.x) && !isnan(pass_point_cmd.y)) {
                if (fabs(loc_srv->get_ref_wall()->angle - target_angl) < rot_epsilon) {
                    if (pt->recognize(apf, loc_srv->lm, this->on_left_side) == ortogonal) {
                        msn_srv->unlock();
                        break;
                    }
			        if (( this->on_left_side && pass_point_ang < 0) ||
			            (!this->on_left_side && pass_point_ang > 0)) {
			            double wall_to_pass_angle = fabs(loc_srv->get_ref_wall()->angle) -
			                                        fabs(pass_point_ang);
			            if (fabs(wall_to_pass_angle - angle_to_pass) < rot_epsilon) {
                            msn_srv->unlock();
                            break;
			            }
			            else if (fabs(wall_to_pass_angle) >= angle_to_pass) {
			                msn_srv->move_parallel(vel);
			            }
			            else {
			                msn_srv->move_parallel(-vel);
			            }
			        }
                    else {
                        msn_srv->move_parallel(vel);
                    }
			    }
                else {
                    msn_srv->move_parallel(0);
                }
			}
			else {
				ROS_ERROR("Pass_line wasn't found");
				msn_srv->unlock();
				break;
			}
			msn_srv->unlock();
			r.sleep();
		}
        msn_srv->lock();
        pass_point_kin_op = this->on_left_side ? apf->passages.at(0).kin_rght :
                                                 apf->passages.at(0).kin_left;
        pass_point_kin = this->on_left_side ? apf->passages.at(0).kin_left :
                                              apf->passages.at(0).kin_rght;

        pass_line = apf->get_best_line(pass_point_kin, loc_srv->lm);
        Line_param *pass_line_op = apf->get_best_opposite_line(pass_point_kin, pass_point_kin_op, loc_srv->lm);
        if (pt->recognize(apf, loc_srv->lm, this->on_left_side) == ortogonal) {
                loc_srv->track_wall(pass_line_op);
                msn_srv->unlock();
                result_.ortog_pass = true;
                as_approach_door.setSucceeded(result_);
                return;
        }

        pass_point_kin_op = this->on_left_side ? apf->passages.at(0).kin_rght :
                                                 apf->passages.at(0).kin_left;

        if (pt->recognize(apf, loc_srv->lm, this->on_left_side) == parrallel &&
            apf->passages.at(0).left_ang * apf->passages.at(0).rght_ang < 0) {
            msn_srv->unlock();
            result_.middle_pass = true;
            as_approach_door.setSucceeded(result_);
            return;
        }

        msn_srv->unlock();
        result_.success = true;
        as_approach_door.setSucceeded(result_);
        return;
    }


    void passDoorCB(const action_server::PassDoorGoalConstPtr  &goal) {
        action_server::PassDoorResult   result_;
        action_server::PassDoorFeedback feedback_;
        ros::Rate r(60);

        msn_srv->lock();

        // Check for appropriate input parameters
        pcl::PointXYZ pass_point_cmd (0, 0, 0);
        pcl::PointXYZ pass_point_kin (0, 0, 0);
        Line_param *pass_line = NULL;
        double side_sign = NAN;
        double rot_vel = NAN;
        if (apf->passages.size() == 0) {
			ROS_ERROR("No passage in PD");
			msn_srv->unlock();
            result_.success = true;
            as_pass_door.setSucceeded(result_);
            return;
		}
        else {
        	pass_point_cmd = this->on_left_side ? apf->passages.at(0).cmd_left :
                                                  apf->passages.at(0).cmd_rght;
            pass_point_kin = this->on_left_side ? apf->passages.at(0).kin_left :
                                                  apf->passages.at(0).kin_rght;
        	pass_line = apf->get_best_line(pass_point_kin, loc_srv->lm);
        	side_sign = (this->on_left_side) ? 1 : -1;
        	rot_vel = this->on_left_side ? 0.5 : -0.5;
        }

		if (pass_line == NULL || isnan(pass_point_cmd.x) || isnan(pass_point_cmd.y)){
			ROS_ERROR("Wrong pass distance");
			as_pass_door.setAborted(result_);
			return;
		}

		double vel[3] = {0.75, 0.75, 0.75};
		double angle[3] = {60, 60, 40};
		std::vector<int> stage_num;

		int start_pos_num = map_srv->track(pcl::PointXYZ (0, 0, 0)) - 1;
		pcl::PointXYZ vec (0, 0, 0);
		vec.x = pass_line->ldir_vec.cmd.x * side_sign * 1.5 * target_dist;
		vec.y = pass_line->ldir_vec.cmd.y * side_sign * 1.5 * target_dist;
		stage_num.push_back(map_srv->track(vec) - 1);


		vec.x = pass_line->fdir_vec.cmd.x * 1.75 * target_dist;
		vec.y = pass_line->fdir_vec.cmd.y * 1.75 * target_dist;
		stage_num.push_back(map_srv->track(vec) - 1);

		vec.x = -pass_line->ldir_vec.cmd.x * side_sign * 1.5 * target_dist;
        vec.y = -pass_line->ldir_vec.cmd.y * side_sign * 1.5 * target_dist;
        stage_num.push_back(map_srv->track(vec) - 1);

        msn_srv->unlock();
        for (int i = 0; i < stage_num.size(); i++) {
            msn_srv->lock();
            vec = pcl::PointXYZ(map_srv->tracked_points.at(stage_num.at(i)).x - map_srv->tracked_points.at(start_pos_num).x,
                                map_srv->tracked_points.at(stage_num.at(i)).y - map_srv->tracked_points.at(start_pos_num).y, 0);

            msn_srv->unlock();
            msn_srv->move (vec, vel[i], angle[i], rot_vel);
            bool all_done = false;
            while (!all_done) {
                msn_srv->lock();
                all_done = msn_srv->move_done && msn_srv->rot_done;
                msn_srv->unlock();
                r.sleep();
            }
        }

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

    	msn_srv->unlock();
    	result_.success = true;
		as_switch_side.setSucceeded(result_);
		return;
    }

    void approachWallCB(const action_server::ApproachWallGoalConstPtr  &goal){
        action_server::ApproachWallResult result_;
        ros::Rate r(60);

        while (true) {
            loc_srv->lock();
            msn_srv->track();
            msn_srv->ref_ang  = target_angl;
            msn_srv->ref_dist = target_dist;
            msn_srv->move_parallel(0);
            if (fabs(loc_srv->get_ref_wall()->distance - target_dist) <= move_epsilon)
                break;
            msn_srv->unlock();
            r.sleep();
        }

        msn_srv->unlock();
        result_.success = true;
        as_approach_wall.setSucceeded(result_);
        return;
    }

    void middlePassCB(const action_server::MiddlePassGoalConstPtr  &goal) {
        action_server::MiddlePassResult result_;
        ros::Rate r(60);
        msn_srv->lock();

        pcl::PointXYZ pass_point_kin_op = this->on_left_side ? apf->passages.at(0).kin_rght :
                                                               apf->passages.at(0).kin_left;
        Line_param *pass_line = apf->get_best_line(pass_point_kin_op, loc_srv->lm);
        double offset_x = 0, offset_y = 0;
        if (pass_line != NULL) {
            offset_x = pass_line->fdir_vec.cmd.x * target_dist / 2;
            offset_y = pass_line->fdir_vec.cmd.y * target_dist / 2;
        }
        int tmp = map_srv->track(pcl::PointXYZ(offset_x, offset_y, 0)) - 1;
        pcl::PointXYZ vec (apf->passages.at(0).cmd_middle.x - offset_x,
                           apf->passages.at(0).cmd_middle.y - offset_y, 0);
        tmp = map_srv->track(vec) - 1;

        msn_srv->unlock();
        msn_srv->move (vec, 0.6, 0, 0);
        bool all_done = false;
        while (!all_done) {
            msn_srv->lock();
            all_done = msn_srv->move_done && msn_srv->rot_done;
            msn_srv->unlock();
            r.sleep();
        }

        msn_srv->unlock();
        as_middle_pass.setSucceeded(result_);
        return;
    }


    void takeoffCB(const action_server::TakeoffGoalConstPtr  &goal) {
        action_server::TakeoffResult result_;
        ros::Rate r(60);


        msn_srv->set_height(target_height);

        bool all_done = false;
        while (!all_done) {
            msn_srv->lock();
            all_done = msn_srv->height_done;
            msn_srv->unlock();
            r.sleep();
        }

        result_.success = true;
        as_takeoff.setSucceeded(result_);
        return;
    }


    void landingCB(const action_server::LandingGoalConstPtr  &goal) {
        action_server::LandingResult result_;
        ros::Rate r(60);

        msn_srv->set_height(0.0);

        bool all_done = false;
        while (!all_done) {
            msn_srv->lock();
            all_done = msn_srv->on_floor;
            msn_srv->unlock();
            r.sleep();
        }

        result_.success = true;
        as_landing.setSucceeded(result_);
        return;
    }

private:
    // NodeHandle instance must be created before this line. Otherwise strange error may occur.
    actionlib::SimpleActionServer<action_server::MoveAlongAction>    as_move_along;
    actionlib::SimpleActionServer<action_server::SwitchWallAction>   as_switch_wall;
    actionlib::SimpleActionServer<action_server::ApproachDoorAction> as_approach_door;
    actionlib::SimpleActionServer<action_server::PassDoorAction>     as_pass_door;
    actionlib::SimpleActionServer<action_server::SwitchSideAction>   as_switch_side;
    actionlib::SimpleActionServer<action_server::ApproachWallAction> as_approach_wall;
    actionlib::SimpleActionServer<action_server::MiddlePassAction>   as_middle_pass;
    actionlib::SimpleActionServer<action_server::TakeoffAction>      as_takeoff;
    actionlib::SimpleActionServer<action_server::LandingAction>      as_landing;

    bool on_left_side; // Reference side of the wall
};






void landing_callback(const geometry_msgs::Point32 target) {
    //ROS_INFO("/: %f | %f | %f", target.x, target.y, target.z);
    //davinci->draw_vec(target.z, target.x, 66613, GREEN);
    //davinci->draw_vec(-target.x, -target.z, 66614, BLUE);
    if (isnan(target.x) || isnan(target.y) || isnan(target.z)) {
        ROS_ERROR("Action Server Node: landing_callback get NAN input");
        return;
    }
    pcl::PointXYZ land_pad (-target.x, -target.z, target.y);
    map_srv->add_land_pad(land_pad);
}

void callback(const ransac_slam::LineMap::ConstPtr& lines_msg)
{
    loc_srv->lock();  // Take control under location and motion servers
    if(lines_msg->number == 0) {
        ROS_ERROR("[action server]: no lines detected!");
    }

    loc_srv->spin_once(lines_msg);
    msn_srv->set_ref_wall(loc_srv->get_ref_wall());
    map_srv->spin_once();
    apf->renew(lines_msg);


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

    if (!msn_srv->move_done || !msn_srv->rot_done)
        msn_srv->move_step();

    msn_srv->altitude_step();
    if (msn_srv->on_floor && msn_srv->vert_vel < 0)
        msn_srv->clear_cmd();
    msn_srv->spin_once();


    
    double vlen = VectorMath::len(pcl::PointXYZ(msn_srv->base_cmd.linear.x,
                                                msn_srv->base_cmd.linear.y,
                                                msn_srv->base_cmd.linear.z));
    if (vlen > fabs(movement_speed)) {
        msn_srv->base_cmd.linear.x = msn_srv->base_cmd.linear.x * fabs(movement_speed) / vlen;
        msn_srv->base_cmd.linear.y = msn_srv->base_cmd.linear.y * fabs(movement_speed) / vlen;
        msn_srv->base_cmd.linear.z = msn_srv->base_cmd.linear.z * fabs(movement_speed) / vlen;
    }

    davinci->draw_vec_cmd(msn_srv->base_cmd, 10, GOLD);

    pub_vel.publish(msn_srv->base_cmd);
    msn_srv->clear_cmd();
    loc_srv->unlock();  // Return control to task callback handlers
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
    if (!nh.getParam("distance_to_wall", target_dist))     ROS_ERROR("Failed to get param 'distance_to_wall'");
    if (!nh.getParam("angle_to_wall",    target_angl))     ROS_ERROR("Failed to get param 'angle_to_wall'");
    if (!nh.getParam("base_height",      target_height))   ROS_ERROR("Failed to get param 'base_height'");
    if (!nh.getParam("movement_speed",   movement_speed))  ROS_ERROR("Failed to get param 'movement_speed'");
    if (!nh.getParam("move_epsilon",  move_epsilon))  ROS_ERROR("Failed to get param 'move_epsilon'");
    if (!nh.getParam("angle_to_pass", angle_to_pass)) ROS_ERROR("Failed to get param 'angle_to_pass'");
    if (!nh.getParam("rot_epsilon",   rot_epsilon))   ROS_ERROR("Failed to get param 'rot_epsilon'");
    if (!nh.getParam("wall_ang_eps",  wall_ang_eps))  ROS_ERROR("Failed to get param 'wall_ang_eps'");
    if (!nh.getParam("action_server/fixed_frame",            fixed_frame))           fixed_frame  = "/odom";
    if (!nh.getParam("action_server/base_footprint_frame",   base_footprint_frame))  base_footprint_frame  = "/base_footprint_";
    if (!nh.getParam("action_server/base_stabilized_frame",  base_stabilized_frame)) base_stabilized_frame = "/base_stabilized";
    if (!nh.getParam("action_server/pointcloud_frame",  pointcloud_frame)) pointcloud_frame = "/pointcloud_frame";
    if (!nh.getParam("action_server/input_lm_topic",   input_lm_topic)) input_lm_topic = "/ransac_slam/lm";
    if (!nh.getParam("action_server/output_vel_topic", output_vel_topic)) output_vel_topic = "/cmd_vel_2";
    if (!nh.getParam("action_server/landing_marker_topic", landing_marker_topic)) landing_marker_topic = "landing_marker";

    sub     = nh.subscribe<ransac_slam::LineMap>   (input_lm_topic,   1, callback);
    landing_sub = nh.subscribe<geometry_msgs::Point32>  (landing_marker_topic,   1, landing_callback);
    pub_vel = nh.advertise<geometry_msgs::Twist >  (output_vel_topic, 1);

    davinci   = boost::shared_ptr<DaVinci>        (new DaVinci (nh));
    mutex_ptr = boost::shared_ptr<boost::mutex>   (new boost::mutex);
    loc_srv   = boost::shared_ptr<LocationServer> (new LocationServer(mutex_ptr));
    map_srv   = boost::shared_ptr<MappingServer>  (new MappingServer ());
    msn_srv   = boost::shared_ptr<MotionServer>   (new MotionServer  (mutex_ptr, map_srv));
    apf       = boost::shared_ptr<Advanced_Passage_finder> (new Advanced_Passage_finder());
    pt        = boost::shared_ptr<Passage_type>   (new Passage_type ());

    msn_srv->set_pid_ang(ang_P, ang_I, ang_D);
    msn_srv->set_pid_vel(vel_P, vel_I, vel_D);

    sub     = nh.subscribe<ransac_slam::LineMap>  (input_lm_topic,   1, callback);
    pub_vel = nh.advertise<geometry_msgs::Twist > (output_vel_topic, 1);

    ActionServer action_server(nh);


    ros::spin ();
    return 0;
};
