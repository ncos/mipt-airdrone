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





class ActionServer
{
public:
    ActionServer(ros::NodeHandle nh_) :
        as_move_along   (nh_, "MoveAlongAS",     boost::bind(&ActionServer::moveAlongCB,    this, _1), false),
        as_switch_wall  (nh_, "SwitchWallAS",    boost::bind(&ActionServer::switchWallCB,   this, _1), false),
        as_approach_door(nh_, "ApproachDoorAS",  boost::bind(&ActionServer::approachDoorCB, this, _1), false),
        as_pass_door    (nh_, "PassDoorAS",      boost::bind(&ActionServer::passDoorCB,     this, _1), false)
    {
        as_move_along   .start();
        as_switch_wall  .start();
        as_approach_door.start();
        as_pass_door    .start();
    }

    ~ActionServer(void) {}


    void moveAlongCB (const action_server::MoveAlongGoalConstPtr     &goal)
    {
        action_server::MoveAlongResult      result_;
        action_server::MoveAlongFeedback feedback_;
        ros::Rate r(60);

        while (true) {
            loc_srv->lock();
            msn_srv->track();
            msn_srv->ref_ang = target_angl; // Face along the wall

            //
            // Stop cases:
            //
            if (goal->vel == 0) {
                // No velocity no movement. This is actually a misuse case
                result_.error = true;
                msn_srv->move_parallel(0); // Stop movement
                loc_srv->unlock();         // Releasing mutex
                as_move_along.setSucceeded(result_); // Instantly leave the function
                return;
            }

            if (loc_srv->obstacle_detected_left() && goal->vel > 0) {
                // Found wall on the left and moving towards
                result_.left = true;
                msn_srv->move_parallel(0); // Stop movement
                loc_srv->unlock();         // Releasing mutex
                as_move_along.setSucceeded(result_); // Instantly leave the function
                return;
            }

            if (loc_srv->obstacle_detected_rght() && goal->vel < 0) { // Found wall on the left and moving towards
                result_.rght = true;
                msn_srv->move_parallel(0); // Stop movement
                loc_srv->unlock();         // Releasing mutex
                as_move_along.setSucceeded(result_); // Instantly leave the function
                return;
            }

            if (apf->passages.size() > 0) { // Found door in some wall
                result_.found = true;
                msn_srv->move_parallel(0); // Stop movement
                msn_srv->set_angles_current(); // And rotation
                loc_srv->unlock();         // Releasing mutex
                as_move_along.setSucceeded(result_); // Instantly leave the function
                return;
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
                msn_srv->move_parallel(goal->vel);
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

            feedback_.vel = goal->vel; // TODO: Publish current velocity (from sensors), not input
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
                as_switch_wall.setAborted(result_);
                return;
            }
        }

        loc_srv->unlock();
    }

    //TODO: Add cases for NAN input
    bool move(pcl::PointXY dir, double vel)
    {
        if (isnan(dir.x) || isnan(dir.y)) {
            return false;
        }
        ros::Rate r(60);
        double prev_angl = map_srv->get_global_angle();
        pcl::PointXY prev_pos = map_srv->get_global_positon();
        pcl::PointXY target;
        target.x = dir.x;
        target.y = dir.y;

        while (true)
        {
            msn_srv->lock();
            msn_srv->untrack();

            double current_angl      = map_srv->get_global_angle();
            pcl::PointXY current_pos = map_srv->get_global_positon();
            double delta_angl  = map_srv->diff(current_angl, prev_angl);
            pcl::PointXY shift = map_srv->diff(current_pos, prev_pos);
            prev_angl = current_angl;
            prev_pos  = current_pos;

            // "-" delta_angl here cuz we need to rotate target destination backwards, to compensate the positive drone rotation
            target = map_srv->rotate(target, -delta_angl);
            target.x = target.x - shift.x;
            target.y = target.y - shift.y;

            double len = sqrt (target.x * target.x + target.y * target.y);
            if (len < move_epsilon) {
                msn_srv->unlock();
                break;
            }


            msn_srv->buf_cmd.linear.x = target.x * vel / len;
            msn_srv->buf_cmd.linear.y = target.y * vel / len;

            davinci->draw_point_cmd(target.x, target.y, 666, GOLD);

            msn_srv->unlock();
            r.sleep();
        }
        return true;
    }

    void approachDoorCB(const action_server::ApproachDoorGoalConstPtr  &goal)
    {
        /*
        action_server::ApproachDoorResult   result_;
        ros::Rate r(60);

        msn_srv->lock();
        msn_srv->set_angles_current();

        double alpha = loc_srv->get_ref_wall()->angle;
        ROS_INFO ("Alpha: %f", alpha);
        double pass_x = apf->passages.at(0).cmd_left.x;
        double pass_y = apf->passages.at(0).cmd_left.y;
        if (isnan(pass_x) || isnan(pass_y)){
            as_approach_door.setAborted(result_);
            return;
        }
        double pass_dist = sqrt (pass_x * pass_x + pass_y * pass_y);
        ROS_INFO ("Pass: x %f\t y %f\t len %f", pass_x, pass_y, pass_dist);
        double targ_len = pass_dist * cos ((90 - alpha - apf->passages.at(0).left_ang) * M_PI / 180) + passage_width / 2;
        ROS_INFO("Angl: %f", 90 - alpha - apf->passages.at(0).left_ang);
        double targ_x = targ_len * sin (alpha * M_PI / 180);
        double targ_y = targ_len * cos (alpha * M_PI / 180);

        ROS_INFO ("Targ: x %f\t y %f\t len %f", targ_x, targ_y, targ_len);



        msn_srv->untrack();
        pcl::PointXY vec ;
        vec.x =  targ_x;
        vec.y =  targ_y;
        msn_srv->unlock();
        if (move (vec, 0.4) == false) {
            as_approach_door.setAborted(result_);
            return;
        }

        ROS_ERROR("Approach done");

        msn_srv->lock(); // Location and motion servers are bound to the same mutex
        msn_srv->move_parallel(0); // Stop movement
        msn_srv->set_angles_current(); // And rotation
        msn_srv->unlock();

        as_approach_door.setSucceeded(result_);
        return;
        */



        /*
        action_server::ApproachDoorResult   result_;
        ros::Rate r(60);

        while (true) {

            if (as_approach_door.isPreemptRequested() || !ros::ok()) {
                ROS_INFO("Full stop called at approach the wall callback");
                msn_srv->lock(); // Location and motion servers are bound to the same mutex
                msn_srv->move_parallel(0); // Stop movement
                msn_srv->set_angles_current(); // And rotation
                msn_srv->unlock();
                as_approach_door.setPreempted();
                return;
            }

            loc_srv->lock();

            if (apf->passages.size() == 0) {
                ROS_WARN("No passage here (mistaken or lost)! Continue searching...");
                loc_srv->unlock();
                result_.success = false;
                as_approach_door.setAborted(result_);
                return;
            }
            else {
                // TODO: Always choosing the "0" passage. Maybe better the closest?

                as_approach_door.setSucceeded(result_);
                return;
            }


            loc_srv->unlock();
            r.sleep();
        }
        */

        action_server::ApproachDoorResult result_;
        ros::Rate r(60);

        msn_srv->lock();
        msn_srv->untrack();
        msn_srv->unlock();

        pcl::PointXY vec ;
        while(true) {
            ROS_WARN("Commensing shape dance!");
            vec.x =  1;
            vec.y =  0;
            move (vec, 0.2);

            vec.x = -1;
            vec.y = 0;
            move (vec, 0.2);
        }
        as_approach_door.setSucceeded(result_);
        return;

    }


    void passDoorCB(const action_server::PassDoorGoalConstPtr  &goal) {
        // TODO: Change for door on left sight
        action_server::PassDoorResult   result_;
        action_server::PassDoorFeedback feedback_;

        msn_srv->lock();

        double door_width = target_dist;

        if (loc_srv->obstacle_detected_rght() == false) {
            msn_srv->unlock();
            while (true) {
                msn_srv->lock();
                msn_srv->ref_ang  = 90;
                if (loc_srv->obstacle_detected_rght()) {
                    break;
                }
                msn_srv->unlock();
            }
        }
        if (loc_srv->obstacle_detected_rght()) {
            door_width = loc_srv->get_crn_wall_rght()->distance;
            loc_srv->track_wall(loc_srv->get_crn_wall_rght());
        }
        else
        {
            ROS_ERROR("Pass Door failed: no right wall");
            msn_srv->unlock();
            result_.success = false;
            as_pass_door.setAborted(result_);
            return;
        }

        msn_srv->unlock();
        while (true) {
            msn_srv->lock();
            msn_srv->ref_ang  = 15;
            msn_srv->ref_dist = door_width;
            if (fabs (loc_srv->get_ref_wall()->angle - msn_srv->ref_ang) < 3) {
                msn_srv->unlock();
                break;
            }
            msn_srv->unlock();
        }
        while (true) {
            msn_srv->lock();
            msn_srv->move_parallel(goal->vel);
            if (loc_srv->obstacle_detected_rght()) {
                break;
            }
            msn_srv->unlock();
        }
        msn_srv->unlock();
        result_.success = true;
        as_pass_door.setSucceeded(result_);
        return;
    }


private:
    // NodeHandle instance must be created before this line. Otherwise strange error may occur.
    actionlib::SimpleActionServer<action_server::MoveAlongAction>    as_move_along;
    actionlib::SimpleActionServer<action_server::SwitchWallAction>   as_switch_wall;
    actionlib::SimpleActionServer<action_server::ApproachDoorAction> as_approach_door;
    actionlib::SimpleActionServer<action_server::PassDoorAction>     as_pass_door;
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
        davinci->draw_point(apf->passages.at(i).kin_rght.x,   apf->passages.at(i).kin_rght.y,   i*3 + 2, RED);

        davinci->draw_vec(apf->passages.at(i).kin_left.x,   apf->passages.at(i).kin_left.y,   i*3 + 0, RED);
        davinci->draw_vec(apf->passages.at(i).kin_middle.x, apf->passages.at(i).kin_middle.y, i*3 + 1, GREEN);
        davinci->draw_vec(apf->passages.at(i).kin_rght.x,   apf->passages.at(i).kin_rght.y,   i*3 + 2, RED);
    }

    msn_srv->spin_once();
    davinci->draw_vec_cmd(msn_srv->base_cmd, 10, GOLD);
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
