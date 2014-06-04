#include <cstdio>
#include <iostream>
#include <vector>
#include <cmath>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#include "pcl_ros/point_cloud.h"
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



boost::shared_ptr<boost::mutex> mutex_ptr;
boost::shared_ptr<LocationServer> loc_srv;
boost::shared_ptr<MotionServer>   msn_srv;
boost::shared_ptr<MappingServer>   map_srv;
boost::shared_ptr<Advanced_Passage_finder> apf;

std::string input_topic;
std::string output_topic_mrk;
std::string output_topic_vel;

ros::Subscriber sub;
ros::Publisher  pub_mrk;
ros::Publisher  pub_vel;

double vel_P = 0.0, vel_I = 0.0, vel_D = 0.0;
double ang_P = 0.0, ang_I = 0.0, ang_D = 0.0;

double target_dist = 0.0;
double target_angl = 0.0;
double movement_speed = 0.0;
double move_epsilon = 0.1;
double angle_of_kinect = 0.0;


visualization_msgs::Marker line_list;




// Draw in kinect coordinates
enum POINT_COLOR {RED, GREEN, BLUE};
void draw_point(double x, double y, int id, POINT_COLOR color)
{
    if (isnan(x) || isnan(y)) return;

    visualization_msgs::Marker marker;

    marker.header.frame_id = "/camera_link";
    marker.ns = "basic_shapes";
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;

    marker.pose.position.x =  y;
    marker.pose.position.y = -x;
    marker.pose.position.z = 0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    // Set the color -- be sure to set alpha to something non-zero!
    switch (color)
    {
    case RED:
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        break;
    case GREEN:
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        break;
    case BLUE:
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        break;
    default:
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
    }

    marker.color.a = 0.6;
    marker.lifetime = ros::Duration(0.1);
    marker.header.stamp = ros::Time::now();
    // Publish the marker
    pub_mrk.publish(marker);
};


void add_e(double x, double y)
{
    geometry_msgs::Point p;
    p.y = 0;
    p.z = 0;
    p.x = 0;

    line_list.points.push_back(p);

    p.y = -x;
    p.z =  0;
    p.x =  y;

    line_list.points.push_back(p);
};


void add_l(Line_param *lp)
{
    if(lp == NULL) return;

    const int length = 40;
    geometry_msgs::Point p;
    p.y = -lp->fdir_vec.kin.x * lp->distance + lp->ldir_vec.kin.x * length / 2;
    p.z =  0;
    p.x =  lp->fdir_vec.kin.y * lp->distance - lp->ldir_vec.kin.y * length / 2;
    line_list.points.push_back(p);
    p.y =  p.y - lp->ldir_vec.kin.x * length;
    p.z =  0;
    p.x =  p.x + lp->ldir_vec.kin.y * length;
    line_list.points.push_back(p);
};






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

            if (this->locate_passage()) { // Found door in some wall
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


    bool locate_passage()
    {
        // WARNING! mutex lock/unlock should be handled outside the function!
        Passage_finder pf(*(loc_srv->get_ref_wall()));

        if(pf.passage.size() <= 0)
            return false;

        return true;
    }

    void move(pcl::PointXY dir, double vel)
    {
        ros::Rate r(60);
        pcl::PointXY pos = map_srv->get_positon();
        pcl::PointXY end, vec;
        end.x = pos.x + dir.x;
        end.y = pos.y + dir.y;
        ROS_ERROR("Start: (cmd) %3f\t %3f", pos.x, pos.y);
        ROS_ERROR("End:   (cmd) %3f\t %3f", end.x, end.y);
        ROS_ERROR("--------------------------------");
        while (true)
        {
            msn_srv->lock();
            pos = map_srv->get_positon();
            vec.x = end.x - pos.x;
            vec.y = end.y - pos.y;
            double len = sqrt (vec.x * vec.x + vec.y * vec.y);
            if (len < move_epsilon) {
                msn_srv->unlock();
                //ROS_ERROR("Move done");
                break;
            }
            vec.x /= len;
            vec.y /= len;

            msn_srv->buf_cmd.linear.x = vec.x*0.1;
            msn_srv->buf_cmd.linear.y = vec.y*0.1;
            //ROS_INFO("Pos: x: %f\t %f\n  Vec: x: %f\t %f\n  End: x: %f\t %f", pos.x, pos.y, msn_srv->buf_cmd.linear.x, msn_srv->buf_cmd.linear.y, end.x, end.y);
            msn_srv->unlock();
            r.sleep();
        }
    }

    void approachDoorCB(const action_server::ApproachDoorGoalConstPtr  &goal)
    {
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
        action_server::ApproachDoorResult   result_;
        ros::Rate r(60);

        msn_srv->untrack();
        pcl::PointXY vec ;
        while(true) {
            vec.x =  0.0;
            vec.y =  0.5;
            move (vec, 0.4);

            vec.x = -0.5;
            vec.y =  0.0;
            move (vec, 0.4);

            vec.x =  0.0;
            vec.y = -0.5;
            move (vec, 0.4);

            vec.x =  0.5;
            vec.y =  0.0;
            move (vec, 0.4);
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

    line_list.header.stamp = ros::Time::now();
    line_list.points.clear();

    if(cloud->points.size() < 10) {
        ROS_ERROR("AS: Cloud size = %lu", cloud->points.size());
    }

    loc_srv->spin_once(cloud);
    msn_srv->set_ref_wall(loc_srv->get_ref_wall());
    apf->renew(cloud);

    add_l(loc_srv->get_ref_wall());
    add_l(loc_srv->get_crn_wall_left());
    add_l(loc_srv->get_crn_wall_rght());

    for (int i = 0; i < apf->passages.size(); ++i) {
        draw_point(apf->passages.at(i).kin_left.x,   apf->passages.at(i).kin_left.y,   i*3 + 0, RED);
        draw_point(apf->passages.at(i).kin_middle.x, apf->passages.at(i).kin_middle.y, i*3 + 0, GREEN);
        draw_point(apf->passages.at(i).kin_rght.x,   apf->passages.at(i).kin_rght.y,   i*3 + 0, RED);
    }



    msn_srv->spin_once();
    pub_vel.publish(msn_srv->base_cmd);
    pub_mrk.publish(line_list);
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
  if (!nh.getParam("angle_to_wall", target_angl)) ROS_ERROR("Failed to get param 'angle_to_wall'");
  if (!nh.getParam("movement_speed", movement_speed)) ROS_ERROR("Failed to get param 'movement_speed'");
  if (!nh.getParam("angle_of_kinect", angle_of_kinect)) ROS_ERROR("Failed to get param 'angle_of_kinect'");



  input_topic      = nh.resolveName("/shrinker/depth/laser_points");
  output_topic_mrk = nh.resolveName("visualization_marker");
  output_topic_vel = nh.resolveName("/cmd_vel_2");

  sub     = nh.subscribe<pcl::PointCloud<pcl::PointXYZ> > (input_topic,      1, callback);
  pub_mrk = nh.advertise<visualization_msgs::Marker >     (output_topic_mrk, 5 );
  pub_vel = nh.advertise<geometry_msgs::Twist >           (output_topic_vel, 1 );

  mutex_ptr = boost::shared_ptr<boost::mutex>   (new boost::mutex);
  loc_srv   = boost::shared_ptr<LocationServer> (new LocationServer(mutex_ptr));
  msn_srv   = boost::shared_ptr<MotionServer>   (new MotionServer  (mutex_ptr));
  map_srv   = boost::shared_ptr<MappingServer>  (new MappingServer (nh, "/ground_truth_to_tf/pose"));
  apf       = boost::shared_ptr<Advanced_Passage_finder> (new Advanced_Passage_finder());

  msn_srv->set_pid_ang(ang_P, ang_I, ang_D);
  msn_srv->set_pid_vel(vel_P, vel_I, vel_D);

  ActionServer action_server(nh);





  line_list.header.frame_id = "/camera_link";
  line_list.ns = "lines_ns";
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.id = 0;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.scale.x = 0.03;
  line_list.color.r = 0.0;
  line_list.color.g = 0.0;
  line_list.color.b = 1.0;
  line_list.color.a = 0.3;
  line_list.lifetime = ros::Duration(0.2);



  ros::spin ();

  return 0;
}
