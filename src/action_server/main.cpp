#include <cstdio>
#include <iostream>
#include <vector>
#include <cmath>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
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
#include <action_server/RotationAction.h>
#include <action_server/ApproachWallAction.h>

// Mutex is necessary to avoid control races between task handlers and cloud callback
// cloud callback (or callback) is needed to renew information about point cloud and external sensors
#include <boost/thread/mutex.hpp>



boost::shared_ptr<boost::mutex> mutex_ptr;
boost::shared_ptr<LocationServer> loc_srv;
boost::shared_ptr<MotionServer>	  msn_srv;

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

visualization_msgs::Marker line_list;





// Draw in kinect coordinates
void draw_point(double x, double y, int id)
{
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
	marker.color.r = 1.0f;
	marker.color.g = 0.0f;
	marker.color.b = 0.0f;
	marker.color.a = 0.6;

	marker.lifetime = ros::Duration(0.2);

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
		as_rotation 	(nh_, "RotationAS", 	 boost::bind(&ActionServer::rotationCB,     this, _1), false),
		as_approach_wall(nh_, "ApproachWallAS",  boost::bind(&ActionServer::approachWallCB, this, _1), false)
	{
        as_move_along.   start();
        as_switch_wall.  start();
		as_rotation.	 start();
        as_approach_wall.start();
	}

	~ActionServer(void) {}


	void rotationCB(const action_server::RotationGoalConstPtr 		&goal)
	{
		ros::Rate r(60);
		action_server::RotationFeedback feedback_;
		action_server::RotationResult result_;

		// TODO: Rewrite the function as it uses obsolete interfaces
		msn_srv->rotate(goal->angle);
		double current_yaw = loc_srv->get_yaw();
		double target_yaw  = loc_srv->get_yaw() + goal->angle;

		mutex_ptr->lock();
		ROS_ERROR("RotationCB");
		mutex_ptr->unlock();

		feedback_.percentage = 0;
		while(feedback_.percentage < 80) {
			mutex_ptr->lock();
			ROS_ERROR("RotationCB - processing");
			mutex_ptr->unlock();


			if (as_rotation.isPreemptRequested() || !ros::ok()) {
				ROS_INFO("rotation: Preempted");
				as_rotation.setAborted();
			    break;
			}
			feedback_.percentage = 100 - fabs(loc_srv->get_ref_wall()->angle - msn_srv->ref_ang)/goal->angle * 100;
			as_rotation.publishFeedback(feedback_);
			r.sleep();
		}
		result_.success = true;
		result_.percentage = feedback_.percentage;
		as_rotation.setSucceeded(result_);
	}




	void moveAlongCB (const action_server::MoveAlongGoalConstPtr 	&goal)
	{
		action_server::MoveAlongResult 	 result_;
        action_server::MoveAlongFeedback feedback_;
		ros::Rate r(60);

		while (true) {
			loc_srv->lock();
			msn_srv->ref_ang = target_angl; // Face along the wall

			//
			// Stop cases:
			//
	        if(goal->vel == 0) {
	            // No velocity no movement. This is actually a misuse case
	            result_.error = true;
                msn_srv->move_parallel(0); // Stop movement
                loc_srv->unlock();         // Releasing mutex
                as_move_along.setSucceeded(result_); // Instantly leave the function
                return;
	        }

			if(loc_srv->obstacle_detected_left() && goal->vel > 0) {
			    // Found wall on the left and moving towards
				result_.left = true;
				msn_srv->move_parallel(0); // Stop movement
				loc_srv->unlock();         // Releasing mutex
				as_move_along.setSucceeded(result_); // Instantly leave the function
				return;
			}

            if(loc_srv->obstacle_detected_rght() && goal->vel < 0) { // Found wall on the left and moving towards
                result_.rght = true;
                msn_srv->move_parallel(0); // Stop movement
                loc_srv->unlock();         // Releasing mutex
                as_move_along.setSucceeded(result_); // Instantly leave the function
                return;
            }

            if(false) { // Found door in some wall
                result_.found = true;
                msn_srv->move_parallel(0); // Stop movement
                loc_srv->unlock();         // Releasing mutex
                as_move_along.setSucceeded(result_); // Instantly leave the function
                return;
            }

            if (as_move_along.isPreemptRequested() || !ros::ok()) {
                msn_srv->move_parallel(0); // Stop movement
                loc_srv->unlock();         // Releasing mutex
                as_move_along.setAborted();
                return;
            }

            //
            // Movement sustain
            //
			if(fabs(loc_srv->get_ref_wall()->angle - msn_srv->ref_ang) < 10 ) {
				msn_srv->ref_ang  = target_angl;
				msn_srv->ref_dist = target_dist;
				msn_srv->move_parallel(goal->vel);
			}
			loc_srv->unlock();

			feedback_.vel = goal->vel; // Publish current velocitu (TODO)
			as_move_along.publishFeedback(feedback_);
			r.sleep();
		}
	}

	void switchWallCB(const action_server::SwitchWallGoalConstPtr  &goal)
	{
        action_server::SwitchWallResult   result_;

        result_.success = false;
        loc_srv->lock();

        if(goal->left) {
            if(loc_srv->obstacle_detected_left()) {
                loc_srv->track_wall(loc_srv->get_crn_wall_left());
                loc_srv->unlock();
                result_.success = true;
                as_switch_wall.setSucceeded(result_);
                return;
            }
            else
            {
                // TODO: Handle errors
                as_switch_wall.setAborted();
                return;
            }
        }
        if(goal->rght) {
            if(loc_srv->obstacle_detected_rght()) {
                loc_srv->track_wall(loc_srv->get_crn_wall_rght());
                loc_srv->unlock();
                result_.success = true;
                as_switch_wall.setSucceeded(result_);
                return;
            }
            else
            {
                // TODO: Handle errors
                as_switch_wall.setAborted();
                return;
            }
        }
        if(goal->any) {
            if(loc_srv->obstacle_detected_left()) {
                loc_srv->track_wall(loc_srv->get_crn_wall_left());
                loc_srv->unlock();
                result_.success = true;
                as_switch_wall.setSucceeded(result_);
                return;
            }
            if(loc_srv->obstacle_detected_rght()) {
                loc_srv->track_wall(loc_srv->get_crn_wall_rght());
                loc_srv->unlock();
                result_.success = true;
                as_switch_wall.setSucceeded(result_);
                return;
            }
            else
            {
                // TODO: Handle errors
                as_switch_wall.setAborted();
                return;
            }
        }

        loc_srv->unlock();
	}


	void locate_passage()
	{
	    loc_srv->lock();
	    Passage_finder pf(*(loc_srv->get_ref_wall()));
	    loc_srv->unlock();

        for (int i = 0; i < pf.passage.size(); i++) {
            draw_point(pf.passage.at(i).kin_middle.x, pf.passage.at(i).kin_middle.y, i*3 + 0);
            draw_point(pf.passage.at(i).kin_left.x,   pf.passage.at(i).kin_left.y,   i*3 + 1);
            draw_point(pf.passage.at(i).kin_rght.x,   pf.passage.at(i).kin_rght.y,   i*3 + 2);
        }
	}



    void approachWallCB(const action_server::ApproachWallGoalConstPtr  &goal)
    {
        action_server::ApproachWallResult   result_;
        action_server::ApproachWallFeedback feedback_;
        ros::Rate r(60);

        // TODO: Rewrote this function so it can accept passage coordinates instead of searching for it again
        while (true) {

            if (as_approach_wall.isPreemptRequested() || !ros::ok()) {
                ROS_INFO("Full stop called at approach the wall callback");
                msn_srv->lock(); // Location and motion servers are bound to the same mutex
                msn_srv->move_parallel(0); // Stop movement
                msn_srv->set_angles_current(); // And rotation
                msn_srv->unlock();
                as_approach_wall.setAborted();
                return;
            }

            loc_srv->lock();
            Passage_finder pf(*(loc_srv->get_ref_wall()));
            for (int i = 0; i < pf.passage.size(); i++) {
                draw_point(pf.passage.at(i).kin_middle.x, pf.passage.at(i).kin_middle.y, i*3 + 0);
                draw_point(pf.passage.at(i).kin_left.x,   pf.passage.at(i).kin_left.y,   i*3 + 1);
                draw_point(pf.passage.at(i).kin_rght.x,   pf.passage.at(i).kin_rght.y,   i*3 + 2);
            }

            if (pf.passage.size() == 0) {
                ROS_WARN("No passage here (mistaken or lost)!");
                // TODO: Handle errors (no passage, continue search)
                loc_srv->unlock();
                as_approach_wall.setAborted();
                return;
            }
            else {
                msn_srv->ref_ang = loc_srv->get_ref_wall()->angle + pf.passage.at(0).rght_ang + 15;

                // TODO: Always choosing the "0" passage. Maybe better the closest? (Or something smarter than current decision)
                double err_shift = loc_srv->get_ref_wall()->ldir_vec.kin.x * pf.passage.at(0).kin_middle.x +
                                   loc_srv->get_ref_wall()->ldir_vec.kin.y * pf.passage.at(0).kin_middle.y;

                msn_srv->move_parallel(-vel_P * movement_speed * err_shift);
                msn_srv->ref_dist = 1.2;
                if (fabs(err_shift) < 0.1) {
                    msn_srv->ref_ang = 55;
                    loc_srv->unlock(); // Release the mutex
                    result_.success = true;
                    result_.x = pf.passage.at(0).kin_middle.x; // TODO: Check that
                    result_.y = pf.passage.at(0).kin_middle.y; // TODO: Check that
                    as_approach_wall.setSucceeded(result_);
                    return;
                }
            }

            feedback_.x = pf.passage.at(0).kin_middle.x; // TODO: Check that
            feedback_.y = pf.passage.at(0).kin_middle.y; // TODO: Check that
            loc_srv->unlock();
            as_approach_wall.publishFeedback(feedback_);
            r.sleep();
        }
    }



private:
	// NodeHandle instance must be created before this line. Otherwise strange error may occur.
    actionlib::SimpleActionServer<action_server::MoveAlongAction>    as_move_along;
    actionlib::SimpleActionServer<action_server::SwitchWallAction>   as_switch_wall;
	actionlib::SimpleActionServer<action_server::RotationAction>  	 as_rotation;
    actionlib::SimpleActionServer<action_server::ApproachWallAction> as_approach_wall;
};








void callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
    // Take control upon location and motion servers
	loc_srv->lock();

    line_list.header.stamp = ros::Time::now();
    line_list.points.clear();

    loc_srv->spin_once(cloud);
    msn_srv->set_ref_wall(loc_srv->get_ref_wall());

/*
// Actually pass through door
    if(in_front_of_passage)
    {
        if(fabs(loc_srv.get_ref_wall()->angle - msn_srv.ref_ang) < 5 ) {
        	if(!loc_srv.obstacle_detected_rght() && ready_to_enter) ROS_ERROR("CANT SEE WALL ON THE RIGHT");
        }


    	if( loc_srv.obstacle_detected_rght() && ready_to_enter) {
    		ROS_INFO("in_front_of_passage");
    		loc_srv.track_wall(loc_srv.get_crn_wall_rght());
    	    msn_srv.set_ref_wall(loc_srv.get_ref_wall());
    		msn_srv.ref_dist = loc_srv.get_ref_wall()->distance;
    		msn_srv.ref_ang = 20;
    		ready_to_enter = false;
    	}


    	if(!ready_to_enter) {
			move_along(movement_speed, 20, msn_srv.ref_dist);


			if(loc_srv.obstacle_detected_rght()) {
				ROS_ERROR("ON THE RIGHT");

				left_wall_with_door = false;
				in_front_of_passage = false;
				unexplored_wall = false;
			}
    	}
    }
*/




    add_l(loc_srv->get_ref_wall());
    add_l(loc_srv->get_crn_wall_left());
    add_l(loc_srv->get_crn_wall_rght());






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



  input_topic      = nh.resolveName("/shrinker/depth/laser_points");
  output_topic_mrk = nh.resolveName("visualization_marker");
  output_topic_vel = nh.resolveName("/cmd_vel_2");

  sub     = nh.subscribe<pcl::PointCloud<pcl::PointXYZ> > (input_topic,  1, callback);
  pub_mrk = nh.advertise<visualization_msgs::Marker >     (output_topic_mrk, 5 );
  pub_vel = nh.advertise<geometry_msgs::Twist >           (output_topic_vel, 1 );

  mutex_ptr = boost::shared_ptr<boost::mutex>   (new boost::mutex);
  loc_srv   = boost::shared_ptr<LocationServer> (new LocationServer(mutex_ptr));
  msn_srv   = boost::shared_ptr<MotionServer>   (new MotionServer  (mutex_ptr));

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
