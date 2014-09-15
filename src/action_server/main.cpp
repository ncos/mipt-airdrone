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
#include <action_server/ApproachWallAction.h>

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


double rot_epsilon = 5;
double wall_ang_eps = 5;
double angle_to_pass = 30;


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
		on_left_side (true)
    {
        as_move_along   .start();
        as_switch_wall  .start();
        as_approach_door.start();
        as_pass_door    .start();
        as_switch_side  .start();
        as_approach_wall.start();
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


            double current_angl       = map_srv->get_global_angle();
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
            ROS_ERROR("No passage in AD");
            as_approach_door.setAborted(result_);
                return;
        }

        pcl::PointXYZ pass_point_kin_op = this->on_left_side ? apf->passages.at(0).kin_rght :
                                                               apf->passages.at(0).kin_left;
        // Enable for parallel walls
/*
        if (!isnan(apf->passages.at(0).cmd_middle.x) && !isnan(apf->passages.at(0).cmd_middle.y)) {
        	ROS_INFO("Middle pass");
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
*/
        pcl::PointXYZ pass_point_cmd = this->on_left_side ? apf->passages.at(0).cmd_left :
                                                            apf->passages.at(0).cmd_rght;
        if (isnan(pass_point_cmd.x) || isnan(pass_point_cmd.y)){
            ROS_ERROR("Wrong pass dist");
            as_approach_door.setAborted(result_);
            return;
        }

        pcl::PointXYZ pass_point_kin = this->on_left_side ? apf->passages.at(0).kin_left :
                                                            apf->passages.at(0).kin_rght;
        Line_param *pass_line = apf->get_best_line(pass_point_kin, loc_srv->lm);
        davinci->draw_vec(pass_line->fdir_vec.kin.x, pass_line->fdir_vec.kin.y, 667, RED);

        if (pass_line == NULL) {
            ROS_ERROR("Pass_line wasn't found");
        }

        pcl::PointXYZ vec;
        double vec_len = move_epsilon;
        int pass_pos_num = 0;
        int start_pos_num = 0;

        double pass_point_ang = 0;

        ROS_INFO("AD Stage #1");
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
            double vel = this->on_left_side ? -0.4 : 0.4;
			pass_line = apf->get_best_line(pass_point_kin, loc_srv->lm);
			//pass_pos_num = map_srv->track(pcl::PointXYZ(pass_point_cmd.x, pass_point_cmd.y, 0)) - 1;
			if (pass_line != NULL && !isnan(pass_point_cmd.x) && !isnan(pass_point_cmd.y)) {
			    msn_srv->ref_ang  = target_angl;
			    msn_srv->ref_dist = target_dist;
			    double pass_pos_ang_diff = fabs(pass_point_ang) - angle_to_pass;
			    ROS_INFO("%f | %f | %f", pass_pos_ang_diff, pass_point_ang, fabs(pass_point_ang));
			    if (fabs(pass_pos_ang_diff) < rot_epsilon) {
			        msn_srv->unlock();
			        break;
			    }
			    else {
			        if (pass_pos_ang_diff > 0) {
			            ROS_INFO("Forw");
			            msn_srv->move_parallel(-vel);
			        }
			        else {
			            ROS_INFO("Backw");
			            msn_srv->move_parallel(vel);
			        }
			    }
			    /*
			    double offset_x = pass_line->ldir_vec.cmd.x * target_dist / 1.8;
			    double offset_y = pass_line->ldir_vec.cmd.y * target_dist / 1.8;
			    if (this->on_left_side) {
			        offset_x *= -1;
			        offset_y *= -1;
			    }
			    map_srv->track(pcl::PointXYZ((pass_point_cmd.x - pass_line->fdir_vec.cmd.x * target_dist + offset_x),
			                                 (pass_point_cmd.y - pass_line->fdir_vec.cmd.y * target_dist + offset_y), 0));


			    msn_srv->ref_ang  = target_angl;
			    msn_srv->ref_dist = target_dist;

			    ROS_INFO("%f | %f ", loc_srv->get_ref_wall()->distance,
			                         loc_srv->get_ref_wall()->angle);

			    if (fabs(pass_point_ang) < 10) {
			        ROS_INFO("DEBUG 2");
			        msn_srv->move_parallel(-vel);
			    }
			    if (fabs(loc_srv->get_ref_wall()->distance - target_dist) > move_epsilon ||
			        fabs(loc_srv->get_ref_wall()->angle - target_angl) > rot_epsilon) {
			        msn_srv->move_parallel(0);
			        ROS_INFO("DEBUG 3");
			    }
			    else {
			        double pass_point_dist = sqrt (pass_point_cmd.x * pass_point_cmd.x +
			                                       pass_point_cmd.y * pass_point_cmd.y) *
			                                 fabs(sin (pass_point_ang * M_PI / 180)) - target_dist / 1.8;
			        ROS_INFO("DEBUG 4");
			        ROS_INFO("%f | %f | %f | %f | %f", pass_point_dist, pass_point_dist + target_dist / 1.8, pass_point_ang,
			                fabs(sin (pass_point_ang * M_PI / 180)),
			                sqrt (pass_point_cmd.x * pass_point_cmd.x + pass_point_cmd.y * pass_point_cmd.y));
			        if (fabs(pass_point_dist) <= move_epsilon) {
			            ROS_INFO("DEBUG 5");
			            msn_srv->unlock();
			            break;
			        }
			        else if (pass_point_dist > 0) {
			            msn_srv->move_parallel(-vel);
			            ROS_INFO("DEBUG 6");
			        }
			        else {
			            msn_srv->move_parallel(vel);
			            ROS_INFO("DEBUG 7");
			        }
			    }

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
			   if (fabs(loc_srv->get_ref_wall()->angle - msn_srv->ref_ang) < wall_ang_eps ) {
			   msn_srv->ref_ang = 0;
			   if (loc_srv->get_ref_wall()->distance <= target_dist) {
			   msn_srv->ref_dist = target_dist;
			   msn_srv->move_parallel(-0.6);
			   }
			   else {
			   msn_srv->ref_dist = target_dist;
			   msn_srv->move_parallel(0);
			   }
			   }
			   else {
			   msn_srv->buf_cmd.angular.z += -0.5;
			   }
			   msn_srv->unlock();
			   }
			   msn_srv->lock();
			   */
			    /*
			    double offset_x = pass_line->ldir_vec.cmd.x * target_dist / 2;
			    double offset_y = pass_line->ldir_vec.cmd.x * target_dist / 2;

			    if (this->on_left_side) {
			        offset_x *= -1;
			        offset_y *= -1;
			    }

			    davinci->draw_vec(pass_line->ldir_vec.kin.x, pass_line->ldir_vec.kin.y, 5555, VIOLET);
			    davinci->draw_vec(pass_line->fdir_vec.kin.x, pass_line->fdir_vec.kin.y, 5556, CYAN);
				davinci->draw_line(pass_line, 5557, RED);
			    vec = pcl::PointXYZ((pass_point_cmd.x - pass_line->fdir_vec.cmd.x * target_dist + offset_x) / 2,
									(pass_point_cmd.y - pass_line->fdir_vec.cmd.y * target_dist + offset_y) / 2, 0);
				map_srv->track(pcl::PointXYZ(pass_point_cmd.x - pass_line->fdir_vec.cmd.x,
				                             pass_point_cmd.y - pass_line->fdir_vec.cmd.y, 0));
				map_srv->track(pcl::PointXYZ(pass_point_cmd.x + offset_x / target_dist,
				                             pass_point_cmd.y + offset_y / target_dist, 0));
				ROS_INFO("DEBUG 2");
				//start_pos_num = map_srv->track(vec) - 1;
				ROS_INFO("DEBUG 3");
				//msn_srv->unlock();
				if (this->move (vec, 0.4, 0, 0) == false) {
					ROS_ERROR("Move function caused error");
					as_approach_door.setAborted(result_);
					return;
				}
				//msn_srv->lock();
				ROS_INFO("DEBUG 4");
				vec_len = sqrt(vec.x * vec.x + vec.y*vec.y);
				if (!apf->passages.empty()) {
                    pass_point_cmd = this->on_left_side ? apf->passages.at(0).cmd_left :
                                                          apf->passages.at(0).cmd_rght;
                    ROS_INFO("DEBUG 4A");
                    pass_point_kin = this->on_left_side ? apf->passages.at(0).kin_left :
                                                          apf->passages.at(0).kin_rght;
				}
				else
				    break;
	            ROS_INFO("DEBUG 4B");
	            //int tmp = 0;
	            //scanf("%d", &tmp);
	             */
			}
			else {
			    double vel = 0.4;
                if (this->on_left_side)
                    vel *= -1;
			    msn_srv->move_parallel(vel);
				ROS_ERROR("Pass_line wasn't found");
			}
			msn_srv->unlock();
			r.sleep();
		}
        msn_srv->lock();
        ROS_INFO("DEBUG 5");
        pass_point_kin_op = this->on_left_side ? apf->passages.at(0).kin_rght :
                                                 apf->passages.at(0).kin_left;
        pass_line = apf->get_best_line(pass_point_kin_op, loc_srv->lm);
        pcl::PointXYZ pass_point_cmd_op = this->on_left_side ? apf->passages.at(0).cmd_rght :
                                                               apf->passages.at(0).cmd_left;
        if (pass_line != NULL && !isnan(pass_point_cmd_op.x) && !isnan(pass_point_cmd_op.y)) {
            loc_srv->track_wall(pass_line);
            msn_srv->unlock();
            result_.full_pass = true;
            as_approach_door.setSucceeded(result_);
            return;
        }
        ROS_INFO("DEBUG 6");
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

        // Check for appropriate input parameters
        pcl::PointXYZ pass_point_cmd (0, 0, 0);
        pcl::PointXYZ pass_point_kin (0, 0, 0);
        Line_param *pass_line = NULL;
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
        }

		if (pass_line == NULL || isnan(pass_point_cmd.x) || isnan(pass_point_cmd.y)){
			ROS_ERROR("Wrong pass dist");
			as_pass_door.setAborted(result_);
			return;
		}

		// Start action
        ROS_INFO("PD Stage #1");
        pcl::PointXYZ vec (0, 0, 0);

        vec.x = ( pass_point_cmd.x + pass_point_cmd.y) / sqrt(1.7);
        vec.y = (-pass_point_cmd.x + pass_point_cmd.y) / sqrt(1.7); // WTF? why '-'

        map_srv->track(vec) - 1;
        int pass_pos_num  = map_srv->track(pcl::PointXYZ (pass_point_cmd.x, pass_point_cmd.y, 0)) - 1;
        int start_pos_num = map_srv->track(pcl::PointXYZ (0, 0, 0)) - 1;

        if (this->move (vec, 0.4, 55, 0.5) == false) {
            ROS_ERROR("Move function caused error");
            as_pass_door.setAborted(result_);
            return;
        }

        ROS_INFO("PD Stage #2");
        //vec = pcl::PointXYZ (map_srv->tracked_points.at(pass_pos_num).x - map_srv->tracked_points.at(start_pos_num).x,
        //                     map_srv->tracked_points.at(pass_pos_num).y - map_srv->tracked_points.at(start_pos_num).y, 0);
        // 70 and 40 angle

        ROS_INFO("Vec2: %f\t %f", vec.x, vec.y);

        map_srv->track(vec);


        pcl::PointXYZ pass_vec (map_srv->tracked_points.at(pass_pos_num).x - map_srv->tracked_points.at(start_pos_num).x,
                                map_srv->tracked_points.at(pass_pos_num).y - map_srv->tracked_points.at(start_pos_num).y, 0);
/*
        if (apf->passages.size() > 0) {
            pass_x = apf->passages.at(0).cmd_left.x;
            pass_y = apf->passages.at(0).cmd_left.y;
            if (!isnan(pass_x) || !isnan(pass_y)){
                pass_vec = pcl::PointXYZ (pass_x - map_srv->tracked_points.at(start_pos_num).x,
                                          pass_y - map_srv->tracked_points.at(start_pos_num).y, 0);
            }
        }
*/
        vec.x = pass_vec.x;
        vec.y = pass_vec.y;

        map_srv->track(vec);


        if (this->move (vec, 0.5, 25, 0.5) == false) {
            ROS_ERROR("Move function caused error");
            as_pass_door.setAborted(result_);
            return;
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
    	//msn_srv->ref_ang = target_angl;

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
            if (loc_srv->get_ref_wall()->distance - target_dist <= move_epsilon)
                break;
            msn_srv->unlock();
            r.sleep();
        }

        msn_srv->unlock();
        result_.success = true;
        as_approach_wall.setSucceeded(result_);
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

    davinci->draw_vec(loc_srv->get_ref_wall()->ldir_vec.kin.x, loc_srv->get_ref_wall()->ldir_vec.kin.y, 5555, VIOLET);
    davinci->draw_vec(loc_srv->get_ref_wall()->fdir_vec.kin.x, loc_srv->get_ref_wall()->fdir_vec.kin.y, 5556, CYAN);

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
