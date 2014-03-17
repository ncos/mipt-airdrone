#include "passage_finder.h"


// *****************************************
// 				Passage finder
// *****************************************
Passage_finder::Passage_finder (Line_param &line)
{
	const double critical_angle = 11.0;
	double ref_angle = atan(line.kin_inliers.at(0).x/line.kin_inliers.at(0).y)*180/PI;
	for (int i = 1; i < line.kin_inliers.size(); i++)
	{
		double current_angle = atan(line.kin_inliers.at(i).x/line.kin_inliers.at(i).y)*180/PI;
		if (fabs(current_angle - ref_angle) > critical_angle) this->add_passage (i-1, i, line);
		ref_angle = current_angle;
	}
	this->check_boundary(line);
};


void Passage_finder::add_passage (int id1, int id2, Line_param &line)
{
	Passage new_passage;

	new_passage.width += pow(line.kin_inliers.at(id1).x - line.kin_inliers.at(id2).x, 2.0);
	new_passage.width += pow(line.kin_inliers.at(id1).y - line.kin_inliers.at(id2).y, 2.0);
	new_passage.width =  sqrt(new_passage.width);

	new_passage.kin_left.x = line.kin_inliers.at(id1).x;
	new_passage.kin_left.y = line.kin_inliers.at(id1).y;
	new_passage.kin_rght.x = line.kin_inliers.at(id2).x;
	new_passage.kin_rght.y = line.kin_inliers.at(id2).y;

	new_passage.kin_middle.x = (line.kin_inliers.at(id1).x + line.kin_inliers.at(id2).x)/2;
	new_passage.kin_middle.y = (line.kin_inliers.at(id1).y + line.kin_inliers.at(id2).y)/2;

	new_passage.left_ang = atan(new_passage.kin_left.x/new_passage.kin_left.y)*180/PI;
	new_passage.rght_ang = atan(new_passage.kin_rght.x/new_passage.kin_rght.y)*180/PI;
	new_passage.mid_ang  = atan(new_passage.kin_middle.x/new_passage.kin_middle.y)*180/PI;

	if (new_passage.width > 1.7)
		this->passage.push_back(new_passage);
};


void Passage_finder::check_boundary (Line_param &line)
{
	int max_id = line.kin_inliers.size() - 1;

	double left_ref_angle = atan(line.kin_inliers.at(0).x/line.kin_inliers.at(0).y)*180/PI;
	double rght_ref_angle = atan(line.kin_inliers.at(max_id).x/
								 line.kin_inliers.at(max_id).y)*180/PI;
	double lra_sq_distance = line.kin_inliers.at(0).x*line.kin_inliers.at(0).x +
			                 line.kin_inliers.at(0).y*line.kin_inliers.at(0).y;
	double rra_sq_distance = line.kin_inliers.at(max_id).x*line.kin_inliers.at(max_id).x +
							 line.kin_inliers.at(max_id).y*line.kin_inliers.at(max_id).y;

	//ROS_INFO("lra %f, \tlrd %f", left_ref_angle, lra_sq_distance);
	Passage new_passage;
	new_passage.width += 1.8;

	if(lra_sq_distance < 8.0 && left_ref_angle > -25)
	{
		new_passage.kin_left.x = line.kin_inliers.at(0).x - (new_passage.width)*line.ldir_vec.kin.x;
		new_passage.kin_left.y = line.kin_inliers.at(0).y - (new_passage.width)*line.ldir_vec.kin.y;
		new_passage.kin_rght.x = line.kin_inliers.at(0).x;
		new_passage.kin_rght.y = line.kin_inliers.at(0).y;

		new_passage.kin_middle.x = line.kin_inliers.at(0).x - (new_passage.width / 2.0)*line.ldir_vec.kin.x;
		new_passage.kin_middle.y = line.kin_inliers.at(0).y - (new_passage.width / 2.0)*line.ldir_vec.kin.y;

		new_passage.left_ang = atan(new_passage.kin_left.x/new_passage.kin_left.y)*180/PI;
		new_passage.rght_ang = atan(new_passage.kin_rght.x/new_passage.kin_rght.y)*180/PI;
		new_passage.mid_ang  = atan(new_passage.kin_middle.x/new_passage.kin_middle.y)*180/PI;

		this->passage.push_back(new_passage);
	}
/*
	if(rght_ref_angle < 29)
	{
		new_passage.kin_middle.x = line.kin_inliers.at(line.kin_inliers.size() - 1).x + 0.9*line.ldir_vec.kin.x;
		new_passage.kin_middle.y = line.kin_inliers.at(line.kin_inliers.size() - 1).y + 0.9*line.ldir_vec.kin.y;
		this->passage.push_back(new_passage);
	}
*/
};


// *****************************************
// 				Location server
// *****************************************
void LocationServer::track_wall(Line_param *wall)
{
	if(wall == NULL) {
		ROS_ERROR("LocationServer::track_wall argument is NULL");
		return;
	}

	if(this->ref_wall == NULL) {
		this->yaw = wall->angle;
	}

	this->ref_wall = wall;
	this->stm.angle = wall->angle;
	this->stm.distance = wall->distance;
	this->lost_ref_wall = false;
};


void LocationServer::spin_once(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
    if(this->ref_wall == NULL) {
    	ROS_WARN("No ref_wall. Using random!");
    };

	this->lm.renew(cloud);
    this->ref_wall = this->lm.get_best_fit(this->stm.angle, this->stm.distance);


    if(!this->ref_wall->found) { this->lost_ref_wall = true; ROS_WARN("Lost ref_wall"); }
    else this->lost_ref_wall = false;
    double d_angle = this->stm.angle - this->ref_wall->angle;
    this->stm.angle = this->ref_wall->angle;
    this->stm.distance = this->ref_wall->distance;

    this->yaw -= d_angle;

	this->corner_wall_left = lm.get_closest(this->stm.angle + 90);
	this->corner_wall_rght = lm.get_closest(this->stm.angle - 90);
};





bool LocationServer::obstacle_detected_left ()
{
	if(this->corner_wall_left == NULL) return false;
	return true;
};



bool LocationServer::obstacle_detected_rght ()
{
	if(this->corner_wall_rght == NULL) return false;
	return true;
};


// *****************************************
// 				Motion server
// *****************************************
void MotionServer::set_ref_wall (Line_param *wall)
{
	this->base_cmd.angular.x = this->base_cmd.angular.y = this->base_cmd.angular.z = 0;
	this->base_cmd.linear.x  = this->base_cmd.linear.y  = this->base_cmd.linear.z  = 0;

	if(wall == NULL) {
		ROS_ERROR("MotionServer::set_ref_wall argument is NULL");
		return;
	}

	this->ref_wall = wall;

	if(!wall->found)
		this->set_angles_current();
};


void MotionServer::set_angles_current ()
{
	if(this->ref_wall == NULL) {
		ROS_ERROR("MotionServer::set_angles_current ref_wall == NULL");
		return;
	}

	this->ref_ang = this->ref_wall->angle;
	this->ref_dist = this->ref_wall->distance;
};


bool MotionServer::rotate(double angle)
{
	if(this->ref_wall == NULL) {
		ROS_ERROR("MotionServer::rotate ref_wall == NULL");
		return false;
	}

	this->ref_ang += angle;
	return true;
};


bool MotionServer::move_parallel(double vel)
{
	if(this->ref_wall == NULL) {
		ROS_ERROR("MotionServer::move_parallel ref_wall == NULL");
		return false;
	}

	base_cmd.linear.x  -= vel * this->ref_wall->ldir_vec.cmd.x;
	base_cmd.linear.y  -= vel * this->ref_wall->ldir_vec.cmd.y;

	return true;
};


bool MotionServer::move_perpendicular(double shift)
{
	this->ref_dist += shift;
	return true;
};


void MotionServer::spin_once()
{
	if(this->ref_wall == NULL) {
		ROS_ERROR("MotionServer::spin_once ref_wall == NULL");
		return;
	}

	if(this->ref_dist < 0.6) {
    	ROS_WARN("Invalid ref_dist (%f)", this->ref_dist);
		this->ref_dist = 0.6;
	}


	this->base_cmd.angular.z = - this->pid_ang.get_output(this->ref_ang, this->ref_wall->angle);

	double vel_k = - this->pid_vel.get_output(ref_dist, this->ref_wall->distance);
	base_cmd.linear.x  += vel_k * this->ref_wall->fdir_vec.cmd.x;
	base_cmd.linear.y  += vel_k * this->ref_wall->fdir_vec.cmd.y;
};


