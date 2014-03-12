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

	new_passage.kin_middle.x = (line.kin_inliers.at(id1).x + line.kin_inliers.at(id2).x)/2;
	new_passage.kin_middle.y = (line.kin_inliers.at(id1).y + line.kin_inliers.at(id2).y)/2;

	if (new_passage.width > 1.7)
		this->passage.push_back(new_passage);
};


void Passage_finder::check_boundary (Line_param &line)
{
	double left_ref_angle = atan(line.kin_inliers.at(0).x/line.kin_inliers.at(0).y)*180/PI;
	double rght_ref_angle = atan(line.kin_inliers.at(line.kin_inliers.size() - 1).x/
								 line.kin_inliers.at(line.kin_inliers.size() - 1).y)*180/PI;
	Passage new_passage;
	new_passage.width += 1.8;

	if(left_ref_angle > -29)
	{
		new_passage.kin_middle.x = line.kin_inliers.at(0).x - 0.9*line.ldir_vec.kin.x;
		new_passage.kin_middle.y = line.kin_inliers.at(0).y - 0.9*line.ldir_vec.kin.y;
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
	if (wall == NULL) return;
	if(this->ref_wall == NULL)
	{
		this->yaw = wall->angle;
	}

	this->ref_wall = wall;
	this->stm.angle = wall->angle;
	this->stm.distance = wall->distance;
	this->lost_ref_wall = false;
};


void LocationServer::spin_once(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
	this->lm.renew(cloud);
    this->ref_wall = this->lm.get_best_fit(this->stm.angle, this->stm.distance);
    if(!this->ref_wall->found) this->lost_ref_wall = true;
    double d_angle = stm.angle - this->ref_wall->angle;
    stm.angle = this->ref_wall->angle;
    stm.distance = this->ref_wall->distance;

    this->yaw -= d_angle;
};


bool LocationServer::obstacle_detected ()
{
	Line_param *lp_corner = lm.get_closest(stm.angle + 90);
	if(lp_corner->found) return true;
	else return true;
};



// *****************************************
// 				Motion server
// *****************************************
bool MotionServer::rotate(double angle)
{
	if(this->ref_wall == NULL) return false;
	if(angle + this->ref_wall->angle > 70.0) return false;
	this->ref_ang += angle;
	return true;
};


void MotionServer::spin_once()
{
	this->base_cmd.angular.x = this->base_cmd.angular.y = this->base_cmd.angular.z = 0;
	this->base_cmd.linear.x  = this->base_cmd.linear.y  = this->base_cmd.linear.z  = 0;
	this->base_cmd.angular.z = - this->pid_ang.get_output(this->ref_ang, this->ref_wall->angle);

	double vel_k = - this->pid_vel.get_output(ref_dist, this->ref_wall->distance);
	base_cmd.linear.x  += vel_k * this->ref_wall->fdir_vec.cmd.x;
	base_cmd.linear.y  += vel_k * this->ref_wall->fdir_vec.cmd.y;
};


bool MotionServer::move_parallel(double vel)
{
	base_cmd.linear.x  -= vel * this->ref_wall->ldir_vec.cmd.x;
	base_cmd.linear.y  -= vel * this->ref_wall->ldir_vec.cmd.y;

	return true;
};


bool MotionServer::move_perpendicular(double shift)
{
	this->ref_dist += shift;
	return true;
};




