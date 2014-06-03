#include "passage_finder.h"


// *****************************************
// 				Passage finder
// *****************************************
Passage_finder::Passage_finder (Line_param &line)
{
	const double critical_angle = 11.0;
	double ref_angle = atan(line.kin_inliers.at(0).x/line.kin_inliers.at(0).y)*180/PI;
	for (int i = 1; i < line.kin_inliers.size(); ++i)
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

	new_passage.width  = pow(line.kin_inliers.at(id1).x - line.kin_inliers.at(id2).x, 2.0);
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
	new_passage.width = 1.8;

	if (lra_sq_distance < 8.0 && left_ref_angle > -25)
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
	if (rght_ref_angle < 29)
	{
		new_passage.kin_middle.x = line.kin_inliers.at(line.kin_inliers.size() - 1).x + 0.9*line.ldir_vec.kin.x;
		new_passage.kin_middle.y = line.kin_inliers.at(line.kin_inliers.size() - 1).y + 0.9*line.ldir_vec.kin.y;
		this->passage.push_back(new_passage);
	}
*/
};






// *****************************************
//              Advanced Passage Finder
// *****************************************

void Advanced_Passage_finder::renew(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
    // cloud->points.at(0).x is "x" coordinate
    // cloud->points.at(0).z is "y" coordinate
    this->passages.clear();
    const double min_width = 1.4;
    const int max_id = cloud->points.size() - 1;
    if (cloud->points.size() < 2) return;
    for (int i = 1; i < cloud->points.size(); ++i) {
        if (this->sqrange(cloud->points.at(i-1), cloud->points.at(i)) > min_width) {
            this->add_passage(cloud->points.at(i-1).x, cloud->points.at(i-1).z, cloud->points.at(i).x, cloud->points.at(i).z);
        }
    }

    // Check the leftmost point
    double left_ang = atan(cloud->points.at(0).x/cloud->points.at(0).z)*180/PI;
    double rght_ang = atan(cloud->points.at(max_id).x/cloud->points.at(max_id).z)*180/PI;
    double left_sqdist = cloud->points.at(0).x*cloud->points.at(0).x + cloud->points.at(0).z*cloud->points.at(0).z;
    double rght_sqdist = cloud->points.at(max_id).x*cloud->points.at(max_id).x + cloud->points.at(max_id).z*cloud->points.at(max_id).z;

    //ROS_INFO("LA: %3f - %3f    %3f - %3f", left_ang, left_sqdist, rght_ang, rght_sqdist);

    if ((cloud->points.at(0).z < 2.8) && (left_ang > -25)) {
        //ROS_INFO("LA: %3f - %3f", left_ang, cloud->points.at(0).z);
        add_passage(cloud->points.at(0).x, cloud->points.at(0).z, NAN, NAN);
    }

    if ((cloud->points.at(max_id).z < 2.8) && (rght_ang <  25)) {
        //ROS_ERROR("RA: %3f - %3f", rght_ang, cloud->points.at(max_id).z);
        add_passage(NAN, NAN, cloud->points.at(max_id).x, cloud->points.at(max_id).z);
    }
};


void Advanced_Passage_finder::add_passage(double point1x, double point1y, double point2x, double point2y)
{
    Passage new_passage;

    if (!isnan(point1x) && !isnan(point1y) && !isnan(point2x) && !isnan(point2y))
        new_passage.is_nan = false;

    new_passage.kin_left.x = point1x;
    new_passage.kin_left.y = point1y;
    new_passage.kin_rght.x = point2x;
    new_passage.kin_rght.y = point2y;

    new_passage.width += pow(point1x - point2x, 2.0);
    new_passage.width += pow(point1y - point2y, 2.0);
    new_passage.width =  sqrt(new_passage.width);

    new_passage.kin_middle.x = (point1x + point2x)/2;
    new_passage.kin_middle.y = (point1y + point2y)/2;

    new_passage.left_ang = atan(new_passage.kin_left.x/new_passage.kin_left.y)*180/PI;
    new_passage.rght_ang = atan(new_passage.kin_rght.x/new_passage.kin_rght.y)*180/PI;
    new_passage.mid_ang  = atan(new_passage.kin_middle.x/new_passage.kin_middle.y)*180/PI;

    this->passages.push_back(new_passage);
};



bool Advanced_Passage_finder::passage_on_line(Line_param &line, Passage &passage)
{
    const double eps = 0.1; // How close should be the passage border to the line

    if (line.distance_to_point(passage.kin_left.x, passage.kin_left.y) < eps) return true;
    if (line.distance_to_point(passage.kin_rght.x, passage.kin_rght.y) < eps) return true;

    return false;
};


Line_param* Advanced_Passage_finder::get_best_line(Passage &passage, Line_map &linemap)
{
    const double eps = 0.1; // How close should be the passage border to the line
    const int min_quality = 10;

    int best_quality = 0, best_i = 0;
    for (int i = 0; i < linemap.lines.size(); ++i) {
        if ((linemap.lines.at(i).distance_to_point(passage.kin_left.x, passage.kin_left.y) < eps)
           && (linemap.lines.at(i).quality > best_quality))
        {
            best_quality = linemap.lines.at(i).quality;
            best_i = i;
        }
    }

    if (best_quality < min_quality) return NULL;
    return &linemap.lines.at(best_i);
};


double Advanced_Passage_finder::sqrange(pcl::PointXYZ p1, pcl::PointXYZ p2)
{
    return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
};



// *****************************************
// 				Location server
// *****************************************
void LocationServer::track_wall(Line_param *wall)
{
	if (wall == NULL) {
		ROS_ERROR("LocationServer::track_wall argument is NULL");
		return;
	}

	if (this->ref_wall == NULL) {
		this->yaw = wall->angle;
	}

	this->ref_wall = wall;
	this->stm.angle = wall->angle;
	this->stm.distance = wall->distance;
	this->lost_ref_wall = false;

	this->corner_wall_left = lm.get_closest(this->stm.angle + 90);
	this->corner_wall_rght = lm.get_closest(this->stm.angle - 90);
};


void LocationServer::spin_once(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
    if (cloud->points.size() < 10) {
        ROS_WARN("The cloud is empty. Location server is unable to provide pose estimation. Skipping...");
        if (this->ref_wall != NULL) {
            this->ref_wall->found = false;
        }
        return;
    };

    if (this->ref_wall == NULL) {
    	ROS_WARN("No ref_wall. Using random!");
    };

	this->lm.renew(cloud);
    this->ref_wall = this->lm.get_best_fit(this->stm.angle, this->stm.distance);


    if (!this->ref_wall->found) { this->lost_ref_wall = true; ROS_WARN("Lost ref_wall"); }
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
	if (this->corner_wall_left == NULL) return false;
	return true;
};



bool LocationServer::obstacle_detected_rght ()
{
	if (this->corner_wall_rght == NULL) return false;
	return true;
};


// *****************************************
// 				Motion server
// *****************************************
void MotionServer::clear_cmd ()
{
	this->base_cmd.angular.x = this->base_cmd.angular.y = this->base_cmd.angular.z = 0;
	this->base_cmd.linear.x  = this->base_cmd.linear.y  = this->base_cmd.linear.z  = 0;
	this->buf_cmd.angular.x  = this->buf_cmd.angular.y  = this->buf_cmd.angular.z  = 0;
	this->buf_cmd.linear.x   = this->buf_cmd.linear.y   = this->buf_cmd.linear.z   = 0;
};


void MotionServer::set_ref_wall (Line_param *wall)
{
	if (wall == NULL) {
		ROS_ERROR("MotionServer::set_ref_wall argument is NULL");
		return;
	}

	this->ref_wall = wall;

	if (!wall->found)
		this->set_angles_current();
};

void MotionServer::track () {
    this->tracking_on = true;
}

void MotionServer::untrack()
{
    tracking_on = false;
    this->ref_wall = NULL;
};

void MotionServer::set_angles_current ()
{
	if (this->tracking_on && this->ref_wall == NULL) {
		ROS_ERROR("MotionServer::set_angles_current ref_wall == NULL");
		return;
	}

	this->ref_ang = this->ref_wall->angle;
	this->ref_dist = this->ref_wall->distance;
};


bool MotionServer::rotate(double angle)
{
	if (this->tracking_on && this->ref_wall == NULL) {
		ROS_ERROR("MotionServer::rotate ref_wall == NULL");
		return false;
	}

	this->ref_ang += angle;
	return true;
};


bool MotionServer::move_parallel(double vel)
{
	if (this->tracking_on && this->ref_wall == NULL) {
		ROS_ERROR("MotionServer::move_parallel ref_wall == NULL");
		return false;
	}

	buf_cmd.linear.x  = - vel * this->ref_wall->ldir_vec.cmd.x;
	buf_cmd.linear.y  = - vel * this->ref_wall->ldir_vec.cmd.y;

	return true;
};


bool MotionServer::move_perpendicular(double shift)
{
    if (this->tracking_on && this->ref_wall == NULL) {
        ROS_ERROR("MotionServer::move_perpendicular ref_wall == NULL");
        return false;
    }

	this->ref_dist += shift;
	return true;
};


void MotionServer::spin_once()
{
    if (this->tracking_on && this->ref_wall == NULL) {
		ROS_ERROR("MotionServer::spin_once ref_wall == NULL");
		return;
	}

	if (this->tracking_on && this->ref_dist < 0.6) {
    	ROS_WARN("Invalid ref_dist (%f)", this->ref_dist);
		this->ref_dist = 0.6;
	}

	if (this->tracking_on) {
	    this->base_cmd.angular.z = - this->pid_ang.get_output(this->ref_ang, this->ref_wall->angle);

        double vel_k = - this->pid_vel.get_output(this->ref_dist, this->ref_wall->distance);
        this->base_cmd.linear.x  += vel_k * this->ref_wall->fdir_vec.cmd.x + this->buf_cmd.linear.x;
        this->base_cmd.linear.y  += vel_k * this->ref_wall->fdir_vec.cmd.y + this->buf_cmd.linear.y;
	}
	else {
	    this->base_cmd.linear.x  += this->buf_cmd.linear.x;
	    this->base_cmd.linear.y  += this->buf_cmd.linear.y;
	}
};




// *****************************************
//              Mapping server
// *****************************************
MappingServer::MappingServer(ros::NodeHandle _nh, std::string inp_topic)
{
    init_flag =   false; // Look into class defenition
    offset_cmd.x =    0;
    offset_cmd.y =    0;
    distance.x =      0;
    distance.y =      0;
    position_prev.x = 0;
    position_prev.y = 0;
    mutex = boost::shared_ptr<boost::mutex>   (new boost::mutex);
    std::string topic = _nh.resolveName(inp_topic);
    sub = _nh.subscribe<geometry_msgs::PoseStamped> (topic, 1,  &MappingServer::callback, this);
}

void MappingServer::callback (const geometry_msgs::PoseStamped pos_msg)
{
    this->lock();

    if (init_flag == false) {
        position_prev.x = pos_msg.pose.position.x;
        position_prev.y = pos_msg.pose.position.y;
        init_flag = true;
        this->unlock();
        return;
    }

    pcl::PointXY position;
    position.x = pos_msg.pose.position.x;
    position.y = pos_msg.pose.position.y;
    double alpha = PI/2 - 2 * asin (fabs(pos_msg.pose.orientation.z)) - kinect_angl * PI / 180.0; // Fucking magic: sin(pi/4-a/2) and offset for kinect turn
    pcl::PointXY offset; // Offset in gazebo
    offset.x = (position.x - position_prev.x);
    offset.y = (position.y - position_prev.y);
    offset_cmd.x = offset.x * cos (alpha) - offset.y * sin (alpha);
    offset_cmd.y = offset.x * sin (alpha) + offset.y * cos (alpha);
    //ROS_INFO("Offset: %f\t %f\n alpha: %f", offset_cmd.x, offset_cmd.y, alpha);
    distance.x += offset_cmd.x;
    distance.y += offset_cmd.y;
    position_prev.x = position.x;
    position_prev.y = position.y;
    //ROS_INFO("Offset: x: %f\t y: %f\n Distance: x: %f\t y: %f\n", offset_cmd.x, offset_cmd.x, distance.x, distance.y);
    this->unlock();
};

pcl::PointXY MappingServer::get_positon()
{
    this->lock();
    pcl::PointXY ret = distance;
    this->unlock();
    return distance;
}
