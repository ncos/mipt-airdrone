#include "passage_finder.h"

// *****************************************
//              Advanced Passage Finder
// *****************************************

void Advanced_Passage_finder::renew(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
    // cloud->points.at(0).x is "x" coordinate
    // cloud->points.at(0).z is "y" coordinate
    this->passages.clear();


    const int max_id = cloud->points.size() - 1;
    const int min_id = 0;

    if (cloud->points.size() < 2) return;


    for (int i = 1; i < cloud->points.size(); ++i) {
        if (apf_better_q == 0) {
            if (this->sqrange(cloud->points.at(i - 1), cloud->points.at(i)) > apf_min_width * apf_min_width) {
                this->add_passage(cloud->points.at(i - 1).x, cloud->points.at(i - 1).z, cloud->points.at(i).x, cloud->points.at(i).z);
            }
        }

        if (apf_better_q == 1) {
            if (this->sqrange(cloud->points.at(i - 1), cloud->points.at(i)) > apf_min_width * apf_min_width) {
                bool edge_left = ((cloud->points.at(i - 1).z > apf_min_dist) && (cloud->points.at(i - 1).z < apf_max_dist));
                bool edge_rght = ((cloud->points.at(i).z     > apf_min_dist) && (cloud->points.at(i).z     < apf_max_dist));

                if (edge_left &&  edge_rght) {
                    this->add_passage(cloud->points.at(i - 1).x, cloud->points.at(i - 1).z, cloud->points.at(i).x, cloud->points.at(i).z);
                }

                if (edge_left && !edge_rght) {
                    pcl::PointXYZ rght = this->get_closest_rght(cloud, i - 1);
                    if (this->sqrange(cloud->points.at(i - 1), rght) > apf_min_width * apf_min_width) {
                        this->add_passage(cloud->points.at(i - 1).x, cloud->points.at(i - 1).z, rght.x, rght.z);
                    }
                }

                if (!edge_left && edge_rght) {
                    pcl::PointXYZ left = this->get_closest_left(cloud, i);
                    if (this->sqrange(left, cloud->points.at(i)) > apf_min_width * apf_min_width) {
                        this->add_passage(left.x, left.z, cloud->points.at(i).x, cloud->points.at(i).z);
                    }
                }
            }
        }



        if (apf_better_q != 0 && apf_better_q != 1) {
            ROS_WARN("Invalid 'apf_better_q' param (...= %f). Assuming it is zero", apf_better_q);
            apf_better_q = 0;
        }
    }

    // Check the leftmost and rightmost point (this is in case only one side of the passage is seen)
    double left_ang = atan(cloud->points.at(min_id).x / cloud->points.at(min_id).z) * 180 / M_PI;  // The angle of the leftmost point
    double rght_ang = atan(cloud->points.at(max_id).x / cloud->points.at(max_id).z) * 180 / M_PI;  // The angle of the rightmost point


    // RIGHT (!important!) passage point detection
    if ((cloud->points.at(min_id).z > apf_min_dist) && (cloud->points.at(min_id).z < apf_max_dist) && (left_ang > apf_min_angl)) {
        // If this is the leftmost point, the passage is possibly even more at the left, so this point is the (supposedly)
        // RIGHT point of the passage (if rotate left and look directly at the passage this point will be on the right)
        add_passage(NAN, NAN, cloud->points.at(min_id).x, cloud->points.at(min_id).z);
    }

    // LEFT (!important!) passage point detection
    if ((cloud->points.at(max_id).z > apf_min_dist) && (cloud->points.at(max_id).z < apf_max_dist) && (rght_ang < apf_max_angl)) {
        // This is similar to the RIGHT point detection
        add_passage(cloud->points.at(max_id).x, cloud->points.at(max_id).z, NAN, NAN);
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

    new_passage.width = sqrt(pow(point1x - point2x, 2.0) + pow(point1y - point2y, 2.0));

    new_passage.kin_middle.x = (point1x + point2x) / 2;
    new_passage.kin_middle.y = (point1y + point2y) / 2;

    new_passage.left_ang = atan(new_passage.kin_left.x/new_passage.kin_left.y) * 180 / M_PI;
    new_passage.rght_ang = atan(new_passage.kin_rght.x/new_passage.kin_rght.y) * 180 / M_PI;
    new_passage.mid_ang  = atan(new_passage.kin_middle.x/new_passage.kin_middle.y) * 180 / M_PI;

    // Convert kin to cmd coordinate system:
    new_passage.cmd_left.x   =   new_passage.kin_left.y;
    new_passage.cmd_left.y   = - new_passage.kin_left.x;

    new_passage.cmd_middle.x =   new_passage.kin_middle.y;
    new_passage.cmd_middle.y = - new_passage.kin_middle.x;

    new_passage.cmd_rght.x   =   new_passage.kin_rght.y;
    new_passage.cmd_rght.y   = - new_passage.kin_rght.x;

    this->passages.push_back(new_passage);
};



bool Advanced_Passage_finder::passage_on_line(Line_param &line, Passage &passage)
{
    const double eps = 0.1; // How close should be the passage border to the line

    if (line.distance_to_point(passage.kin_left.x, passage.kin_left.y) < eps) return true;
    if (line.distance_to_point(passage.kin_rght.x, passage.kin_rght.y) < eps) return true;

    return false;
};


Line_param* Advanced_Passage_finder::get_best_line(pcl::PointXYZ &point, Line_map &linemap)
{
    const double eps = 0.1; // How close should be the passage border to the line
    const int min_quality = 1;

    int best_quality = 0, best_i = 0;
    for (int i = 0; i < linemap.lines.size(); ++i) {
        if ((linemap.lines.at(i).distance_to_point(point.x, point.y) < eps)
           && (linemap.lines.at(i).quality > best_quality))
        {
            best_quality = linemap.lines.at(i).quality;
            best_i = i;
        }
    }

    if (best_quality < min_quality) return NULL;
    return &linemap.lines.at(best_i);
};


pcl::PointXYZ Advanced_Passage_finder::get_closest_left(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, int point_id)
{
    // Return the geometrically closest point on the left from the given point. A given point is specified by its id in the cloud
    if (point_id < 1 || cloud->points.size() < point_id + 1) { // There should be at least one point on the left
        ROS_ERROR("Advanced_Passage_finder::get_closest_left: invalid arguments");
        return pcl::PointXYZ(NAN, NAN, NAN);
    }

    double min_id   = 0;
    double min_dist = this->sqrange(cloud->points.at(point_id), cloud->points.at(min_id));
    for (int i = 1; i < point_id; ++i) {
        double dist = this->sqrange(cloud->points.at(point_id), cloud->points.at(i));
        if (dist < min_dist) {
            min_dist = dist;
            min_id = i;
        }
    }

    return cloud->points.at(min_id);
};


pcl::PointXYZ Advanced_Passage_finder::get_closest_rght(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, int point_id)
{
    // Return the geometrically closest point on the right from the given point. A given point is specified by its id in the cloud
    if (point_id < 0 || cloud->points.size() < point_id + 2) { // There should be at least one point on the right
        ROS_ERROR("Advanced_Passage_finder::get_closest_rght: invalid arguments");
        return pcl::PointXYZ(NAN, NAN, NAN);
    }

    double min_id   = cloud->points.size() - 1;
    double min_dist = this->sqrange(cloud->points.at(point_id), cloud->points.at(min_id));
    for (int i = point_id + 1; i < cloud->points.size(); ++i) {
        double dist = this->sqrange(cloud->points.at(point_id), cloud->points.at(i));
        if (dist < min_dist) {
            min_dist = dist;
            min_id = i;
        }
    }

    return cloud->points.at(min_id);
};


double Advanced_Passage_finder::sqrange(pcl::PointXYZ p1, pcl::PointXYZ p2)
{
    if (isnan(p1.x) || isnan(p1.y) || isnan(p1.z) || isnan(p2.x) || isnan(p2.y) || isnan(p2.z)) {
        ROS_ERROR("Advanced_Passage_finder::sqrange: getting distance from NAN!");
    }

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
MotionServer::~MotionServer ()
{
    this->lock(); // Location and motion servers are bound to the same mutex
    this->move_parallel(0); // Stop movement
    this->set_angles_current(); // And rotation
    this->unlock();
};


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
};


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
	    this->base_cmd.linear.z  += this->buf_cmd.linear.z;

	    this->base_cmd.angular.x += this->buf_cmd.angular.x;
	    this->base_cmd.angular.y += this->buf_cmd.angular.y;
	    this->base_cmd.angular.z += this->buf_cmd.angular.z;
	}
};




// *****************************************
//              Mapping server
// *****************************************
MappingServer::MappingServer(ros::NodeHandle _nh, std::string inp_topic)
{
    init_flag =   false; // Look into class defenition
    offset_cmd.x    = 0;
    offset_cmd.y    = 0;
    distance.x      = 0;
    distance.y      = 0;
    position_prev.x = 0;
    position_prev.y = 0;
    delta_phi       = 0;
    rotation_cnt    = 0;
    mutex = boost::shared_ptr<boost::mutex>   (new boost::mutex);
    std::string topic = _nh.resolveName(inp_topic);
    sub = _nh.subscribe<geometry_msgs::PoseStamped> (topic, 1,  &MappingServer::callback, this);
};


double MappingServer::get_angl_from_quaternion (const geometry_msgs::PoseStamped pos_msg)
{
    double vec_len = sqrt(pos_msg.pose.orientation.x * pos_msg.pose.orientation.x +
                          pos_msg.pose.orientation.y * pos_msg.pose.orientation.y +
                          pos_msg.pose.orientation.z * pos_msg.pose.orientation.z);
    double angle = 0;

    if (vec_len != 0 ) {
            angle = 2.0 * atan2(pos_msg.pose.orientation.z, pos_msg.pose.orientation.w) * 180 / M_PI;
    }
    else {
        angle = 0;
    }
    if (angle < 0)
        angle += 360;
    return angle;
};


void MappingServer::callback (const geometry_msgs::PoseStamped pos_msg)
{
    this->lock();

    if (this->init_flag == false) {
        this->position_prev.x = pos_msg.pose.position.x;
        this->position_prev.y = pos_msg.pose.position.y;
        this->position_prev.z = 0;
        this->prev_phi  = this->get_angl_from_quaternion (pos_msg);
        this->init_flag = true;
        this->unlock();
        return;
    }


    this->delta_phi = this->get_angl_from_quaternion (pos_msg) - this->prev_phi;
    if (this->delta_phi > 300) {
        this->delta_phi -= 360;
        this->rotation_cnt --;
    }

    if (this->delta_phi < -300) {
        this->delta_phi += 360;
        this->rotation_cnt ++;
    }
    this->prev_phi = this->get_angl_from_quaternion (pos_msg);


    pcl::PointXYZ position (pos_msg.pose.position.x, pos_msg.pose.position.y, 0);
    pcl::PointXYZ offset (position.x - this->position_prev.x, position.y - this->position_prev.y, 0);

    this->offset_cmd = this->rotate(offset, -(this->get_angl_from_quaternion (pos_msg) + angle_of_kinect));

    this->distance.x += this->offset_cmd.x;
    this->distance.y += this->offset_cmd.y;
    this->position_prev.x = position.x;
    this->position_prev.y = position.y;


    for (int i = 0; i < this->tracked_points.size(); ++i) {
        this->tracked_points.at(i)   = this->rotate(this->tracked_points.at(i), -this->delta_phi);
        this->tracked_points.at(i).x = this->tracked_points.at(i).x - this->offset_cmd.x;
        this->tracked_points.at(i).y = this->tracked_points.at(i).y - this->offset_cmd.y;
    }

    for (int i = 0; i < this->visited_points.size(); ++i) {
        this->visited_points.at(i)   = this->rotate(this->visited_points.at(i), -this->delta_phi);
        this->visited_points.at(i).x = this->visited_points.at(i).x - this->offset_cmd.x;
        this->visited_points.at(i).y = this->visited_points.at(i).y - this->offset_cmd.y;
    }


    for (int i = 0; i < this->visited_points.size(); ++i) {
        davinci->draw_point_cmd(this->visited_points.at(i).x, this->visited_points.at(i).y, 0.04, 494 + i, GOLD);
    }

    for (int i = 0; i < this->tracked_points.size(); ++i) {
        davinci->draw_point_cmd(this->tracked_points.at(i).x, this->tracked_points.at(i).y, 994 + i, CYAN);
    }

    this->add_visited();
    this->unlock();
};


int MappingServer::track (pcl::PointXYZ p)
{
    this->lock();
    this->tracked_points.push_back(p);
    this->unlock();
    return this->tracked_points.size();
};


void MappingServer::add_visited ()
{
    const unsigned int max_size_ = 30;
    const double dist_ = 0.05;

    if (this->visited_points.size() > 0) {
        pcl::PointXYZ diff = this->visited_points.back();
        if ((diff.x * diff.x + diff.y * diff.y) > dist_) {
            this->visited_points.push_back(pcl::PointXYZ(0, 0, 0));
        }
    }
    else {
        this->visited_points.push_back(pcl::PointXYZ(0, 0, 0));
    }

    if (this->visited_points.size() > max_size_) {
        this->visited_points.erase (this->visited_points.begin());
    }
};


pcl::PointXYZ MappingServer::get_global_positon()
{
    this->lock();
    pcl::PointXYZ ret = this->distance;
    this->unlock();
    return ret;
};


double MappingServer::get_global_angle()
{
    this->lock();
    double ret = this->prev_phi;
    this->unlock();
    return ret;
};


double MappingServer::diff(double a, double b)
{
    return a - b;
};


pcl::PointXYZ MappingServer::diff(pcl::PointXYZ a, pcl::PointXYZ b)
{
    pcl::PointXYZ c;
    c.x = a.x - b.x;
    c.y = a.y - b.y;
    return c;
};


pcl::PointXYZ MappingServer::rotate(const pcl::PointXYZ vec, double angle)
{
    pcl::PointXYZ rotated;
    rotated.x = vec.x * cos(angle * M_PI / 180.0) - vec.y * sin(angle * M_PI / 180.0);
    rotated.y = vec.x * sin(angle * M_PI / 180.0) + vec.y * cos(angle * M_PI / 180.0);
    rotated.z = vec.z;

    return rotated;
};


//              Passage type recognition

int Passage_type::recognize (boost::shared_ptr<Advanced_Passage_finder> apf, Line_map lm, bool on_left_side) {
    this->type = non_valid;
    this->pass_exist = false;
    this->closest_exist = false;
    this->opposite_exist = false;
    this->middle_exist = false;

    if (apf->passages.empty()) {
        return non_valid;
    }

    pcl::PointXYZ pass_point_cmd = on_left_side ? apf->passages.at(0).cmd_left :
                                                  apf->passages.at(0).cmd_rght;
    pcl::PointXYZ pass_point_cmd_op = on_left_side ? apf->passages.at(0).cmd_rght :
                                                     apf->passages.at(0).cmd_left;
    pcl::PointXYZ pass_point_kin = on_left_side ? apf->passages.at(0).kin_left :
                                                  apf->passages.at(0).kin_rght;
    pcl::PointXYZ pass_point_kin_op = on_left_side ? apf->passages.at(0).kin_rght :
                                                     apf->passages.at(0).kin_left;
    pcl::PointXYZ pass_point_middle = apf->passages.at(0).cmd_middle;
    double pass_point_ang = on_left_side ? apf->passages.at(0).left_ang :
                                           apf->passages.at(0).rght_ang;
    Line_param *pass_line    = apf->get_best_line(pass_point_kin,    lm);
    Line_param *pass_line_op = apf->get_best_line(pass_point_kin_op, lm);

    double scalar_mul = NAN;
    if (pass_line != NULL && pass_line_op != NULL)
        scalar_mul = pass_line_op->ldir_vec.cmd.x * pass_line->ldir_vec.cmd.x +
                     pass_line_op->ldir_vec.cmd.y * pass_line->ldir_vec.cmd.y;

    if(pass_line != NULL && !isnan(pass_point_cmd.x) && !isnan(pass_point_cmd.y)) {
        this->pass_exist = true;
        this->closest_exist = true;
    }

    if(pass_line_op != NULL && !isnan(pass_point_cmd_op.x) && !isnan(pass_point_cmd_op.y)) {
        this->pass_exist = true;
        this->opposite_exist = true;
    }

    if(!isnan(pass_point_middle.x) && !isnan(pass_point_middle.y)) {
        this->pass_exist = true;
        this->middle_exist = true;
    }

    if ( this->pass_exist   &&  this->closest_exist &&
        !this->middle_exist && !this->opposite_exist) {
        this->type = single_wall;
        return single_wall;
    }

    if (this->pass_exist && this->closest_exist && this->opposite_exist &&
        !isnan(scalar_mul) && fabs(scalar_mul) < 0.1) {
        this->type = ortogonal;
        return ortogonal;
    }

    if (this->pass_exist && this->middle_exist &&
        !isnan(scalar_mul) && fabs(scalar_mul - 1) < 0.1) {
        this->type = parrallel;
        return parrallel;
    }

    this->type = undefined;
    return undefined;
}

