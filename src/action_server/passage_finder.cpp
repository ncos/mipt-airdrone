#include "passage_finder.h"


// *****************************************
//              Vector Math
// *****************************************
pcl::PointXYZ VectorMath::cross(pcl::PointXYZ p1, pcl::PointXYZ p2) {
    pcl::PointXYZ result;
    result.x = p1.y * p2.z - p1.z * p2.y;
    result.y = p1.z * p2.x - p1.x * p2.z;
    result.z = p1.x * p2.y - p1.y * p2.x;
    return result;
};

double VectorMath::dot(pcl::PointXYZ p1, pcl::PointXYZ p2) {
    return p1.x * p2.x + p1.y * p2.y + p1.z * p2.z;
};

double VectorMath::len(pcl::PointXYZ p1) {
    return sqrt(dot(p1, p1));
};

pcl::PointXYZ VectorMath::to_e(pcl::PointXYZ p1) {
    pcl::PointXYZ result;
    result.x = p1.x / len(p1);
    result.y = p1.y / len(p1);
    result.z = p1.z / len(p1);
    return result;
};



// *****************************************
//              Advanced Passage Finder
// *****************************************
void Advanced_Passage_finder::renew(const ransac_slam::LineMap::ConstPtr& lines_msg)
{
    this->passages.clear();
    for (int i = 0; i < lines_msg->pnumber; ++i) {
        this->add_passage(lines_msg->header.frame_id,
                 pcl::PointXYZ(lines_msg->pleft.at(i).x, lines_msg->pleft.at(i).y, lines_msg->pleft.at(i).z),
                 pcl::PointXYZ(lines_msg->prght.at(i).x, lines_msg->prght.at(i).y, lines_msg->prght.at(i).z));
    }
};


void Advanced_Passage_finder::add_passage(std::string frame, pcl::PointXYZ point1, pcl::PointXYZ point2)
{
    Passage new_passage;

    if (!isnan(point1.x) && !isnan(point1.y) && !isnan(point1.x) &&
        !isnan(point2.x) && !isnan(point2.y) && !isnan(point2.x))
        new_passage.is_nan = false;

    new_passage.kin_left.x = point1.x;
    new_passage.kin_left.y = point1.z;
    new_passage.kin_rght.x = point2.x;
    new_passage.kin_rght.y = point2.z;

    new_passage.width = sqrt(pow(point1.x - point2.x, 2.0) + pow(point1.z - point2.z, 2.0));

    new_passage.kin_middle.x = (point1.x + point2.x) / 2;
    new_passage.kin_middle.y = (point1.z + point2.z) / 2;

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

    int best_i = 0;
    for (int i = 0; i < linemap.lines.size(); ++i) {
        if (linemap.lines.at(i).distance_to_point(point.x, point.y) < eps) {
            best_i = i;
        }
    }

    return &linemap.lines.at(best_i);
};


pcl::PointXYZ Advanced_Passage_finder::get_closest_left(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, int point_id)
{
    // Return the geometrically closest point on the left from the given point. A given point is specified by its id in the cloud
    if (point_id < 1 || cloud->points.size() < point_id + 1) { // There should be at least one point on the left
        ROS_ERROR("[action_server]: Advanced_Passage_finder::get_closest_left: invalid arguments");
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
        ROS_ERROR("[action_server]: Advanced_Passage_finder::get_closest_rght: invalid arguments");
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
        ROS_ERROR("[action_server]: Advanced_Passage_finder::sqrange: getting distance from NAN!");
    }

    return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
};


Line_param* Advanced_Passage_finder::get_best_opposite_line(pcl::PointXYZ pass_point_kin, pcl::PointXYZ pass_point_kin_op, Line_map &linemap) {
    Line_param *pass_line_op;
    Line_param *pass_line = this->get_best_line(pass_point_kin, linemap);
    double scalar_mul = 2;
    if (pass_line != NULL) {
        for (int i = 0; i < linemap.lines.size(); ++i) {
            double tmp = pass_line->ldir_vec.cmd.x * linemap.lines.at(i).ldir_vec.cmd.x +
                         pass_line->ldir_vec.cmd.y * linemap.lines.at(i).ldir_vec.cmd.y;
            if (tmp < scalar_mul) {
                scalar_mul = tmp;
                pass_line_op = &linemap.lines.at(i);
            }
        }
    }
    else
        return NULL;
    return pass_line_op;
}




// *****************************************
// 				Location server
// *****************************************
void LocationServer::track_wall(Line_param *wall)
{
	if (wall == NULL) {
		ROS_ERROR("[action_server]: LocationServer::track_wall argument is NULL");
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



void LocationServer::spin_once(const ransac_slam::LineMap::ConstPtr& lines)
{
    if(lines->number == 0) {
        ROS_WARN("[action_server]: The cloud is empty. Location server is unable to provide pose estimation. Skipping...");
        if (this->ref_wall != NULL) {
            this->ref_wall->found = false;
        }
        return;
    };

    if (this->ref_wall == NULL) {
        ROS_WARN("[action_server]: No ref_wall. Using random!");
    };

    this->lm.renew(lines);
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
		ROS_ERROR("[action_server]: MotionServer::set_ref_wall argument is NULL");
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
		ROS_ERROR("[action_server]: MotionServer::set_angles_current ref_wall == NULL");
		return;
	}

	this->ref_ang = this->ref_wall->angle;
	this->ref_dist = this->ref_wall->distance;
};


bool MotionServer::rotate(double angle)
{
	if (this->tracking_on && this->ref_wall == NULL) {
		ROS_ERROR("[action_server]: MotionServer::rotate ref_wall == NULL");
		return false;
	}

	this->ref_ang += angle;
	return true;
};


bool MotionServer::move_parallel(double vel)
{
	if (this->tracking_on && this->ref_wall == NULL) {
		ROS_ERROR("[action_server]: MotionServer::move_parallel ref_wall == NULL");
		return false;
	}

	buf_cmd.linear.x  = - vel * this->ref_wall->ldir_vec.cmd.x;
	buf_cmd.linear.y  = - vel * this->ref_wall->ldir_vec.cmd.y;

	return true;
};


bool MotionServer::move_perpendicular(double shift)
{
    if (this->tracking_on && this->ref_wall == NULL) {
        ROS_ERROR("[action_server]: MotionServer::move_perpendicular ref_wall == NULL");
        return false;
    }

	this->ref_dist += shift;
	return true;
};


void MotionServer::spin_once()
{
    if (this->tracking_on && this->ref_wall == NULL) {
		ROS_ERROR("[action_server]: MotionServer::spin_once ref_wall == NULL");
		return;
	}

	if (this->tracking_on && this->ref_dist < 0.6) {
    	ROS_WARN("[action_server]: Invalid ref_dist (%f)", this->ref_dist);
		this->ref_dist = 0.6;
	}

	if (this->tracking_on) {
	    this->base_cmd.angular.z = - this->pid_ang.get_output(this->ref_ang, this->ref_wall->angle);

        double vel_k = - this->pid_vel.get_output(this->ref_dist, this->ref_wall->distance);
        this->base_cmd.linear.x  += vel_k * this->ref_wall->fdir_vec.cmd.x + this->buf_cmd.linear.x;
        this->base_cmd.linear.y  += vel_k * this->ref_wall->fdir_vec.cmd.y + this->buf_cmd.linear.y;
        this->base_cmd.linear.z  += this->buf_cmd.linear.z;
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

void MotionServer::move(pcl::PointXYZ target, double vel, double phi, double rot_vel) {
    this->lock();
    if (isnan(target.x) || isnan(target.y) || isnan(phi)) {
        this->move_done = true;
        this->rot_done = true;
        this->unlock();
        return;
    }

    this->move_target = target;
    this->move_vel = vel;
    this->move_phi = phi;
    this->move_rot_vel = rot_vel;
    this->move_done = false;
    this->rot_done = false;
    this->prev_angl = this->map_srv->get_global_angle();
    this->prev_pos = this->map_srv->get_global_positon();
    this->untrack();

    this->unlock();
}

void MotionServer::set_height(double height) {
    this->lock();
    if (isnan(height)) {
        this->height_done = true;
        this->unlock();
        return;
    }

    this->target_height = height;
    this->height_done = false;

    this->unlock();
}

void MotionServer::move_step() {
    this->untrack();
    double current_angl = this->map_srv->get_global_angle();
    double delta_angl   = this->map_srv->diff(current_angl, prev_angl);
    this->prev_angl     = current_angl;

    this->move_target = this->map_srv->do_transform(this->move_target);

    double len = sqrt (this->move_target.x * this->move_target.x +
                       this->move_target.y * this->move_target.y);

    if (len < this->move_epsilon) {
        move_done = true;
    }
    else {
        this->buf_cmd.linear.x = this->move_target.x * this->move_vel / len;
        this->buf_cmd.linear.y = this->move_target.y * this->move_vel / len;
        this->buf_cmd.angular.z = 0;
    }
    //TODO: conform sign of phi and rot_vel
    if (this->move_phi < this->move_rot_epsilon) {
        rot_done = true;
    }
    else {
        this->move_phi -= fabs(delta_angl) < 180 ? fabs(delta_angl) : 360 - fabs(delta_angl);
        this->move_phi = this->move_phi > 360 ? (this->move_phi - 360) : this->move_phi;
        this->buf_cmd.angular.z = this->move_rot_vel;
    }
}

void MotionServer::altitude_step() {
    PID pid_vel(1, 0.5, 0.5);

    tf::StampedTransform transform;
    try { this->tf_listener.lookupTransform(base_footprint_frame, base_stabilized_frame, ros::Time(0), transform); }
    catch (tf::TransformException &ex) { ROS_ERROR("[action_server]: (lookup) Unable to transform: %s", ex.what()); }

    double tmp_vel = pid_vel.get_output(this->target_height, transform.getOrigin().z());

    this->height = transform.getOrigin().z();

    if (fabs(this->target_height - this->height) < this->height_epsilon) {
        this->height_done = true;
    }

    if (fabs(this->prev_height - this->height) < this->height_epsilon / 10) {
        this->on_floor = true;
    }
    else {
        this->on_floor = false;
    }

    if (!(this->on_floor && tmp_vel < 0)) {
        this->buf_cmd.linear.z = tmp_vel;
    }

    this->vert_vel = tmp_vel;

    char text[20];
    sprintf(text, "Height = %f", this->height);
    this->height_text.text = text;
    this->pub_mrk.publish(this->height_text);
}

// *****************************************
//              Mapping server
// *****************************************
MappingServer::MappingServer()
{
    init_flag =   false; // Look into class defenition
    rotation_cnt    = 0;
    mutex = boost::shared_ptr<boost::mutex>   (new boost::mutex);
    aver_land_pad   = pcl::PointXYZ (0, 0, 0);

    try {
        this->tf_listener.waitForTransform(fixed_frame, pointcloud_frame, ros::Time(0), ros::Duration(10.0) );
        this->tf_listener.waitForTransform(fixed_frame, base_stabilized_frame, ros::Time(0), ros::Duration(10.0) );
        this->tf_listener.waitForTransform(pointcloud_frame, fixed_frame, ros::Time(0), ros::Duration(10.0) );
    }
    catch (tf::TransformException &ex) { ROS_ERROR("[action_server]: (wait) Unable to transform: %s", ex.what()); }
};


void MappingServer::spin_once()
{
    this->lock();

    tf::StampedTransform transform;
    try { this->tf_listener.lookupTransform(fixed_frame, pointcloud_frame, ros::Time(0), transform); }
    catch (tf::TransformException &ex) { ROS_ERROR("[action_server]: (lookup) Unable to transform: %s", ex.what()); }

    if (this->init_flag == false) {
        this->prev_transform = transform;
        this->init_flag = true;
        this->unlock();
        return;
    }

    for (int i = 0; i < this->tracked_points.size(); ++i) {
        this->tracked_points.at(i) = do_transform(this->tracked_points.at(i));
    }

    for (int i = 0; i < this->visited_points.size(); ++i) {
        this->visited_points.at(i) = do_transform(this->visited_points.at(i));
    }

    for (int i = 0; i < this->landing_points.size(); ++i) {
	this->landing_points.at(i) = do_transform(this->landing_points.at(i));
    }

    for (int i = 0; i < this->visited_points.size(); ++i) {
        davinci->draw_point_cmd(this->visited_points.at(i).x, this->visited_points.at(i).y, 0.04, 494 + i, GOLD);
    }

    for (int i = 0; i < this->tracked_points.size(); ++i) {
        davinci->draw_point_cmd(this->tracked_points.at(i).x, this->tracked_points.at(i).y, 994 + i, CYAN);
    }

    this->prev_transform = transform;

    aver_land_pad = pcl::PointXYZ(0, 0, 0);

    for (int i = 0; i < this->landing_points.size(); ++i) {
        aver_land_pad.x += this->landing_points.at(i).x;
        aver_land_pad.y += this->landing_points.at(i).y;
        aver_land_pad.z += this->landing_points.at(i).z;
        davinci->draw_point_cmd(this->landing_points.at(i).x, this->landing_points.at(i).y, 1294 + i, BLUE);
    }
    if (this->landing_points.size() != 0) {
        aver_land_pad.x /= this->landing_points.size();
        aver_land_pad.y /= this->landing_points.size();
        aver_land_pad.z /= this->landing_points.size();
    }
    davinci->draw_point_cmd(this->aver_land_pad.x, this->aver_land_pad.y, 5577, RED);

    this->add_visited();
    this->resize_land_pads();
    this->unlock();
};


pcl::PointXYZ MappingServer::do_transform(const pcl::PointXYZ point) {
    tf::StampedTransform transform;
    try { this->tf_listener.lookupTransform(fixed_frame, pointcloud_frame, ros::Time(0), transform); }
    catch (tf::TransformException &ex) { ROS_ERROR("[action_server]: (lookup) Unable to transform: %s", ex.what()); }
    tf::Transform past_to_current = transform.inverseTimes(this->prev_transform);
    tf::Vector3 point_(point.x, point.y, point.z);
    point_ = past_to_current * point_;
    return pcl::PointXYZ(point_.getX(), point_.getY(), point_.getZ());
};



int MappingServer::track (pcl::PointXYZ p)
{
    this->lock();
    this->tracked_points.push_back(p);
    this->unlock();
    return this->tracked_points.size();
};

int MappingServer::add_land_pad (pcl::PointXYZ p)
{
    this->lock();
    this->landing_points.push_back(p);
    this->unlock();
    return this->landing_points.size();
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

void MappingServer::resize_land_pads ()
{
    const unsigned int max_size_ = 10;
    if (this->landing_points.size() > max_size_) {
        this->landing_points.erase (this->landing_points.begin());
    }
};

pcl::PointXYZ MappingServer::get_global_positon()
{
    this->lock();
    tf::StampedTransform transform;
    try { this->tf_listener.lookupTransform(fixed_frame, base_stabilized_frame, ros::Time(0), transform); }
    catch (tf::TransformException &ex) { ROS_ERROR("[action_server]: (lookup) Unable to transform: %s", ex.what()); }
    pcl::PointXYZ ret (transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
    this->unlock();
    return ret;
};


double MappingServer::get_global_angle()
{
    this->lock();
    tf::StampedTransform transform;
    try { this->tf_listener.lookupTransform(fixed_frame, base_stabilized_frame, ros::Time(0), transform); }
    catch (tf::TransformException &ex) { ROS_ERROR("[action_server]: (lookup) Unable to transform: %s", ex.what()); }
    double roll, pitch, yaw;
    tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
    this->unlock();
    return yaw;
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
    c.z = a.z - b.z;
    return c;
};

void MappingServer::clear_land_pad() {
    for (int i = 0; i < this->landing_points.size(); i++) {
        this->landing_points.erase (this->landing_points.begin());
    }
    aver_land_pad.x = 0;
    aver_land_pad.y = 0;
    aver_land_pad.z = 0;
}


// *****************************************
//              Passage type recognition
// *****************************************
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
    Line_param *pass_line = apf->get_best_line(pass_point_kin, lm);
    Line_param *pass_line_op = apf->get_best_opposite_line(pass_point_kin, pass_point_kin_op, lm);

    double scalar_mul = NAN;
    if (pass_line != NULL && pass_line_op != NULL) {
        scalar_mul = pass_line->ldir_vec.cmd.x * pass_line_op->ldir_vec.cmd.x +
                     pass_line->ldir_vec.cmd.y * pass_line_op->ldir_vec.cmd.y;
    }

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
        fabs(scalar_mul) <= 1 && fabs(scalar_mul) < 0.1) {
        this->type = ortogonal;
        return ortogonal;
    }

    if (this->pass_exist && this->middle_exist &&
        fabs(scalar_mul) <= 1 && fabs(scalar_mul - 1) < 0.1) {
        this->type = parrallel;
        return parrallel;
    }

    this->type = undefined;
    return undefined;
};

