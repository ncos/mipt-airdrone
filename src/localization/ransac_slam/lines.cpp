#include "lines.h"



// *****************************************
//              Line map
// *****************************************
Line_map::Line_map ()
{
    this->coefficients = pcl::ModelCoefficients::Ptr (new pcl::ModelCoefficients ());
    this->inliers      = pcl::PointIndices::Ptr      (new pcl::PointIndices ());
    // Optional

    this->seg.setOptimizeCoefficients (true);
    this->seg.setModelType (pcl::SACMODEL_LINE);
    this->seg.setMethodType (pcl::SAC_RANSAC);
    this->seg.setDistanceThreshold (0.003);

    this->cloud_f = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
};


void Line_map::renew (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	bool lines_cleared = false;
	(*cloud_f).clear();
	(*cloud_f) += *cloud;

	int nr_points = (int) cloud_f->points.size ();

	while (cloud_f->points.size () > 0.06 * nr_points)
	{
		// Segment the largest planar component from the remaining cloud
	    seg.setInputCloud (cloud_f);
	    seg.segment (*inliers, *coefficients);
	    if (inliers->indices.size () <= min_points_in_line) break;
	    else if (!lines_cleared) {lines.clear(); lines_cleared = true; }

	    Line_param lp;
	    lp.found = true;
	    lp.renew(coefficients, cloud, inliers);

	    lines.push_back(lp);

	    // Create the filtering object
	    extract.setInputCloud (cloud_f);
	    extract.setIndices (inliers);
	    extract.setNegative (true);
	    extract.filter (*cloud_f);
	}
};

Line_param Line_map::get_best_fit (double angle, double distance)
{
    std::vector<Line_param> l;
    for (int i = 0; i < this->lines.size(); ++i) {
        l.push_back(lines[i]);
    }
    lineCmp line_cmp_class(angle, distance);
    std::sort (l.begin(), l.end(), line_cmp_class);
    return l[0];
};

Line_param Line_map::get_best_fit_a (double angle)
{
    std::vector<Line_param> l;
    this->sort_lines_a(angle, l);
    return l[0];
};

Line_param Line_map::get_best_fit_d (double distance)
{
    std::vector<Line_param> l;
    this->sort_lines_d(distance, l);
    return l[0];
};

void Line_map::sort_lines_a (double angle, std::vector<Line_param> &l)
{
    for (int i = 0; i < this->lines.size(); ++i) {
        l.push_back(lines[i]);
    }
    lineCmp line_cmp_class(angle, NAN);
    std::sort (l.begin(), l.end(), line_cmp_class);

};

void Line_map::sort_lines_d (double distance, std::vector<Line_param> &l)
{
    for (int i = 0; i < this->lines.size(); ++i) {
        l.push_back(lines[i]);
    }
    lineCmp line_cmp_class(NAN, distance);
    std::sort (l.begin(), l.end(), line_cmp_class);

};


// *****************************************
//              Brute force matcher
// *****************************************
double BruteForceMatcher::match(std::vector<Line_param> l1, std::vector<Line_param> l2,
                                std::vector<BruteForceMatcher::Pair> &matched, std::vector<Line_param> &unmatched) {
    const double unmatch_penalty = 90;
    if (l1.size() == 0) {
        unmatched = l2;
        return unmatch_penalty * unmatched.size();
    }

    if (l2.size() == 0) {
        unmatched = l1;
        return unmatch_penalty * unmatched.size();
    }

    Line_param query_line = l1.back();
    // vec1: 2 3 4 5
    // vec2: 1 2 3 4 5 ('1' vanished from the view)
    l1.pop_back();
    std::vector<BruteForceMatcher::Pair> matched_0;
    std::vector<Line_param> unmatched_0;
    double err_0 = match(l1, l2, matched_0, unmatched_0) + unmatch_penalty;

    // vec1: 2 3 4 5
    // vec2: 1 2 4 5 ('1' did not vanish)
    unsigned int l2_id = get_best_fit(query_line, l2);
    BruteForceMatcher::Pair new_pair(query_line, l2[l2_id]);
    l2.erase (l2.begin() + l2_id);
    std::vector<BruteForceMatcher::Pair> matched_1;
    std::vector<Line_param> unmatched_1;
    double err_1 = match(l1, l2, matched_1, unmatched_1) + new_pair.err;

    if (err_0 < err_1) {
        matched = matched_0;
        unmatched = unmatched_0;
        unmatched.push_back(query_line);
        return err_0;
    }
    else {
        matched = matched_1;
        unmatched = unmatched_1;
        matched.push_back(new_pair);
        return err_1;
    }
};

unsigned int BruteForceMatcher::get_best_fit(Line_param &line, std::vector<Line_param> &lines) {
    // return the id in the 'lines' vector that gives the lowest error
    unsigned int id = 0;
    if (lines.size() == 0) {
        ROS_ERROR("ransac_slam/lines.cpp: 'lines' vector is empty!");
        return 0;
    }

    double err = LineMetrics::get_error(line, lines[0]);
    for (unsigned int i = 0; i < lines.size(); ++i) {
        if (lines[i].frame != line.frame) {
            ROS_ERROR("ransac_slam/lines.cpp: Unable to compare\
                       lines with different frames! ('%s' and '%s')", lines[i].frame.c_str(), line.frame.c_str());
            return i;
        }
        double err_ = LineMetrics::get_error(line, lines[i]);
        if (err_ < err) {
            err = err_;
            id = i;
        }
    }
    return id;
};


// *****************************************
//              Location server
// *****************************************
nav_msgs::Odometry LocationServer::spin_once(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, tf::Transform& fixed_to_base,
                                                                                                    tf::Transform& base_to_cloud,
                                                                                                    tf::Transform& fixed_to_cloud,
                                                                                                    tf::TransformListener& tf_listener)
{
    nav_msgs::Odometry result_pose;
    result_pose.pose.covariance =   boost::assign::list_of(1e6)   (0) (0)  (0)  (0)  (0)
                                                          (0)  (1e6)  (0)  (0)  (0)  (0)
                                                          (0)   (0)  (1e6) (0)  (0)  (0)
                                                          (0)   (0)   (0) (1e6) (0)  (0)
                                                          (0)   (0)   (0)  (0) (1e6) (0)
                                                          (0)   (0)   (0)  (0)  (0)  (1e6);
    result_pose.twist.covariance =  boost::assign::list_of(1e6)   (0) (0)  (0)  (0)  (0)
                                                          (0)  (1e6)  (0)  (0)  (0)  (0)
                                                          (0)   (0)  (1e6) (0)  (0)  (0)
                                                          (0)   (0)   (0) (1e6) (0)  (0)
                                                          (0)   (0)   (0)  (0) (1e6) (0)
                                                          (0)   (0)   (0)  (0)  (0)  (1e6);
    if (cloud->points.size() < min_points_in_cloud) {
        ROS_WARN("[ransac_slam]: The cloud is too small (%lu points). Location server is unable \
                                        to provide reliable pose estimation", cloud->points.size());
        return result_pose;
    }
    this->pf.renew(cloud);
    std::vector<Line_param> lines_old = this->lm.lines;
    this->lm.renew(cloud);

    if (this->lm.lines.size() > 10) {
        ROS_WARN("[ransac_slam]: The number of walls detected is too large to compute the transform (%lu lines). \
                  Maybe your input flat cloud is faulty - try reducing flat_cloud_min/max difference", this->lm.lines.size());
        return result_pose;
    }

    std::vector<BruteForceMatcher::Pair> matched;
    std::vector<Line_param> unmatched;
    BruteForceMatcher::match(lines_old, this->lm.lines, matched, unmatched);
    this->match_list = matched;

    double delta_yaw = 0;
    this->estimate_rotation(matched, delta_yaw);

    pcl::PointXYZ shift(0.0, 0.0, 0.0);
    this->estimate_shift(matched, shift);

    // Rotating the shift vector:
    geometry_msgs::PointStamped cloud_point_start, cloud_point_end;
    geometry_msgs::PointStamped fixed_point_start, fixed_point_end;
    cloud_point_end.header.frame_id = kinect_depth_optical_frame;
    cloud_point_end.header.stamp = ros::Time();
    cloud_point_end.point.x = shift.x;
    cloud_point_end.point.y = shift.y;
    cloud_point_end.point.z = shift.z;
    cloud_point_start.header.frame_id = kinect_depth_optical_frame;
    cloud_point_start.header.stamp = ros::Time();
    cloud_point_start.point.x = 0;
    cloud_point_start.point.y = 0;
    cloud_point_start.point.z = 0;

    try {
       tf_listener.transformPoint(fixed_frame, cloud_point_start, fixed_point_start);
       tf_listener.transformPoint(fixed_frame, cloud_point_end,   fixed_point_end);
    }
    catch(tf::TransformException& ex) {
        ROS_ERROR("[ransac_slam]: Received an exception trying to transform a point from \"kinect\" to \"odom\": %s", ex.what());
    }
    shift.x = fixed_point_end.point.x - fixed_point_start.point.x;
    shift.y = fixed_point_end.point.y - fixed_point_start.point.y;
    shift.z = fixed_point_end.point.z - fixed_point_start.point.z;

    result_pose.pose.pose.position.x = fixed_to_base.getOrigin().x() + shift.x;
    result_pose.pose.pose.position.y = fixed_to_base.getOrigin().y() + shift.y;
    result_pose.pose.pose.position.z = this->range_from_sonar;

    double rotation_err = 1e-4;
    double transl_err   = 1e-2;
    tf::Quaternion res = fixed_to_base.getRotation() * tf::createQuaternionFromRPY(0, 0, delta_yaw * M_PI / 180.0);
    tf::quaternionTFToMsg(res, result_pose.pose.pose.orientation);

    if (matched.size() == 0) rotation_err = 1e6;
    if (matched.size() == 0) transl_err   = 1e6;

    result_pose.pose.covariance =   boost::assign::list_of(transl_err)  (0) (0)  (0)  (0)  (0)
                                                          (0) (transl_err)  (0)  (0)  (0)  (0)
                                                          (0)   (0)  (1e-4) (0)  (0)  (0)
                                                          (0)   (0)   (0) (1e6) (0)  (0)
                                                          (0)   (0)   (0)  (0) (1e6) (0)
                                                          (0)   (0)   (0)  (0)  (0)  (rotation_err);
    result_pose.twist.twist.angular.x = 0;
    result_pose.twist.twist.angular.y = 0;
    result_pose.twist.twist.angular.z = 0;
    result_pose.twist.twist.linear.x  = 0;
    result_pose.twist.twist.linear.y  = 0;
    result_pose.twist.twist.linear.z  = 0;
    result_pose.twist.covariance =  boost::assign::list_of(1e6)  (0) (0)  (0)  (0)  (0)
                                                          (0) (1e6)  (0)  (0)  (0)  (0)
                                                          (0)   (0)  (1e6) (0)  (0)  (0)
                                                          (0)   (0)   (0) (1e6) (0)  (0)
                                                          (0)   (0)   (0)  (0) (1e6) (0)
                                                          (0)   (0)   (0)  (0)  (0)  (1e6);

    result_pose.child_frame_id  = base_frame;
    result_pose.header.frame_id = fixed_frame;
    result_pose.header.stamp = ros::Time::now();
    return result_pose;
};


void LocationServer::estimate_rotation(std::vector<BruteForceMatcher::Pair> &matched, double &delta_yaw)
{
    delta_yaw = 0;
    int divider = 0;
    for (int i = 0; i < matched.size(); ++i) {
        if (matched.at(i).err > 10) continue;
        pcl::PointXYZ cr = VectorMath::cross(matched.at(i).l1.fdir_vec, matched.at(i).l2.fdir_vec);
        double delta_yaw_sin = cr.y;
        divider ++;
        delta_yaw += asin(delta_yaw_sin) * 180 / M_PI;
    }

    if (divider != 0) delta_yaw /= divider;
};

void LocationServer::estimate_shift(std::vector<BruteForceMatcher::Pair> &matched, pcl::PointXYZ &shift)
{
    shift = pcl::PointXYZ(0.0, 0.0, 0.0);
    const double ortog_eps = 0.5;
    unsigned int cnt1 = 0, cnt2 = 0;

    pcl::PointXYZ shift1(NAN, NAN, NAN), shift2(0.0, 0.0, 0.0);

    for (int i = 0; i < matched.size(); ++i) {
        if (matched.at(i).err > 10) continue;
        double d = matched.at(i).l1.distance - matched.at(i).l2.distance;
        if (isnan(shift1.x)) {
            shift1.x = matched.at(i).l2.fdir_vec.x * d;
            shift1.y = matched.at(i).l2.fdir_vec.y * d;
            shift1.z = matched.at(i).l2.fdir_vec.z * d;
            cnt1 ++;
            continue;
        }
        if (VectorMath::dot(shift1, matched.at(i).l2.fdir_vec) >= ortog_eps) {
            shift1.x += matched.at(i).l2.fdir_vec.x * d;
            shift1.y += matched.at(i).l2.fdir_vec.y * d;
            shift1.z += matched.at(i).l2.fdir_vec.z * d;
            cnt1 ++;
        }
        else {
            shift2.x += matched.at(i).l2.fdir_vec.x * d;
            shift2.y += matched.at(i).l2.fdir_vec.y * d;
            shift2.z += matched.at(i).l2.fdir_vec.z * d;
            cnt2 ++;
        }
    }
    if (cnt1 != 0) {
        shift1.x /= cnt1;
        shift1.y /= cnt1;
        shift1.z /= cnt1;
    }
    else {
        shift1.x = 0;
        shift1.y = 0;
        shift1.z = 0;
    }

    if (cnt2 != 0) {
        shift2.x /= cnt2;
        shift2.y /= cnt2;
        shift2.z /= cnt2;
    }
    else {
        shift2.x = 0;
        shift2.y = 0;
        shift2.z = 0;
    }

    shift.x = shift1.x + shift2.x;
    shift.y = shift1.y + shift2.y;
    shift.z = shift1.z + shift2.z;
};


