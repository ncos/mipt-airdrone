#include "passage.h"


void AdvancedPassageFinder::renew(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
    this->passages.clear();
    const int max_id = cloud->points.size() - 1;
    const int min_id = 0;

    if (cloud->points.size() < 2) return;


    for (int i = 1; i < cloud->points.size(); ++i) {
        if (apf_better_q == false) {
            if (this->sqrange(cloud->points.at(i - 1), cloud->points.at(i)) > apf_min_width * apf_min_width) {
                this->add_passage(cloud->header.frame_id, cloud->points.at(i - 1), cloud->points.at(i));
            }
        }

        if (apf_better_q == true) {
            if (this->sqrange(cloud->points.at(i - 1), cloud->points.at(i)) > apf_min_width * apf_min_width) {
                bool edge_left = ((cloud->points.at(i - 1).z > apf_min_dist) && (cloud->points.at(i - 1).z < apf_max_dist));
                bool edge_rght = ((cloud->points.at(i).z     > apf_min_dist) && (cloud->points.at(i).z     < apf_max_dist));

                if (edge_left &&  edge_rght) {
                    this->add_passage(cloud->header.frame_id, cloud->points.at(i - 1), cloud->points.at(i));
                }

                if (edge_left && !edge_rght) {
                    pcl::PointXYZ rght = this->get_closest_rght(cloud, i - 1);
                    if (this->sqrange(cloud->points.at(i - 1), rght) > apf_min_width * apf_min_width) {
                        this->add_passage(cloud->header.frame_id, cloud->points.at(i - 1), rght);
                    }
                }

                if (!edge_left && edge_rght) {
                    pcl::PointXYZ left = this->get_closest_left(cloud, i);
                    if (this->sqrange(left, cloud->points.at(i)) > apf_min_width * apf_min_width) {
                        this->add_passage(cloud->header.frame_id, left, cloud->points.at(i));
                    }
                }
            }
        }
    }

    // Check the leftmost and rightmost point (this is in case only one side of the passage is seen)
    double left_ang = atan(cloud->points.at(min_id).x / cloud->points.at(min_id).z) * 180 / M_PI;  // The angle of the leftmost point
    double rght_ang = atan(cloud->points.at(max_id).x / cloud->points.at(max_id).z) * 180 / M_PI;  // The angle of the rightmost point


    // RIGHT (!important!) passage point detection
    if ((cloud->points.at(min_id).z > apf_min_dist) && (cloud->points.at(min_id).z < apf_max_dist) && (left_ang > apf_min_angl)) {
        // If this is the leftmost point, the passage is possibly even more at the left, so this point is the (supposedly)
        // RIGHT point of the passage (if rotate left and look directly at the passage this point will be on the right)
        add_passage(cloud->header.frame_id, pcl::PointXYZ(NAN, NAN, NAN), cloud->points.at(min_id));
    }

    // LEFT (!important!) passage point detection
    if ((cloud->points.at(max_id).z > apf_min_dist) && (cloud->points.at(max_id).z < apf_max_dist) && (rght_ang < apf_max_angl)) {
        // This is similar to the RIGHT point detection
        add_passage(cloud->header.frame_id, cloud->points.at(max_id), pcl::PointXYZ(NAN, NAN, NAN));
    }
};


void AdvancedPassageFinder::add_passage(std::string frame, pcl::PointXYZ point1, pcl::PointXYZ point2)
{
    Passage new_passage;

    if (!isnan(point1.x) && !isnan(point1.y) && !isnan(point1.x) &&
        !isnan(point2.x) && !isnan(point2.y) && !isnan(point2.x))
        new_passage.is_nan = false;

    new_passage.frame = frame;
    new_passage.left  = point1;
    new_passage.rght  = point2;

    new_passage.width = sqrt(VectorMath::dot(new_passage.left, new_passage.rght));

    new_passage.middle.x = (point1.x + point2.x) / 2;
    new_passage.middle.y = (point1.y + point2.y) / 2;
    new_passage.middle.z = (point1.z + point2.z) / 2;

    new_passage.left_ang = atan(new_passage.left.x   / new_passage.left.z)   * 180 / M_PI;
    new_passage.rght_ang = atan(new_passage.rght.x   / new_passage.rght.z)   * 180 / M_PI;
    new_passage.mid_ang  = atan(new_passage.middle.x / new_passage.middle.z) * 180 / M_PI;

    this->passages.push_back(new_passage);
};


pcl::PointXYZ AdvancedPassageFinder::get_closest_left(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, int point_id)
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


pcl::PointXYZ AdvancedPassageFinder::get_closest_rght(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, int point_id)
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


double AdvancedPassageFinder::sqrange(pcl::PointXYZ p1, pcl::PointXYZ p2)
{
    if (isnan(p1.x) || isnan(p1.y) || isnan(p1.z) || isnan(p2.x) || isnan(p2.y) || isnan(p2.z)) {
        ROS_ERROR("Advanced_Passage_finder::sqrange: getting distance from NAN!");
    }

    return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
};
