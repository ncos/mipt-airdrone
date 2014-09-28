#ifndef PASSAGE_H
#define PASSAGE_H

#include <vector>
#include <cmath>
#include <assert.h>

#include <ros/ros.h>
#include <pcl/point_cloud.h>

#include "advanced_types.h"


extern double apf_min_width;
extern double apf_min_dist;
extern double apf_max_dist;
extern double apf_min_angl;
extern double apf_max_angl;
extern bool   apf_better_q;


class AdvancedPassageFinder // Da ADVANCED Passage findr! (looks for holes in walls and is better than Passage findr)
{
public:
    std::vector<Passage> passages;
    void renew(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);

private:
    void add_passage(std::string frame, pcl::PointXYZ point1, pcl::PointXYZ point2);
    double sqrange(pcl::PointXYZ p1, pcl::PointXYZ p2);
    pcl::PointXYZ get_closest_left(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, int point_id);
    pcl::PointXYZ get_closest_rght(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, int point_id);
};



#endif // PASSAGE_H
