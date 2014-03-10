#ifndef PCV_H
#define PCV_H


#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

class PCV
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    PCV() {};

public:
    PCV(	pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr);

    PCV( 	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr);

    PCV(	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr,
    		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1);

    void redraw ();
    bool wasStopped ();
    //void draw ();


private:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis      (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis         (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis     (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
                                                                         pcl::PointCloud<pcl::Normal>::ConstPtr normals);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> shapesVis      (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewportsVis   (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
                                                                         pcl::PointCloud<pcl::Normal>::ConstPtr normals1,
                                                                         pcl::PointCloud<pcl::Normal>::ConstPtr normals2);
};

#endif
