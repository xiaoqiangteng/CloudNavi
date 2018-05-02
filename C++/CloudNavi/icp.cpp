//
// Created by teng on 18-4-22.
//

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <string>

int
main (int argc, char** argv)
{
    std::string cloud1 = argv[1];
    std::string cloud2 = argv[2];
    std::cout << "Reading " << cloud1 << " and " << cloud2 << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
    if((pcl::io::loadPCDFile<pcl::PointXYZ> (cloud1, *cloud_in) == -1) || (pcl::io::loadPCDFile<pcl::PointXYZ> (cloud2, *cloud_out) == -1)) // load the file
    {
        PCL_ERROR ("Couldn't read file");
        return -1;
    }
    std::cout << "points of the first cloud: " << cloud_in->points.size () <<std::endl;
    std::cout << "points of the second cloud: " << cloud_out->points.size () <<std::endl;

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputCloud(cloud_in);
    icp.setInputTarget(cloud_out);
    icp.setMaxCorrespondenceDistance(0.2);
    icp.setTransformationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(1);
    icp.setMaximumIterations(100);
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
              icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;
    std::cout << "ICP: No of correspondences is " << icp.correspondences_.use_count() << std::endl;

    return (0);
}