//
// Created by teng on 18-5-1.
//

#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <string>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/correspondence_rejection.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/registration/correspondence_rejection_var_trimmed.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/features/normal_3d.h>

using namespace std;

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

    boost::shared_ptr<pcl::Correspondences> correspondences (new pcl::Correspondences);
    pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> corr_est;
    corr_est.setInputCloud (cloud_in);
    corr_est.setInputTarget (cloud_out);
    corr_est.determineCorrespondences (*correspondences,.1);
    std::cout << "No. of correspondence is [*correspondences] " << int (correspondences->size ()) << std::endl;

//    boost::shared_ptr<pcl::Correspondences> correspondences_remain (new pcl::Correspondences);
//    pcl::registration::CorrespondenceRejectorMedianDistance rej;
//    rej.setMedianFactor (8.79241104);
//    rej.setInputCorrespondences (correspondences);
//    rej.getCorrespondences (*correspondences_remain);
//    std::cout << "No. of remain correspondence is " << int (correspondences_remain->size ()) << std::endl;

    return (0);
}