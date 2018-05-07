//
// Created by teng on 18-4-22.
//

/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2011, Willow Garage, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 * * Neither the name of Willow Garage, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 *
 */

// STL
#include <iostream>
#include <pcl/console/parse.h>
#include <fstream>
#include <iomanip>
#include <string>
#include <stdlib.h>
#include <vector>
#include <dirent.h>
#include <time.h>
// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/normal_3d.h>
// #include <pcl/visualization/pcl_visualizer.h>

/* This example shows how to estimate the SIFT points based on the
 * Normal gradients i.e. curvature than using the Intensity gradient
 * as usually used for SIFT keypoint estimation.
 */

using namespace std;

// This function displays the help
void showHelp(char * program_name)
{
    std::cout << std::endl;
    std::cout << "Usage: " << program_name << " path[PCD] file_name[results] " << std::endl;
    std::cout << "-h:  Show this help." << std::endl;
}

bool GetFiles(const std::string filePath, std::vector<std::string > &files){
    struct dirent *ptr;
    DIR *dir;
    std::string PATH = filePath;
    dir=opendir(PATH.c_str());
    while((ptr=readdir(dir))!=NULL)
    {
        if(ptr->d_name[0] == '.')
            continue;
        files.push_back(ptr->d_name);
    }
    closedir(dir);
    return true;
}

int
main(int argc, char** argv)
{
    // Show help
    if (pcl::console::find_switch (argc, argv, "-h") || pcl::console::find_switch (argc, argv, "--help")) {
        showHelp (argv[0]);
        return 0;
    }
    string pcd_path = argv[1];
    string file_name_results = argv[2];
    std::string txt_file_path = pcd_path + "/";
    std::vector<std::string > files;
    // Parameters for sift computation
    const float min_scale = 0.01f;
    const int n_octaves = 3;
    const int n_scales_per_octave = 4;
    const float min_contrast = 0.001f;
    //Write file
    fstream outfile;
    outfile.open(file_name_results, ios::out);
    outfile << "# pcd_file_name\tSIFT points\tcompute_time\n";
    clock_t compute_sift_start_time, compute_sift_end_time;
    if(!GetFiles(txt_file_path, files)){
        std::cerr << "Read file path error\n" << std::endl;
    } else {
        for (int i = 0; i < files.size(); ++i) {
            std::cout << "Reading " << files[i] << std::endl;
            string pcd_file_name = txt_file_path + files[i];
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
            if(pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_file_name, *cloud_xyz) == -1) // load the file
            {
                PCL_ERROR ("Couldn't read pcd file");
            } else {
                outfile << files[i] << "\t";
                std::cout << "\tpoints: " << cloud_xyz->points.size () <<std::endl;
                compute_sift_start_time = clock();
                // Estimate the normals of the cloud_xyz
                pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
                pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
                pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_n(new pcl::search::KdTree<pcl::PointXYZ>());
                ne.setInputCloud(cloud_xyz);
                ne.setSearchMethod(tree_n);
                ne.setRadiusSearch(0.2);
                ne.compute(*cloud_normals);
                // Copy the xyz info from cloud_xyz and add it to cloud_normals as the xyz field in PointNormals estimation is zero
                for(size_t i = 0; i<cloud_normals->points.size(); ++i)
                {
                    cloud_normals->points[i].x = cloud_xyz->points[i].x;
                    cloud_normals->points[i].y = cloud_xyz->points[i].y;
                    cloud_normals->points[i].z = cloud_xyz->points[i].z;
                }
                // Estimate the sift interest points using normals values from xyz as the Intensity variants
                pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointWithScale> sift;
                pcl::PointCloud<pcl::PointWithScale> result;
                pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal> ());
                sift.setSearchMethod(tree);
                sift.setScales(min_scale, n_octaves, n_scales_per_octave);
                sift.setMinimumContrast(min_contrast);
                sift.setInputCloud(cloud_normals);
                sift.compute(result);
                compute_sift_end_time = clock();
                outfile << result.points.size () << "\t" << (double) (compute_sift_end_time - compute_sift_start_time)/CLOCKS_PER_SEC << endl;
                std::cout << "\tSIFT points: " << result.points.size () << std::endl;
                std::cout << "\ttime: " << (double) (compute_sift_end_time - compute_sift_start_time)/CLOCKS_PER_SEC << "s" << std::endl;
            }
        }
    }
    outfile.close();
    return 0;

}
