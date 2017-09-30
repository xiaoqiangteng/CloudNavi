#include <iostream>
#include <vector>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/normal_3d.h>

using namespace std;

int main (int argc, char** argv){
    //Read all files
    struct dirent *ptr;
    DIR *dir;
    string PATH = "../data/file";
    dir = opendir(PATH.c_str());
    vector<string> files;
    while((ptr=readdir(dir))!=NULL){
        //跳过'.'和'..'两个目录
        if(ptr->d_name[0] == '.')
            continue;
        cout << ptr->d_name << endl;
        files.push_back(ptr->d_name);
    }
    for (int i = 0; i < files.size(); ++i){
        cout << files[i] << endl;
    }
    closedir(dir);

    //Read a file
    ifstream out;
    string str = "../data/depth_3609.839398001.depth";
    out.open(str.c_str(), ios::in);
    string line;
    vector<string> vecXYZ;
    while(!out.eof()){
        std::getline(out,line,'\n');
        if (line == ""){
        } else {
            vecXYZ.push_back(line);
        }
    }
    out.close();

    //Write to PCD file
    vector<string> vecX;
    vector<string> vecY;
    vector<string> vecZ;
    for (int i = 0; i < vecXYZ.size(); ++i) {
        if (i%3 == 0){
            vecX.push_back(vecXYZ[i]);
        } else if (i%3 == 1){
            vecY.push_back(vecXYZ[i]);
        } else if (i%3 == 2){
            vecZ.push_back(vecXYZ[i]);
        }
    }
    pcl::PointCloud<pcl::PointXYZ> cloud;
    // Fill in the cloud data
    cloud.width    = vecX.size();
    cloud.height   = 1;
    cloud.is_dense = false;
    cloud.points.resize (cloud.width * cloud.height);
    for (int i = 0; i < vecX.size(); ++i) {
        cloud.points[i].x = (float) atof(vecX[i].c_str());
        cloud.points[i].y = (float) atof(vecY[i].c_str());
        cloud.points[i].z = (float) atof(vecZ[i].c_str());
    }
    pcl::io::savePCDFileASCII ("depth_3609.pcd", cloud);
    std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;

    //Dectect SIFT keypoints
    std::string filename = "depth_3609.pcd";
    std::cout << "Reading " << filename << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
    // load the file
    if(pcl::io::loadPCDFile<pcl::PointXYZ> (filename, *cloud_xyz) == -1){
        PCL_ERROR ("Couldn't read file");
        return -1;
    }
    std::cout << "points: " << cloud_xyz->points.size () <<std::endl;
    // Parameters for sift computation
    const float min_scale = 0.01f;
    const int n_octaves = 3;
    const int n_scales_per_octave = 4;
    const float min_contrast = 0.001f;
    // Estimate the normals of the cloud_xyz
    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_n(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setInputCloud(cloud_xyz);
    ne.setSearchMethod(tree_n);
    ne.setRadiusSearch(0.2);
    ne.compute(*cloud_normals);
    // Copy the xyz info from cloud_xyz and add it to cloud_normals as the xyz field in PointNormals estimation is zero
    for(size_t i = 0; i<cloud_normals->points.size(); ++i){
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
    std::cout << "No of SIFT points in the result are " << result.points.size () << std::endl;

    return (0);
}
