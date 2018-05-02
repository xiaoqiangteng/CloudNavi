//
// Created by teng on 18-5-1.
//

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
//#include <pcl/ros/conversions.h>//formROSMsg所属头文件；
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_lib_io.h>
//#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using namespace pcl;
int main()
{
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFileOBJ("1.obj", mesh);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
    pcl::io::savePCDFileASCII("1-1.pcd", *cloud);

    cout << cloud->size() << endl;
    return 0;
}