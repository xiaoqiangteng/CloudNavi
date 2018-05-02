//
// Created by teng on 18-5-1.
//
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>

int user_data;

void viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (1.0, 0.5, 1.0);
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape ("text", 0);
    viewer.addText (ss.str(), 200, 300, "text", 0);
    //FIXME: possible race condition here:
    user_data++;
}

int main (int argc, char** argv)
{
    std::string cloudName = argv[1];
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile<pcl::PointXYZ> (cloudName, *cloud) == -1) // load the file
    {
        PCL_ERROR ("Couldn't read file");
        return -1;
    }
    std::cout << "points of the first cloud: " << cloud->points.size () <<std::endl;
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    //blocks until the cloud is actually rendered
    viewer.showCloud(cloud);
    //This will get called once per visualization iteration
    viewer.runOnVisualizationThread (viewerPsycho);
    while (!viewer.wasStopped ())
    {
        //you can also do cool processing here
        //FIXME: Note that this is running in a separate thread from viewerPsycho
        //and you should guard against race conditions yourself...
        user_data++;
    }
    return 0;
}