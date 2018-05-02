//
// Created by teng on 18-5-1.
//

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

// This function displays the help
void
showHelp(char * program_name)
{
    std::cout << std::endl;
    std::cout << "Usage: " << program_name << " ply-folder-path pcd-folder-path" << std::endl;
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

// This is the main function
int main(int argc, char** argv)
{
    // Show help
    if (pcl::console::find_switch (argc, argv, "-h") || pcl::console::find_switch (argc, argv, "--help")) {
        showHelp (argv[0]);
        return 0;
    }

    std::string ply_file_path = argv[1];
    std::string pcd_file_path = argv[2];
    std::vector<std::string > files;
    if(!GetFiles(ply_file_path, files)){
        std::cerr << "Read file path error\n" << std::endl;
    }
    pcl::PLYReader reader;
    for (int j = 0; j < files.size(); ++j) {
        pcl::PCLPointCloud2 clod;
        std::string ply_file_string = ply_file_path + "/" + files[j];
        reader.read(ply_file_string, clod);
        pcl::PCDWriter writer;
        std::string pcd_file_string = pcd_file_path + "/" + files[j].substr(0, files[j].length()-4) + ".pcd";
        writer.writeASCII(pcd_file_string, clod);
    }

    return 0;

}