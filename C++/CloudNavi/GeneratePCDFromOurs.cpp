#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <fstream>
#include <iomanip>
#include <string>
#include <stdlib.h>
#include <vector>
#include <dirent.h>

using namespace std;

// This function displays the help
void showHelp(char * program_name)
{
    std::cout << std::endl;
    std::cout << "Usage: " << program_name << " path[TXT] path[PCD] " << std::endl;
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

bool GetPoints(const char* fileName, std::vector<std::vector<float> > &points){
    //Store the data in a line as string
    string line_str;
    string each_str;
    //Store the file name and the location data as string
    vector<string> line_string;
    vector<vector<string> > lines_string;
    std::ifstream in;
    in.open(fileName);
    std::cout << "Read data from " << fileName << std::endl;
    if (! in.is_open()) {
        std::cerr << "Error opening file" << std::endl;
        return false;
    }
    if (in.eof()){
        cerr << "File is NULL!" << endl;
        return false;
    }
    while(!in.eof())
    {
        //Read one row of data at a time
        getline(in, line_str);
        if (line_str.empty()){
            continue;
        }
        stringstream stringin(line_str);
        while (stringin >> each_str)
        {
            line_string.push_back(each_str);
        }
        lines_string.push_back(line_string);
        line_string.clear();
    }
    in.close();
    if (lines_string.size() < 1){
        cerr << "File is NULL!" << endl;
        return false;
    } else {
        vector<float > pointsStr;
        for (int i = 0; i < lines_string.size(); ++i) {
            pointsStr.push_back(stof(lines_string[i][0]));
            pointsStr.push_back(stof(lines_string[i][1]));
            pointsStr.push_back(stof(lines_string[i][2]));
            points.push_back(pointsStr);
            pointsStr.clear();
        }
    }
    return true;
}

int
main (int argc, char** argv)
{
    // Show help
    if (pcl::console::find_switch (argc, argv, "-h") || pcl::console::find_switch (argc, argv, "--help")) {
        showHelp (argv[0]);
        return 0;
    }
    string txt_path = argv[1];
    string pcd_path = argv[2];
    std::string txt_file_path = txt_path + "/";
    std::string pcd_file_path = pcd_path + "/";
    std::vector<std::string > files;
    if(!GetFiles(txt_file_path, files)){
        std::cerr << "Read file path error\n" << std::endl;
    } else {
        for (int j = 0; j < files.size(); ++j) {
            std::vector<std::vector<float> > points;
            std::string strString = txt_file_path + files[j];
            const char * fileName = strString.c_str();
            if(!GetPoints(fileName, points)){
                std::cerr << "Read file is wrong!\n" << std::endl;
            } else {
                pcl::PointCloud<pcl::PointXYZ> cloud;
                cloud.width = points.size();
                cloud.height = 1;
                cloud.is_dense = false;
                cloud.points.resize (cloud.width * cloud.height);
                for (size_t i = 0; i < cloud.points.size (); ++i)
                {
                    cloud.points[i].x = points[i][0];
                    cloud.points[i].y = points[i][1];
                    cloud.points[i].z = points[i][2];
                }
                std::string pcdFIleName = pcd_file_path + files[j].substr(0, files[j].length()-4) + ".pcd";
                pcl::io::savePCDFileASCII (pcdFIleName, cloud);
                std::cout << "Saved " << cloud.points.size () << " data points to " << pcdFIleName << std::endl;
            }
        }
    }
    return (0);
}