#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <fstream>
#include <iomanip>
#include <string>
#include <stdlib.h>
#include <vector>
#include <dirent.h>

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
    std::vector<float > point;
    std::vector<float > pointX;
    std::vector<float > pointY;
    std::vector<float > pointZ;
    char buffer[256], *end;
    std::ifstream in;
    in.open(fileName);
    std::cout << fileName << std::endl;
    if (! in.is_open()) {
        std::cerr << "Error opening file" << std::endl;
        return false;
    }
    while (!in.eof() )
    {
        in.getline (buffer,100);
        point.push_back(strtod(buffer, &end));
    }
    for (int i = 0; i < point.size(); ++i) {
        if (i%3 == 0){
            pointX.push_back(point[i]);
        } else if (i%3 == 1){
            pointY.push_back(point[i]);
        } else {
            pointZ.push_back(point[i]);
        }
    }
    int xLength = pointX.size();
    int yLength = pointY.size();
    int zLength = pointZ.size();
    int minLength = xLength;
    if (yLength < minLength){
        minLength = yLength;
    }
    if (zLength < minLength){
        minLength = zLength;
    }
    if (xLength > minLength){
        pointX.pop_back();
    }
    if (yLength > minLength){
        pointY.pop_back();
    }
    if (zLength > minLength){
        pointZ.pop_back();
    }
    points.push_back(pointX);
    points.push_back(pointY);
    points.push_back(pointZ);
    return true;
}

int
main (int argc, char** argv)
{
    std::string root_path = "../data/1/";
    std::string filePath = root_path + "txt/";
    std::vector<std::string > files;
    if(!GetFiles(filePath, files)){
        std::cerr << "Read file path error\n" << std::endl;
    }
    for (int j = 0; j < files.size(); ++j) {
        std::vector<std::vector<float> > points;
        std::string strString = root_path + "txt/" + files[j];
        const char * fileName = strString.c_str();
        if(!GetPoints(fileName, points)){
            std::cerr << "Read file is wrong!\n" << std::endl;
        }
        pcl::PointCloud<pcl::PointXYZ> cloud;
        cloud.width = points[0].size();
        cloud.height = 1;
        cloud.is_dense = false;
        cloud.points.resize (cloud.width * cloud.height);
        for (size_t i = 0; i < cloud.points.size (); ++i)
        {
            cloud.points[i].x = points[0][i];
            cloud.points[i].y = points[1][i];
            cloud.points[i].z = points[2][i];
        }
        std::string pcdFIleName = root_path + "pcd/" + files[j].substr(0, files[j].length()-4) + ".pcd";
        pcl::io::savePCDFileASCII (pcdFIleName, cloud);
        std::cout << "Saved " << cloud.points.size () << " data points to " << pcdFIleName << std::endl;
    }

    return (0);
}