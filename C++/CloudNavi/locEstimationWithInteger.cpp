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
#include <pcl/console/parse.h>

using namespace std;

// This function displays the help
void showHelp(char * program_name)
{
    std::cout << std::endl;
    std::cout << "Usage: " << program_name << " file_name[gt_file] path[est_file] path[gt_cloud] path[est_cloud] file_name[results]" << std::endl;
    std::cout << "-h:  Show this help." << std::endl;
}

bool GetData(const char* fileName, vector<string >& gt_cloud_name_vec, vector<vector<int> > &gt_loc_vec){
    //Store the data in a line as string
    string line_str;
    string each_str;
    //Store the file name and the location data as string
    vector<string> line_string;
    vector<vector<string> > lines_string;
    char buffer[256], *end;
    std::ifstream in;
    in.open(fileName);
    std::cout << "Read data from " << fileName << std::endl;
    if (! in.is_open()) {
        std::cerr << "Error opening file" << std::endl;
        return false;
    }
    while(!in.eof())
    {
        //Read one row of data at a time
        getline(in, line_str);
        stringstream stringin(line_str);
        while (stringin >> each_str)
        {
            line_string.push_back(each_str);
        }
        lines_string.push_back(line_string);
    }
    in.close();
    for (int i = 0; i < lines_string.size(); ++i) {
        gt_cloud_name_vec.push_back(lines_string[i][0]);
        vector<int > loc_from_string;
        loc_from_string.push_back(atoi(lines_string[i][1].c_str()));
        loc_from_string.push_back(atoi(lines_string[i][2].c_str()));
        gt_loc_vec.push_back(loc_from_string);
        loc_from_string.clear();
    }
    return true;
}

int main (int argc, char** argv)
{
    // Show help
    if (pcl::console::find_switch (argc, argv, "-h") || pcl::console::find_switch (argc, argv, "--help")) {
        showHelp (argv[0]);
        return 0;
    }

    string gt_file = argv[1];
    string est_file = argv[2];
    string gt_cloud_path = argv[3];
    string est_cloud_path = argv[4];
    const char *results = argv[4];
    //Write file
    ofstream outfile;
    outfile.open(results);
    outfile << "# est_cloud\tmost_gt_cloud\tcorrespondence_num\test_loc[0]\test_loc[1]\tgt_loc[0]\tgt_loc[1]" << endl;
    //Read the cloud name and the loc data of the gt and the est
    vector<string > gt_cloud_name_vec;
    vector<vector<int> > gt_loc_vec;
    const char * gt_file_name = gt_file.c_str();
    if(!GetData(gt_file_name, gt_cloud_name_vec, gt_loc_vec)){
        std::cerr << "Read " << gt_file << " error\n" << std::endl;
    }
    vector<string > est_cloud_name_vec;
    vector<vector<int> > est_loc_vec;
    const char * est_file_name = est_file.c_str();
    if(!GetData(est_file_name, est_cloud_name_vec, est_loc_vec)){
        std::cerr << "Read " << est_file << " error\n" << std::endl;
    }

    //Load point cloud data of the gt and the est
    for (int i = 0; i < est_cloud_name_vec.size(); ++i) {
        outfile << est_cloud_name_vec[i] << "\t";
        string est_cloud_file = est_cloud_path + est_cloud_name_vec[i];
        cout << "Reading " << est_cloud_name_vec[i] << " data [est_cloud_name_vec]" << endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
        int most_similar_num = -1;
        string most_similar_cloud;
        vector<int > most_similar_loc;

        for (int j = 0; j < gt_cloud_name_vec.size(); ++j) {
            string gt_cloud_file = gt_cloud_path + gt_cloud_name_vec[j];
            cout << "Reading " << gt_cloud_name_vec[i] << " data [gt_cloud_name_vec]" << endl;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
            if((pcl::io::loadPCDFile<pcl::PointXYZ> (est_cloud_file, *cloud_in) == -1) ||
                    (pcl::io::loadPCDFile<pcl::PointXYZ> (gt_cloud_file, *cloud_out) == -1))
            {
                PCL_ERROR ("Couldn't read file");
                return -1;
            }
            std::cout << "points of the est cloud: " << cloud_in->points.size () <<std::endl;
            std::cout << "points of the gt cloud: " << cloud_out->points.size () <<std::endl;
            boost::shared_ptr<pcl::Correspondences> correspondences (new pcl::Correspondences);
            pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> corr_est;
            corr_est.setInputCloud (cloud_in);
            corr_est.setInputTarget (cloud_out);
            corr_est.determineCorrespondences (*correspondences,.1);
            boost::shared_ptr<pcl::Correspondences> correspondences_remain (new pcl::Correspondences);
            pcl::registration::CorrespondenceRejectorMedianDistance rej;
            rej.setMedianFactor (8.79241104);
            rej.setInputCorrespondences (correspondences);
            rej.getCorrespondences (*correspondences_remain);
            std::cout << "No. of correspondence is " << int (correspondences_remain->size ()) << std::endl;
            if (int (correspondences_remain->size ()) > most_similar_num){
                most_similar_loc.clear();
                most_similar_num = int (correspondences_remain->size ());
                most_similar_cloud = gt_cloud_name_vec[j];
                most_similar_loc.push_back(gt_loc_vec[j][0]);
                most_similar_loc.push_back(gt_loc_vec[j][1]);
            }
            std::cout << "\tthe most similar cloud is " << most_similar_cloud << std::endl;
            std::cout << "\tcorrespondences number is " << most_similar_num << std::endl;
        }

    }
    outfile.close();



    return (0);
}