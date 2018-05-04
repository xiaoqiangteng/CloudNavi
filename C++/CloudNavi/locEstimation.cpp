//
// Created by teng on 18-5-1.
//

#include <iostream>
#include <fstream>
#include <time.h>
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
    std::cout << "Usage: " << program_name << " file_name[gt_file] file_name[est_file] path[gt_cloud] path[est_cloud] file_name[results] file_name[result_all]" << std::endl;
    std::cout << "-h:  Show this help." << std::endl;
}

bool GetData(const char* fileName, vector<string >& gt_cloud_name_vec, vector<vector<string> > &gt_loc_vec){
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
    for (int i = 0; i < lines_string.size(); ++i) {
        gt_cloud_name_vec.push_back(lines_string[i][0]);
        vector<string > loc_from_string;
        loc_from_string.push_back(lines_string[i][1]);
        loc_from_string.push_back(lines_string[i][2]);
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
    const char *results = argv[5];
    const char *result_all = argv[6];
    //Write file
    fstream outfile, outfile_all;
    outfile.open(results, ios::out);
    outfile_all.open(result_all, ios::out);
    outfile << "# est_cloud\tmost_gt_cloud\tcorrespondence_num\test_loc[0]\test_loc[1]\tgt_loc[0]\tgt_loc[1]\tcompute_time" << endl;
    outfile_all << "# est_cloud\tgt_cloud\tcorrespondence_num\test_loc[0]\test_loc[1]\tgt_loc[0]\tgt_loc[1]\tload_time\tcompute_time" << endl;
    //Read the cloud name and the loc data of the gt and the est
    vector<string > gt_cloud_name_vec;
    vector<vector<string> > gt_loc_vec;
    const char * gt_file_name = gt_file.c_str();
    if(!GetData(gt_file_name, gt_cloud_name_vec, gt_loc_vec)){
        std::cerr << "Read " << gt_file << " error\n" << std::endl;
    }
    vector<string > est_cloud_name_vec;
    vector<vector<string> > est_loc_vec;
    const char * est_file_name = est_file.c_str();
    if(!GetData(est_file_name, est_cloud_name_vec, est_loc_vec)){
        std::cerr << "Read " << est_file << " error\n" << std::endl;
    }
    for (int i = 0; i < gt_cloud_name_vec.size(); ++i) {
        cout << "gt_cloud_name_vec " << gt_cloud_name_vec[i] << " loc: " << gt_loc_vec[i][0] << " " << gt_loc_vec[i][1] << endl;
    }
    for (int i = 0; i < est_cloud_name_vec.size(); ++i) {
        cout << "est_cloud_name_vec " << est_cloud_name_vec[i] << " loc: " << est_loc_vec[i][0] << " " << est_loc_vec[i][1] << endl;
    }

    int total = int (est_cloud_name_vec.size()*gt_cloud_name_vec.size());
    int count = 0;
    clock_t load_start_time_all, load_end_time_all, compute_start_time, compute_end_time, compute_start_time_all, compute_end_time_all;
    //Load point cloud data of the gt and the est
    for (int i = 0; i < est_cloud_name_vec.size(); ++i) {
        cout << "---------------------------------------------------------" << endl;
        outfile << est_cloud_name_vec[i] << "\t";
        string est_locX_string = est_loc_vec[i][0];
        string est_locY_string = est_loc_vec[i][1];
        string est_cloud_file = est_cloud_path + est_cloud_name_vec[i];
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
        if(pcl::io::loadPCDFile<pcl::PointXYZ> (est_cloud_file, *cloud_in) == -1)
        {
            PCL_ERROR ("Couldn't read file [est_cloud_name_vec]");
            return -1;
        }
        int most_similar_num = -1;
        string most_similar_cloud;
        string most_similar_locX;
        string most_similar_locY;
        compute_start_time = clock();
        for (int j = 0; j < gt_cloud_name_vec.size(); ++j) {
            outfile_all << est_cloud_name_vec[i] << "\t";
            outfile_all << gt_cloud_name_vec[j] << "\t";
            string gt_cloud_file = gt_cloud_path + gt_cloud_name_vec[j];
            cout << "[" << count*100/total << "%]" << "\tReading\t\t" << est_cloud_name_vec[i] << "\t" << gt_cloud_name_vec[j] << endl;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
            load_start_time_all = clock();
            if(pcl::io::loadPCDFile<pcl::PointXYZ> (gt_cloud_file, *cloud_out) == -1)
            {
                PCL_ERROR ("Couldn't read file [gt_cloud_name_vec]");
                return -1;
            }
            load_end_time_all = clock();
            std::cout << "\t\tpoints: " << cloud_in->points.size () << "\t" << cloud_out->points.size () <<std::endl;
            compute_start_time_all = clock();
            boost::shared_ptr<pcl::Correspondences> correspondences (new pcl::Correspondences);
            pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> corr_est;
            corr_est.setInputCloud (cloud_in);
            corr_est.setInputTarget (cloud_out);
            corr_est.determineCorrespondences (*correspondences,.1);
            std::cout << "\t\tCorrespondence: " << int (correspondences->size ()) << std::endl;
            compute_end_time_all = clock();
//            boost::shared_ptr<pcl::Correspondences> correspondences_remain (new pcl::Correspondences);
//            pcl::registration::CorrespondenceRejectorMedianDistance rej;
//            rej.setMedianFactor (8.79241104);
//            rej.setInputCorrespondences (correspondences);
//            rej.getCorrespondences (*correspondences_remain);
//            std::cout << "\tNo. of correspondence remain is " << int (correspondences_remain->size ()) << std::endl;
            if (int (correspondences->size ()) > most_similar_num){
                most_similar_num = int (correspondences->size ());
                most_similar_cloud = gt_cloud_name_vec[j];
                most_similar_locX = gt_loc_vec[j][0];
                most_similar_locY = gt_loc_vec[j][1];
            }
            std::cout << "\t\tthe most similar cloud is " << most_similar_cloud << std::endl;
            std::cout << "\t\tcorrespondences number is " << most_similar_num << std::endl;
            count ++;
            outfile_all << int (correspondences->size ()) << "\t" << est_locX_string << "\t"
                    << est_locY_string << "\t" << gt_loc_vec[j][0] << "\t" << gt_loc_vec[j][1] << "\t"
                    << (double) (load_end_time_all - load_start_time_all)/CLOCKS_PER_SEC << "\t"
                    << (double) (compute_end_time_all - compute_start_time_all)/CLOCKS_PER_SEC << endl;
        }
        compute_end_time = clock();
        outfile << most_similar_cloud << "\t" << most_similar_num << "\t" << est_locX_string << "\t"
                << est_locY_string << "\t" << most_similar_locX << "\t" << most_similar_locY << "\t"
                << (double) (compute_end_time - compute_start_time)/CLOCKS_PER_SEC << endl;
    }
    outfile.close();
    outfile_all.close();

    return (0);
}