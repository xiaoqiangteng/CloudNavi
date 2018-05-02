cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cloudnavi)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(GeneratePCD GeneratePCD.cpp)
target_link_libraries (GeneratePCD ${PCL_LIBRARIES})

add_executable(matrix_transform matrix_transform.cpp)
target_link_libraries (matrix_transform ${PCL_LIBRARIES})

add_executable(narf_keypoint_extraction narf_keypoint_extraction.cpp)
target_link_libraries (narf_keypoint_extraction ${PCL_LIBRARIES})

add_executable(sift_normal_keypoint_estimation sift_normal_keypoint_estimation.cpp)
target_link_libraries (sift_normal_keypoint_estimation ${PCL_LIBRARIES})

add_executable(icp icp.cpp)
target_link_libraries (icp ${PCL_LIBRARIES})

add_executable(ply2pcd ply2pcd.cpp)
target_link_libraries (ply2pcd ${PCL_LIBRARIES})

add_executable(ply2pcdColor ply2pcdColor.cpp)
target_link_libraries (ply2pcdColor ${PCL_LIBRARIES})

add_executable(visulization visulization.cpp)
target_link_libraries (visulization ${PCL_LIBRARIES})

add_executable(obj2pcd obj2pcd.cpp)
target_link_libraries (obj2pcd ${PCL_LIBRARIES})

add_executable(CorrespondenceEstimation CorrespondenceEstimation.cpp)
target_link_libraries (CorrespondenceEstimation ${PCL_LIBRARIES})