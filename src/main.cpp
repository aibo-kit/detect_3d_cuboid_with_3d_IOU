// std c
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>

// opencv
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

// ros
// #include <ros/ros.h>
// #include <ros/package.h>

//boost

#include "boost/geometry.hpp"

// Eigen
#include <Eigen/Dense>
#include <Eigen/Core>

// ours
#include "detect_3d_cuboid/detect_3d_cuboid.h"
#include "detect_3d_cuboid/object_3d_util.h"
#include "detect_3d_cuboid/matrix_utils.h"
// #include "tictoc_profiler/profiler.hpp"

using namespace std;
using namespace Eigen;

int main(int argc, char **argv)
{
    // ros::init(argc, argv, "detect_3d_cuboid");
    // ros::NodeHandle nh;
    // ca::Profiler::enable();
    string path_to_dataset = argv[1];
    string base_folder = path_to_dataset + "/data/";

    Matrix3d Kalib;
    Kalib << 535.4,  0,  320.1,   // for TUM cabinet data.
            0,  539.2, 247.6,
            0,      0,     1;

    Eigen::MatrixXd truth_frame_poses(100,8);
    read_all_number_txt(base_folder + "/online_camera_pose/" "truth_cam_poses.txt", truth_frame_poses);
    Eigen::MatrixXd truth_cube_poses(100,10);
    read_all_number_txt(base_folder + "/ground_truth/" "online_cubes.txt", truth_cube_poses);
//    Matrix4d transToWolrd;
//    transToWolrd << 1, 0.0011, 0.0004, 0, // hard coded  NOTE if accurate camera roll/pitch, could sample it!
//        0, -0.3376, 0.9413, 0,
//        0.0011, -0.9413, -0.3376, 1.35,
//        0, 0, 0, 1;

    const int amount_of_img = 58;
    //obj_bbox_coors << 201, 40, 341, 386, 0.32; // [x y w h prob] orginal data:188, 189, 201, 311, 0.8800
    //obj_bbox_coors.leftCols<2>().array() -= 1;    // change matlab coordinate to c++, minus 1
    ObjectSet detected_cuboids;
    detected_cuboids.reserve(60);
    std::vector<int> cube_index;


    for(int frame_index = 0; frame_index < amount_of_img; frame_index++){

        char frame_index_c[256];
        sprintf(frame_index_c, "%04d", frame_index); // format into 4 digit

        MatrixXd obj_bbox_coors(1, 5);
        read_all_number_txt(base_folder + "/filter_2d_obj_txts/" + frame_index_c + "_yolo2_0.15.txt", obj_bbox_coors);
        obj_bbox_coors.leftCols<2>().array() -= 1;    // change matlab coordinate to c++, minus 1

        if(obj_bbox_coors.size() == 0)
            continue;

        Eigen::MatrixXd cam_pose_Twc = truth_frame_poses.row(frame_index).tail<7>(); // xyz, q1234   第frame_index行，后7个数据
        // Eigen::MatrixXd cam_pose_Twc = truth_frame_poses.row(0).tail<7>(); // xyz, q1234
        Matrix<double,4,4> transToWolrd;
        transToWolrd.setIdentity();
        Eigen::Vector3d eulerAngle4;
        quat_to_euler_zyx(Quaterniond(cam_pose_Twc(6),cam_pose_Twc(3),cam_pose_Twc(4),cam_pose_Twc(5)), eulerAngle4[0], eulerAngle4[1], eulerAngle4[2]);
        std::cout<<"roll(x), pitch(y), yaw(z) = "<<eulerAngle4.transpose()<<std::endl;
        //transform quaternion into T

        transToWolrd.block(0,0,3,3) = Quaterniond(cam_pose_Twc(6),cam_pose_Twc(3),cam_pose_Twc(4),cam_pose_Twc(5)).toRotationMatrix();
        transToWolrd.col(3).head(3) = Eigen::Vector3d(cam_pose_Twc(0), cam_pose_Twc(1), cam_pose_Twc(2));
        std::cout<<" orignal T= \n"<<transToWolrd<<std::endl;
        // read images
        cv::Mat rgb_img = cv::imread(base_folder + "/raw_imgs/" + frame_index_c + "_rgb_raw.jpg", 1);

        // read edges
        Eigen::MatrixXd all_lines_raw(110, 4); // 100 is some large frame number,   the txt edge index start from 0
        read_all_number_txt(base_folder + "/online_edge_detection/" + frame_index_c + "_online_edges.txt", all_lines_raw);
        // read_all_number_txt(base_folder + frame_index_c + "_rgb_raw.txt", all_lines_raw);
        // read cube ground truth
        Eigen::MatrixXd cub_pose_Twc = truth_cube_poses.row(frame_index).tail<9>();
        std::cout<<" the pose of cube = "<< cub_pose_Twc(0,0)<<std::endl;


        detect_3d_cuboid detect_cuboid_obj;
        detect_cuboid_obj.whether_plot_detail_images = false;
        detect_cuboid_obj.whether_plot_final_images = true;
        detect_cuboid_obj.print_details = false; // false  true
        detect_cuboid_obj.set_calibration(Kalib);
        detect_cuboid_obj.whether_sample_bbox_height = false;
        detect_cuboid_obj.whether_sample_cam_roll_pitch = false;
        detect_cuboid_obj.whether_save_final_cuboids_txt = true;

        std::vector<ObjectSet> all_object_cuboids;
        std::vector<ObjectSet> object_cuboid_after_3d_iou;
        //std::vector<ObjectSet> result_cuboid_of_all_images;
        cout<< "\n" << "this is the No."<< frame_index <<" Cuboid "<<endl;
        detect_cuboid_obj.detect_cuboid(rgb_img, transToWolrd, obj_bbox_coors, all_lines_raw, cub_pose_Twc, all_object_cuboids, object_cuboid_after_3d_iou);

        detected_cuboids.push_back(object_cuboid_after_3d_iou[0][0]);
        cube_index.push_back(frame_index);
        //cout<<"cube pos rotY = " <<detected_cuboids[frame_index]->rotY<<endl;


      }
        cout<<"size of the total cuboids = "<<detected_cuboids.size()<<endl;
        std::string save_final_cuboids_txt = base_folder +  "all_3d_cuboids_after_3d_IoU.txt";
        std::ofstream save_final_cuboids_stream;
        save_final_cuboids_stream.open(save_final_cuboids_txt.c_str());
        for (size_t i = 0; i < detected_cuboids.size(); i++)
            {//for (size_t j = 0; j < all_object_cuboids[i].size(); j++)

    //            save_final_cuboids_stream << all_object_cuboids[i][j]->pos.transpose() << " "
    //                                        << all_object_cuboids[i][j]->rotY <<  " "
    //                                        << all_object_cuboids[i][j]->scale.transpose() << " "
    //                                        << i*all_object_cuboids[i].size()+j
    //                                        << "\n";
                        save_final_cuboids_stream << cube_index[i]<<" "
                                                    << detected_cuboids[i]->pos.transpose() << " "
                                                    << detected_cuboids[i]->rotY <<  " "
                                                    << detected_cuboids[i]->scale.transpose() << " "
                                                    << 0
                                                    << "\n";

            }
        save_final_cuboids_stream.close();
    //ca::Profiler::print_aggregated(std::cout);

    return 0;
}
