#ifndef FUSION_H
#define FUSION_H

#include <sstream>
#include <iostream>
#include <cmath> 
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <numeric>
#include <pcl/filters/voxel_grid.h>
#include "projector_lidar.hpp"
#include "Lidar_parser_base.h"

class fusion {
    private:

    ros::NodeHandle nh;

    ros::Publisher lidar_pub;//融合后点云可视化结果
    ros::Publisher cam_pub;//融合后图像可视化话题

    ros::Subscriber lidar_sub;//接收rslidar点云数据，进入回调函数getcloud())
    ros::Subscriber cam_sub;//相机话题


    std::string lidar_sub_topic;//接收激光雷达话题
    std::string lidar_pub_topic;//发布融合后点云
    std::string cam_sub_topic;//接收图像话题
    std::string cam_pub_topic; //发布融合后图像话题

    void lidar_callback(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg);
    void cam_callback(const sensor_msgs::ImageConstPtr& Imgmsg);
    void loadCalibrationData(void);
    cv::Mat img;
    sensor_msgs::ImagePtr image_msg;
    sensor_msgs::PointCloud2 fusion_msg;  //等待发送的点云消息

    cv::Mat RT; // rotation matrix and translation vector 旋转矩阵和平移向量AUTOWARE格式，这个格式对应一般的外参标定结果（旋转平移矩阵）需要参考https://blog.csdn.net/qq_22059843/article/details/103022451
    cv::Mat D; 
    cv::Mat K; 
    std::vector<double> camera_matrix;
    std::vector<double> calib;
    std::vector<double> distort;

    // 使用 Projector 类进行投影
    Projector projector;

    public:

    fusion();
    ~fusion(){};
    void run();

};
# endif