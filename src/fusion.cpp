#include "lidar_camera_fusion/fusion.h"

using namespace std;

fusion::fusion(){
    nh.param<std::string>("sensor_fusion/lidar/lidar_input_topic",fusion::lidar_sub_topic,"/rslidar_points");//接收激光雷达话题
    nh.param<std::string>("sensor_fusion/lidar/lidar_output_topic",fusion::lidar_pub_topic,"/fusion/rslidar_points");//发布融合后点云
    nh.param<std::string>("sensor_fusion/cam/cam_input_topic",fusion::cam_sub_topic,"/camera/image");//接收图像话题
    nh.param<std::string>("sensor_fusion/cam/cam_output_topic",fusion::cam_pub_topic,"/fusion/image");//发布融合后图像话题

    nh.param<vector<double>>("camera/camera_matrix", fusion::camera_matrix,vector<double>());
    nh.param<vector<double>>("camera/RT", fusion::calib,vector<double>());
    nh.param<vector<double>>("camera/distort", fusion::distort,vector<double>());

    RT = cv::Mat(4,4,cv::DataType<double>::type); //  旋转矩阵和平移向量
    D = cv::Mat(5,1,cv::DataType<double>::type); 
    K = cv::Mat(3,3,cv::DataType<double>::type);

    fusion::loadCalibrationData();
}

void fusion::loadCalibrationData(void )
{
    // 加载 RT 矩阵
    RT.at<double>(0, 0) = calib[0]; RT.at<double>(0, 1) = calib[1]; RT.at<double>(0, 2) = calib[2]; RT.at<double>(0, 3) = calib[3];
    RT.at<double>(1, 0) = calib[4]; RT.at<double>(1, 1) = calib[5]; RT.at<double>(1, 2) = calib[6]; RT.at<double>(1, 3) = calib[7];
    RT.at<double>(2, 0) = calib[8]; RT.at<double>(2, 1) = calib[9]; RT.at<double>(2, 2) = calib[10]; RT.at<double>(2, 3) = calib[11];
    RT.at<double>(3, 0) = calib[12]; RT.at<double>(3, 1) = calib[13]; RT.at<double>(3, 2) = calib[14]; RT.at<double>(3, 3) = calib[15];

    // 加载相机内参矩阵
    K.at<double>(0, 0) = camera_matrix[0]; K.at<double>(0, 1) = camera_matrix[1]; K.at<double>(0, 2) = camera_matrix[2];
    K.at<double>(1, 0) = camera_matrix[3]; K.at<double>(1, 1) = camera_matrix[4]; K.at<double>(1, 2) = camera_matrix[5];
    K.at<double>(2, 0) = camera_matrix[6]; K.at<double>(2, 1) = camera_matrix[7]; K.at<double>(2, 2) = camera_matrix[8];

    // 加载畸变系数
    D.at<double>(0, 0) = distort[0]; D.at<double>(1, 0) = distort[1]; D.at<double>(2, 0) = distort[2];
    D.at<double>(3, 0) = distort[3]; D.at<double>(4, 0) = distort[4];
    // **将矩阵类型转换为 CV_32F**
    RT.convertTo(RT, CV_32F);
    K.convertTo(K, CV_32F);
    D.convertTo(D, CV_32F);
}

void fusion::run(){
    cam_sub = nh.subscribe<sensor_msgs::Image>(cam_sub_topic,1, &fusion::cam_callback,this);//相机话题
    lidar_sub = nh.subscribe<sensor_msgs::PointCloud2>(lidar_sub_topic,1, &fusion::lidar_callback, this); //接收rslidar点云数据，进入回调函数getcloud())
    lidar_pub = nh.advertise<sensor_msgs::PointCloud2>(lidar_pub_topic,1);//融合后点云可视化结果
    cam_pub = nh.advertise<sensor_msgs::Image>(cam_pub_topic,1);//融合后图像可视化话题
}

void fusion::lidar_callback(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

  if (!img.empty()) {
      // 点云下采样
      pcl::PointCloud<pcl::PointXYZI>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::fromROSMsg(*laserCloudMsg, *current_pc_ptr);

      projector.loadPointCloud(*current_pc_ptr); // 加载点云
      projector.setPointSize(3); // 设置点大小
      projector.setDisplayMode(false); // 使用距离作为颜色
      projector.setFilterMode(true); // 启用重叠点过滤

      // 投影点云到图像
      auto projected_result = projector.ProjectToRawMat(img, K, D, RT(cv::Rect(0, 0, 3, 3)), RT(cv::Rect(3, 0, 1, 3)));

      // 发布融合后的图像
      image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", projected_result.second).toImageMsg();
      cam_pub.publish(image_msg);

      // 发布融合后的点云
      projected_result.first->width = 1;
      projected_result.first->height = projected_result.first->points.size();
      pcl::toROSMsg(*projected_result.first, fusion_msg);
      fusion_msg.header = laserCloudMsg->header;
      lidar_pub.publish(fusion_msg);
  } else {
      std::cout << "Waiting for image topic" << std::endl;
  }
}

void fusion::cam_callback(const sensor_msgs::ImageConstPtr& Imgmsg){
  cv_bridge::CvImagePtr cv_ptr;
  try {
      cv_ptr = cv_bridge::toCvCopy(Imgmsg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }
  img  = cv_ptr->image; 
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_cam_fusion");//ROS节点初始化
  fusion fs;
  fs.run();
  ros::spin();
  // cv::destroyWindow("view");
}
