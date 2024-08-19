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

    nh.param<int>("camera/camera_width", fusion::imagewidth, 1920);
    nh.param<int>("camera/camera_height", fusion::imageheight, 1080);

    is_get_image = 0;
    alpha = 0;

    P_rect_00 = cv::Mat(3,4,cv::DataType<double>::type); // 矫正后的3*4的投影矩阵
    RT = cv::Mat(4,4,cv::DataType<double>::type); //  旋转矩阵和平移向量
    D = cv::Mat(5,1,cv::DataType<double>::type); 
    K = cv::Mat(3,3,cv::DataType<double>::type);

    image_color.resize(imagewidth * imageheight); // 动态分配内存

    fusion::loadCalibrationData();
}

void fusion::loadCalibrationData(void )
{
  RT.at<double>(0,0) = calib[0]; RT.at<double>(0,1) =  calib[1]; RT.at<double>(0,2) = calib[2]; RT.at<double>(0,3) =  calib[3];
  RT.at<double>(1,0) =  calib[4]; RT.at<double>(1,1) = calib[5]; RT.at<double>(1,2) =  calib[6]; RT.at<double>(1,3) = calib[7];
  RT.at<double>(2,0) =  calib[8]; RT.at<double>(2,1) =  calib[9]; RT.at<double>(2,2) =  calib[10]; RT.at<double>(2,3) =  calib[11];
  RT.at<double>(3,0) = calib[12]; RT.at<double>(3,1) = calib[13]; RT.at<double>(3,2) =  calib[14]; RT.at<double>(3,3) =  calib[15];

  //相机的内参矩阵
  P_rect_00.at<double>(0,0) = camera_matrix[0]; P_rect_00.at<double>(0,1) = camera_matrix[1]; P_rect_00.at<double>(0,2) = camera_matrix[2]; P_rect_00.at<double>(0,3) = 0.000000e+00;  //这里是内参标定的projection矩阵：外部世界坐标到像平面的投影矩阵
  P_rect_00.at<double>(1,0) = camera_matrix[3]; P_rect_00.at<double>(1,1) = camera_matrix[4]; P_rect_00.at<double>(1,2) = camera_matrix[5]; P_rect_00.at<double>(1,3) = 0.000000e+00;  
  P_rect_00.at<double>(2,0) = camera_matrix[6]; P_rect_00.at<double>(2,1) = camera_matrix[7]; P_rect_00.at<double>(2,2) = camera_matrix[8]; P_rect_00.at<double>(2,3) = 0.000000e+00;

  K.at<double>(0,0) = P_rect_00.at<double>(0,0);  K.at<double>(0,1) = P_rect_00.at<double>(0,1);  K.at<double>(0,2) = P_rect_00.at<double>(0,2);
  K.at<double>(1,0) = P_rect_00.at<double>(1,0);  K.at<double>(1,1) = P_rect_00.at<double>(1,1);  K.at<double>(1,2) = P_rect_00.at<double>(1,2);
  K.at<double>(2,0) = P_rect_00.at<double>(2,0);  K.at<double>(2,1) = P_rect_00.at<double>(2,1);  K.at<double>(2,2) = P_rect_00.at<double>(2,2);

  D.at<double>(0,0) = distort[0];  D.at<double>(1,0) = distort[1];  D.at<double>(2,0) = distort[2];  D.at<double>(3,0) = distort[3];  D.at<double>(4,0) = distort[4];


  fx = camera_matrix[0]; fy = camera_matrix[4]; cx = camera_matrix[2]; cy = camera_matrix[5];

  NewCameraMatrix = cv::getOptimalNewCameraMatrix(K, D, cv::Size(imagewidth, imageheight), alpha, cv::Size(imagewidth, imageheight), 0);
  cv::initUndistortRectifyMap(K, D, cv::Mat(), NewCameraMatrix, cv::Size(imagewidth, imageheight), CV_32FC1, map1, map2);

}

void fusion::run(){
    cam_sub = nh.subscribe<sensor_msgs::Image>(cam_sub_topic,1, &fusion::cam_callback,this);//相机话题
    lidar_sub = nh.subscribe<sensor_msgs::PointCloud2>(lidar_sub_topic,1, &fusion::lidar_callback, this); //接收rslidar点云数据，进入回调函数getcloud())
    lidar_pub = nh.advertise<sensor_msgs::PointCloud2>(lidar_pub_topic,1);//融合后点云可视化结果
    cam_pub = nh.advertise<sensor_msgs::Image>(cam_pub_topic,1);//融合后图像可视化话题
}

void fusion::lidar_callback(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

  if(is_get_image){

    pcl::PointCloud<pcl::PointXYZI>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);//点云下采样
    pcl::PointCloud<pcl::PointXYZI>::Ptr raw_pcl_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<PointXYZRGBI>::Ptr  fusion_pcl_ptr (new pcl::PointCloud<PointXYZRGBI>);   //放在这里是因为，每次都需要重新初始化
    pcl::fromROSMsg(*laserCloudMsg, *current_pc_ptr);
    pcl::VoxelGrid<pcl::PointXYZI> vg;

    vg.setInputCloud(current_pc_ptr);
    vg.setLeafSize(0.2f, 0.2f, 0.2f);
    vg.filter(*raw_pcl_ptr);

                //pcl::PointCloud<pcl::PointXYZI>::Ptr raw_pcl_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    //pcl::PointCloud<PointXYZRGBI>::Ptr  fusion_pcl_ptr (new pcl::PointCloud<PointXYZRGBI>);   //放在这里是因为，每次都需要重新初始化
    //pcl::PointCloud<PointXYZIR>::Ptr   raw_pcl_ptr (new pcl::PointCloud<PointXYZIR>);   //对应rslidar的XYZI格式点云，命名为XYZIR是因为命名为XYZI会报错但实际头文件定义里没有ring通道，如果实际使用rslidar的XYZIRT点云的话需要自己定义一个点类型PointXYZIRT
    //pcl::fromROSMsg(*laserCloudMsg, *raw_pcl_ptr);  //把msg消息指针转化为点云指针
    

    //另一种做法
    cv::Mat X(4,1,cv::DataType<double>::type);//3D-2D变换
    cv::Mat Y(3,1,cv::DataType<double>::type); 

    for (int i = 0; i <  raw_pcl_ptr->points.size(); i++)
    {

      X.at<double>(0,0) = raw_pcl_ptr->points[i].x;
      X.at<double>(1,0) = raw_pcl_ptr->points[i].y;
      X.at<double>(2,0) = raw_pcl_ptr->points[i].z;
      X.at<double>(3,0) = 1;
      Y = P_rect_00 * RT *X;  //坐标转换，velo2image，Y = P_rect_00 * X为相机的世界坐标点投影到像素坐标点，Y = P_rect_00 * RT * X为点云坐标投影到像素坐标（RT * X）为点云坐标投影到相机的世界坐标
      cv::Point pt;//点云投影到最终像素坐标点（u,v）
      pt.x =  Y.at<double>(0,0) / Y.at<double>(0,2) ;// Y.at<double>(0,2)
      pt.y = Y.at<double>(1,0) / Y.at<double>(0,2) ;
      // std::cout<<  pt << std::endl;

      double val = raw_pcl_ptr->points[i].x;//融合图像
      double maxVal = 100.0;
      int red = min(255, (int)(255 * abs((val - maxVal) / maxVal)));
      int green = min(255, (int)(255 * (1 - abs((val - maxVal) / maxVal))));
      cv::circle(overlay, pt, 5, cv::Scalar(0, green, red), -1);

      if ( pt.x >=0 &&pt.x <  visImg.cols && pt.y>=0 && pt.y<visImg.rows && raw_pcl_ptr->points[i].x>0) //&& raw_pcl_ptr->points[i].x>0去掉图像后方的点云
      {
        PointXYZRGBI  p;
        //融合后RGB点云（x,y,z）
        p.x=raw_pcl_ptr->points[i].x;
        p.y=raw_pcl_ptr->points[i].y;
        p.z=raw_pcl_ptr->points[i].z;
        //点云颜色由图像上对应点确定
        cv::Vec3b color = getColor(pt.y, pt.x);
        p.b = color[0];
        p.g = color[1];
        p.r = color[2];

        p.intensity = raw_pcl_ptr->points[i].intensity;  //继承之前点云的intensity
        fusion_pcl_ptr->points.push_back(p);
      }
    }

    float opacity = 0.6f;
    cv::addWeighted(overlay, opacity, visImg, 1 - opacity, 0, visImg);//加权融合图像overlay和visImg,最终输出visImg
    image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", visImg).toImageMsg();//将图像转化为消息才能发布
    cam_pub.publish(image_msg);//发布投影后的图像
    
    fusion_pcl_ptr->width = 1;
    fusion_pcl_ptr->height = fusion_pcl_ptr->points.size();
    // std::cout<<  fusion_pcl_ptr->points.size() << std::endl;
    pcl::toROSMsg( *fusion_pcl_ptr, fusion_msg);  //将点云转化为消息才能发布
    fusion_msg.header = laserCloudMsg->header;
    //fusion_msg.header.frame_id = "rslidar";//帧id改成和rslidar一样的
    lidar_pub.publish(fusion_msg); //发布调整之后的点云数据，主题为/fusion_cloud

  }
  else
  {
    std::cout<<"Waiting for image topic"<<std::endl;
  }

}

void fusion::cam_callback(const sensor_msgs::ImageConstPtr& Imgmsg){
   try{
    cv::Mat image  = cv_bridge::toCvShare(Imgmsg, "bgr8")->image; //image就是我们得到的图像了
    //cv::undistort(image, visImg, K, D);//图像去畸变
    cv::remap(image, visImg, map1, map2, cv::INTER_LINEAR);//图像去畸变
    //visImg = image.clone();
    overlay = visImg.clone();
    // cv::circle(image,cv::Point(100,250),5,cv::Scalar(0,0,255),3); //注意先列后行
    for (int row = 0; row < visImg.rows; row++ )
    {
      for (int  col= 0; col< visImg.cols; col++ )
      {
        setColor(row, col, visImg.at<cv::Vec3b>(row, col));
      }
    }
    // cv::imshow("view", image);
    is_get_image = 1;
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.",Imgmsg->encoding.c_str());
  }   
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_cam_fusion");//ROS节点初始化
  fusion fs;
  fs.run();
  ros::spin();
  // cv::destroyWindow("view");
}
