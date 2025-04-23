***
# 说明
此工程用于ros1下将lidar的点云投影到camera的image上，将image的rgb染色到lidar的点云
***
# 准备工作
## Step1
camera内参标定，lidar和camera外参标定
## Step2
将标定好的参数填入cfg/params.yaml文件中
|参数名称|功能描述|备注|
|---|---|---|
|lidar_input_topic|接收lidar点云的topic话题| - |
|lidar_output_topic|发布融合染色后点云的topic话题| - |
|cam_input_topic|接收camera图像的topic话题| - |
|cam_output_topic|发布融合lidar点云投影后图像的话题| - |
|camera_matrix|camera内参|3x3的矩阵|
|RT|lidar到camera的外参|4x4的矩阵[R,t]|
|distort|camera的畸变系数|k1,k2,p1,p2,k3|
***
# 启动程序
## Step1
```bash
catkin_make
source devel/setup.bash
```
## Step2
```bash
roslaunch lidar_camera_fusion run.launch
```
## Step3
打开rviz观察发布结果
***


