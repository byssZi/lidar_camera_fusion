#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/*
    * 一个具有XYZ、RGB、intensity的点云类型
    */
struct PointXYZRGBIUV
{
    PCL_ADD_POINT4D;            // x, y, z, padding
    PCL_ADD_RGB;                // rgb
    float intensity;            // 强度
    float u;                    // 图像像素坐标 u
    float v;                    // 图像像素坐标 v

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

   
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRGBIUV,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, rgb, rgb)
    (float, intensity, intensity)
    (float, u, u)
    (float, v, v)
)

