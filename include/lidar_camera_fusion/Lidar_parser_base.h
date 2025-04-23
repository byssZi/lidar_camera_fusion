#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/*
    * 一个具有XYZ、RGB、intensity的点云类型
    */
   struct PointXYZRGBI
   {
       PCL_ADD_POINT4D;
       PCL_ADD_RGB;
       PCL_ADD_INTENSITY;
       EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
   } EIGEN_ALIGN16;
   
   POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZRGBI,
                               (float, x, x)
                               (float, y, y)
                               (float, z, z)
                               (float, rgb, rgb)
                               (float, intensity, intensity)
   )
