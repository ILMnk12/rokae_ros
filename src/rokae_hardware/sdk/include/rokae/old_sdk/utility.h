/**
 * @file utility.h
 * @copyright Copyright (C) 2023 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#ifndef ROKAEAPI_INCLUDE_ROKAE_UTILITY_H_
#define ROKAEAPI_INCLUDE_ROKAE_UTILITY_H_

#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif // defined(_MSC_VER) && (_MSC_VER >= 1200)

#include "base.h"
#include <Eigen/Geometry>

namespace rokae {
 /**
  * @class Utils
  * @brief 数据类型转换工具类
  */
 class Utils {

  public:

   /**
    * @brief 度转弧度
    */
   static double degToRad(double degrees) {
     return degrees * (EIGEN_PI / 180);
   }

   /**
    * @brief 弧度转度
    */
   static double radToDeg(double rad) {
     return rad / EIGEN_PI * 180.0;
   }

   /**
    * @brief 数组度转弧度
    */
   template<size_t S>
   static std::array<double, S> degToRad(const std::array<double, S> &degrees) {
     std::array<double, S> ret {};
     std::transform(degrees.cbegin(), degrees.cend(), ret.begin(),
                    [](const double &d) { return degToRad(d);});
     return ret;
   }

   /**
    * @brief 数组弧度转度
    */
   template<size_t S>
   static std::array<double, S> radToDeg(const std::array<double, S> &rad) {
     std::array<double, S> ret {};
     std::transform(rad.cbegin(), rad.cend(), ret.begin(),
                    [](const double &d) { return radToDeg(d);});
     return ret;
   }

   /**
    * @brief 变换矩阵转为数组
    * @param[in] rot 旋转矩阵
    * @param[in] trans 平移向量
    * @param[out] array 转换结果，行优先
    */
   static void transMatrixToArray(const Eigen::Matrix3d& rot, const Eigen::Vector3d& trans, std::array<double, 16>& array) {
     array = {{rot(0, 0), rot(0, 1), rot(0, 2), trans(0),
               rot(1, 0), rot(1, 1), rot(1, 2), trans(1),
               rot(2, 0), rot(2, 1), rot(2, 2), trans(2),
               0,         0,         0,         1       }};
   }

   /**
    * @brief 数组转为变换矩阵
    * @param[in] array 数组, 行优先
    * @param[out] rot 旋转矩阵
    * @param[out] trans 平移向量
    */
   static void arrayToTransMatrix(const std::array<double, 16>& array, Eigen::Matrix3d& rot, Eigen::Vector3d& trans) {
     rot << array[0], array[1], array[2], array[4], array[5], array[6], array[8], array[9], array[10];
     trans << array[3], array[7], array[11];
   }

   /**
    * @brief 欧拉角转为旋转矩阵
    * @param[in] euler 欧拉角, 顺序[z, y, x], 单位: 弧度
    * @param[out] matrix 旋转矩阵
    */
   static void eulerToMatrix(const Eigen::Vector3d& euler, Eigen::Matrix3d& matrix) {
     Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitX()));
     Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY()));
     Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitZ()));
     matrix = yawAngle * pitchAngle * rollAngle;
   }

   Utils() = delete;
   ~Utils() = delete;
 };
}  // namespace rokae

#endif //ROKAEAPI_INCLUDE_ROKAE_UTILITY_H_
