/**
 * @file planner.h
 * @copyright Copyright (C) 2023 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#ifndef ROKAEAPI_PLANNER_H
#define ROKAEAPI_PLANNER_H

#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif // defined(_MSC_VER) && (_MSC_VER >= 1200)

#include <Eigen/Core>
#include "base.h"
#include "motion_control_rt.h"
#include "model.h"
#include "data_types.h"

namespace rokae {

/**
 * @class CartMotionGenerator
 * @brief S速度规划的笛卡尔空间运动。
 * 参考文献: Wisama Khalil and Etienne Dombre. 2002. Modeling, Identification and Control of Robots
 * (Kogan Page Science Paper edition).
 */
 class CartMotionGenerator {
  public:
   /**
    * @brief 根据关节目标位置和速度系数生成一条轴空间轨迹，可用来回零或到达指定位置。
    * @param[in] speed_factor 速度系数，范围[0, 1].
    * @param[in] s_goal 目标关节角度
    */
   CartMotionGenerator(double speed_factor, double s_goal);
   ~CartMotionGenerator();

   /**
    * @brief 设置笛卡尔空间运动参数
    * @param[in] ds_max 最大速度
    * @param[in] dds_max_start 最大开始加速度
    * @param[in] dds_max_end 最大结束加速度
    */
   void setMax(double ds_max, double dds_max_start, double dds_max_end);

   /**
    * @brief 获得总运动时间
    */
   double getTime();

   /**
    * @brief 计算时间t时的弧长s
    * @return false: 运动规划没有结束 | true: 运动规划结束
    */
   bool calculateDesiredValues(double t, double *delta_s_d) const;

   /**
    * @brief s规划，笛卡尔空间是弧长s规划
    */
   void calculateSynchronizedValues(double s_init);

  ROKAE_DECLARE_IMPL
 };

/**
 * @class JointMotionGenerator
 * @brief S速度规划的轴空间运动。参考文献:
 * Wisama Khalil and Etienne Dombre. 2002. Modeling, Identification and Control of Robots
 * (Kogan Page Science Paper edition).
 */
 class JointMotionGenerator {
   using VectorNd = Eigen::Matrix<double, 7, 1, Eigen::ColMajor>;
   using VectorNi = Eigen::Matrix<int, 7, 1, Eigen::ColMajor>;

  public:
   /**
    * @brief 根据关节目标位置和速度系数生成一条轴空间轨迹，可用来回零或到达指定位置。
    * @param[in] speed_factor 速度系数，范围[0, 1]
    * @param[in] q_goal 目标关节角度
    */
   JointMotionGenerator(double speed_factor, std::array<double, 7> q_goal);
   virtual ~JointMotionGenerator();

   /**
    * @brief 设置轴空间运动参数
    * @param[in] dq_max 最大速度
    * @param[in] ddq_max_start 最大开始加速度
    * @param[in] ddq_max_end 最大结束加速度
    */
   void setMax(const std::array<double, 7> &dq_max,
               const std::array<double, 7> &ddq_max_start,
               const std::array<double, 7> &ddq_max_end);

   /**
    * @brief 获得总运动时间
    */
   double getTime();

   /**
    * @brief 计算时间t时的关节角度增量
    * @param[in] t 时间点
    * @param[out] delta_q_d 计算结果
    * @return false: 运动规划没有结束 | true: 运动规划结束
    */
   bool calculateDesiredValues(double t, std::array<double, 7> &delta_q_d) const;

   /**
    * @brief s规划
    * @param q_init
    */
   void calculateSynchronizedValues(const std::array<double, 7> &q_init);

  ROKAE_DECLARE_IMPL
 };

}  // namespace rokae

#endif //ROKAEAPI_PLANNER_H
