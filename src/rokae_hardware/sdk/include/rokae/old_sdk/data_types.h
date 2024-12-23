/**
 * @file data_types.h
 * @copyright Copyright (C) 2023 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#ifndef ROKAEAPI_INCLUDE_ROKAE_DATA_TYPES_H_
#define ROKAEAPI_INCLUDE_ROKAE_DATA_TYPES_H_

#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif // defined(_MSC_VER) && (_MSC_VER >= 1200)

#include <array>
#include <vector>
#include <string>
#include "base.h"

namespace rokae {

 const int USE_DEFAULT = -1;
 const int Unknown = -1;

// ********************         Enum class          ***********************
 /**
  * @enum OperationState
  * @brief 机器人工作状态
  */
 enum class OperationState {
   idle             = 0, ///< 机器人静止
   jog              = 1, ///< jog
   rci              = 2, ///< RCI控制中
   drag             = 3, ///< 拖动已开启
   rlProgram        = 4, ///< RL工程运行中
   demo             = 5, ///< Demo演示中
   dynamicIdentify  = 6, ///< 动力学辨识中
   frictionIdentify = 7, ///< 摩擦力辨识中
   loadIdentify     = 8, ///< 负载辨识中
   moving           = 9, ///< 机器人运动中
   unknown          = Unknown ///< 未知
 };

 /**
  * @enum WorkType
  * @brief 机型类别
  */
 enum class WorkType {
   industrial,   ///< 工业机器人
   collaborative ///< 协作机器人
 };

 /**
  * @enum OperateMode
  * @brief 机器人操作模式
  */
 enum class OperateMode {
   manual    = 0,      ///< 手动
   automatic = 1,      ///< 自动
   unknown   = Unknown ///< 未知(发生异常)
 };

 /**
  * @enum PowerState
  * @brief 机器人上下电及急停状态
  */
 enum class PowerState {
   on      = 0, ///< 上电
   off     = 1, ///< 下电
   estop   = 2, ///< 急停被按下
   gstop   = 3, ///< 安全门打开
   unknown = Unknown ///< 未知(发生异常)
 };

 /**
  * @enum MotionControlMode
  * @brief SDK运动控制模式
  */
 enum class MotionControlMode : unsigned {
   Idle,       ///< 空闲
   NrtCommand, ///< 非实时模式执行运动指令
   NrtRLTask,  ///< 非实时模式运行RL工程, 目前仅支持查询当前加载的工程的工具工件信息
   RtCommand,  ///< 实时模式控制
 };

 /**
  * @enum RtControllerMode
  * @brief 控制器实时控制模式
  */
 enum class RtControllerMode : unsigned {
   jointPosition,      ///< 实时轴空间位置控制
   cartesianPosition,  ///< 实时笛卡尔空间位置控制
   jointImpedance,     ///< 实时轴空间阻抗控制
   cartesianImpedance, ///< 实时笛卡尔空间阻抗控制
   torque              ///< 实时力矩控制
 };

 namespace RtSupportedFields {
  /// 说明：数据名后为数据类型
  /// ArrayXD = std::array<double, DoF> , DoF为轴数
  /// Array6D = std::array<double, 6>, 以此类型
  constexpr const char *jointPos_m = "q_m";   ///< 关节角度 - ArrayXD
  constexpr const char *jointPos_c = "q_c";   ///< 指令关节角度 - ArrayXD
  constexpr const char *jointVel_m = "dq_m";  ///< 关节速度 - ArrayXD
  constexpr const char *jointVel_c = "dq_c";  ///< 指令关节速度 - ArrayXD
  constexpr const char *jointAcc_c = "ddq_c"; ///< 指令关节加速度 - ArrayXD
  constexpr const char *tcpPose_m  = "pos_m"; ///< 机器人位姿,相对于基坐标系 - Array16D
  constexpr const char *tcpPose_c  = "pos_c"; ///< 指令机器人位姿 - Array16D
  constexpr const char *tcpVel_c   = "pos_vel_c"; ///< 指令机器人末端速度 - Array6D
  constexpr const char *tcpAcc_c   = "pos_acc_c"; ///< 指令机器人末端加速度 - Array6D
  constexpr const char *psi_m      = "psi_m";     ///< 臂角 - double
  constexpr const char *psi_c      = "psi_c";     ///< 指令臂角 - double
  constexpr const char *psiVel_c   = "psi_vel_c"; ///< 指令臂角速度 - double
  constexpr const char *psiAcc_c   = "psi_acc_c"; ///< 指令臂角加速度 - double
  constexpr const char *tau_m      = "tau_m";     ///< 关节力矩 - ArrayXD
  constexpr const char *tau_c      = "tau_c";     ///< 指令关节力矩 - ArrayXD
  constexpr const char *tauFiltered_m    = "tau_filtered_m"; ///< 滤波后关节力矩 - ArrayXD
  constexpr const char *tauVel_c         = "tau_vel_c";      ///< 指令力矩微分 - ArrayXD
  constexpr const char *tauExt_inBase    = "tau_ext_base";   ///< 基坐标系中外部力矩 - Array6D
  constexpr const char *tauExt_inStiff   = "tau_ext_stiff";  ///< 力控坐标系中外部力矩 - Array6D
  constexpr const char *theta_m          = "theta_m";        ///< 电机位置 - ArrayXD
  constexpr const char *thetaVel_m       = "theta_vel_m";        ///< 电机位置微分 - ArrayXD
  constexpr const char *motorTau         = "motor_tau";          ///< 电机转矩 - ArrayXD
  constexpr const char *motorTauFiltered = "motor_tau_filtered"; ///< 滤波后电机转矩 - ArrayXD
 }

 /**
  * @enum StopLevel
  * @brief 机器人停止运动等级, 目前仅支持stop2
  */
 enum class StopLevel {
   stop0, ///< 快速停止机器人运动后断电
   stop1, ///< 规划停止机器人运动后断电, 停在原始路径上
   stop2  ///< 规划停止机器人运动后不断电, 停在原始路径上
 };

 /**
  * @struct DragParameter
  * @brief 机器人拖动模式参数, 包括拖动类型和空间
  */
 struct DragParameter {
   enum Space {
     jointSpace     = 0, ///< 轴空间
     cartesianSpace = 1  ///< 笛卡尔空间
   };
   enum Type {
     translationOnly = 0, ///< 仅平移
     rotationOnly    = 1, ///< 仅旋转
     freely          = 2  ///< 自由拖拽
   };
 };

 /**
  * @brief Jog选项: 坐标系
  */
 struct JogOpt {
   enum Space {
     world = 0, ///< 世界坐标系
     flange, ///< 法兰坐标系
     baseFrame, ///< 基坐标系
     toolFrame, ///< 工具坐标系
     wobjFrame, ///< 工件坐标系
     jointSpace ///< 轴空间
   };
 };

 /**
  * @brief xPanel配置: 对外供电模式
  */
 struct xPanelOpt {
   enum Vout {
     off,       ///< 不输出
     reserve,   ///< 保留
     supply12v, ///< 输出12V
     supply24v, ///< 输出24V
   };
 };


 // *********************   实时   ****************************************

 /**
  * @enum ForceControlFrameType
  * @brief 力控任务坐标系
  */
 enum class ForceControlFrameType {
   world = 0, ///< 世界坐标系
   tool  = 2, ///< 工具坐标系
   path  = 3  ///< 路径坐标系, 力控任务坐标系需要跟踪轨迹变化的过程
 };

// *******************          Data types            ********************
 /**
  * @class Frame
  * @brief 坐标系
  */
 class Frame {
  public:
   ROKAE_EXPORT Frame() = default;
   /**
    * @param trans 平移量
    * @param rpy 旋转量
    */
   ROKAE_EXPORT Frame(const std::array<double, 3> &trans, const std::array<double, 3> &rpy);
   /**
    * @param trans_rpy 长度应为6的初始化列表, {x, y, z, r, p, y}
    */
   ROKAE_EXPORT Frame(std::initializer_list<double> trans_rpy);

   std::array<double, 3> trans {}; ///< 平移量, [x, y, z], 单位:米
   std::array<double, 3> rpy {};   ///< 旋转量, [r, p, y], 单位:弧度
 };

 /**
  * @class CartesianPosition
  * @brief 笛卡尔点位
  */
 class CartesianPosition : public Frame {
  public:
   ROKAE_EXPORT CartesianPosition();
   /**
    * @param frame 长度应为6的初始化列表, {x, y, z, r, p, y}
    * @note Conf参数和外部关节角度可缺省
    */
   ROKAE_EXPORT CartesianPosition(std::initializer_list<double> frame);
   ROKAE_EXPORT CartesianPosition(const std::array<double, 6> &frame);
   /**
    * @param tran 平移量
    * @param rpy 旋转量
    */
   ROKAE_EXPORT CartesianPosition(const std::array<double, 3> &tran, const std::array<double, 3> &rpy);

   double elbow { 0 }; ///< 臂角, 适用于7轴机器人, 单位：弧度
   std::vector<double> confData; ///< 轴配置数据，元素个数应和机器人轴数一致
   std::vector<double> external; ///< 外部关节角度, 单位:弧度
 };

 /**
  * @class Finishable
  * @brief 一次运动循环是否结束
  */
 class Finishable {
  public:
   /**
    * @brief 是否已设置运动循环结束
    */
   uint8_t isFinished() const;

   /**
    * @brief 标识运动循环已结束
    */
   void setFinished();

  protected:
   uint8_t finished { 0 }; ///< 用于判断是否结束一个运动循环
 };

 /**
  * @class JointPosition
  * @brief 关节点位
  */
 class JointPosition : public Finishable {
  public:
   ROKAE_EXPORT JointPosition() = default;
   /**
    * @param joints 长度应与机器人轴数一致. 外部关节可缺省
    */
   ROKAE_EXPORT JointPosition(std::initializer_list<double> joints);
   ROKAE_EXPORT JointPosition(std::vector<double> joints);

   std::vector<double> joints; ///< 关节角度值, 单位:弧度
   std::vector<double> external; ///< 外部关节角度值, 单位:弧度
 };

 class CartesianPose : public Finishable {
  public:
   CartesianPose() = default;
   CartesianPose(const std::array<double,16> &pose);
   CartesianPose(std::initializer_list<double> pose);

   std::array<double, 16> pos {}; ///< 末端位姿, 相对于基坐标系
   double psi { 0 }; ///< 臂角
   bool hasPsi { false }; ///< 是否有臂角
 };

 /**
  * @class Torque
  * @brief 关节扭矩，不包含重力和摩擦力
  */
 class Torque : public Finishable {
  public:
   ROKAE_EXPORT Torque() = default;
   ROKAE_EXPORT Torque(std::vector<double> tau);
   ROKAE_EXPORT Torque(std::initializer_list<double> tau);

   std::vector<double> tau; ///< 期望关节扭矩，单位: Nm
 };

 /**
   * @class Load
   * @brief 负载信息
   */
 class Load {
  public:
   ROKAE_EXPORT Load() = default;
   /**
    * @param m 质量
    * @param cog 质心
    * @param inertia 惯量
    */
   ROKAE_EXPORT Load(double m, const std::array<double, 3> &cog, const std::array<double, 3> &inertia);

   double mass { 0 };  ///< 负载质量, 单位:千克
   std::array<double, 3> cog {};     ///< 质心 [x, y, z], 单位:米
   std::array<double, 3> inertia {}; ///< 惯量 [ix, iy, iz, 单位:千克·平方米
 };

 /**
  * @class Toolset
  * @brief 工具工件组信息, 根据一对工具工件的坐标、负载、机器人手持设置计算得出
  * @note 并不显式区分手持/外部. 该类可这样理解: 如手持工具, 则负载和机器人末端坐标系是工具的, 参考坐标系则是工件的；
  *       反之, 如果手持工件, 则负载和末端坐标系来自工件, 参考坐标系来自工具
  */
 class Toolset {
  public:
   ROKAE_EXPORT Toolset() = default;
   /**
    * @param load 负载信息
    * @param end 末端坐标系
    * @param ref 参考坐标系
    */
   ROKAE_EXPORT Toolset(const Load &load, const Frame &end, const Frame &ref);

   Load load {}; ///< 机器人末端手持负载
   Frame end {}; ///< 机器人末端坐标系相对法兰坐标系转换
   Frame ref {}; ///< 机器人参考坐标系相对世界坐标系转换
 };

 /**
  * @class RLProjectInfo
  * @brief RL工程信息
  */
 class RLProjectInfo {
  public:
   ROKAE_EXPORT RLProjectInfo(std::string name);
   std::string name; ///< 工程名称
   std::vector<std::string> taskList; ///< 任务名称列表
 };

 /**
  * @class WorkToolInfo
  * @brief 工具/工件信息。工件的坐标系已相对其用户坐标系变换
  */
 class WorkToolInfo {
  public:
   ROKAE_EXPORT WorkToolInfo() = default;
   ROKAE_EXPORT WorkToolInfo(std::string name, bool isHeld, const Frame &posture, const Load &load);

   std::string name {};  ///< 名称
   std::string alias {}; ///< 别名, 暂未使用
   bool robotHeld {};    ///< 是否机器人手持
   Frame pos {};         ///< 位姿
   Load load {};         ///< 负载
 };

 /**
  * @class NrtCommand
  * @brief 非实时运动指令
  */
 class NrtCommand {
  public:
   /**
    * @brief 运动类型(moveL/moveJ/moveC)
    */
   const std::string &type() const;
   int speed; ///< 速率
   int zone;  ///< 转弯区大小

  protected:
   explicit NrtCommand(std::string id, int speed = USE_DEFAULT, int zone = USE_DEFAULT);
   virtual ~NrtCommand() = default;
   const std::string type_; ///< 运动类别，内部使用
 };

 /**
  * @class MoveAbsJCommand
  * @brief MoveAbsJ运动指令
  */
 class MoveAbsJCommand : public NrtCommand{
  public:
   /**
    * @param target 目标点位
    */
   ROKAE_EXPORT MoveAbsJCommand(JointPosition target);
   /**
    * @param target 目标点位
    * @param speed 运行速度
    * @param zone 转弯区
    */
   ROKAE_EXPORT MoveAbsJCommand(JointPosition target, int speed, int zone);

   /**
    * @brief 目标关节点位
    */
   JointPosition target;
 };

 /**
  * @class MoveJCommand
  * @brief 运动指令MoveJ
  */
 class MoveJCommand : public NrtCommand {
  public:
   /**
    * @param target 目标笛卡尔点位
    */
   ROKAE_EXPORT MoveJCommand(CartesianPosition target);
   ROKAE_EXPORT MoveJCommand(CartesianPosition &&target);
   /**
    * @param target 目标笛卡尔点位
    * @param speed 运行速度
    * @param zone 转弯区
    */
   ROKAE_EXPORT MoveJCommand(CartesianPosition target, int speed, int zone);

   /**
    * @brief 目标笛卡尔点位
    */
   CartesianPosition target;
 };

 /**
  * @class MoveLCommand
  * @brief 运动指令MoveL
  */
 class MoveLCommand : public NrtCommand {
  public:
   /**
    * @param target 目标笛卡尔点位
    */
   ROKAE_EXPORT MoveLCommand(CartesianPosition target);
   /**
    * @param target 目标笛卡尔点位
    * @param speed 速率
    * @param zone 转弯区
    */
   ROKAE_EXPORT MoveLCommand(CartesianPosition target, int speed, int zone);
   /**
    * @brief 目标笛卡尔点位
    */
   CartesianPosition target;
 };

 /**
  * @class MoveCCommand
  * @brief 运动指令MoveC
  */
 class MoveCCommand : public NrtCommand {
  public:
   /**
    * @param target 目标点
    * @param aux 辅助点
    */
   ROKAE_EXPORT MoveCCommand(CartesianPosition target, CartesianPosition aux);
   /**
    * @param target 目标点
    * @param aux 辅助点
    * @param speed 运行速度
    * @param zone 转弯区
    */
   ROKAE_EXPORT MoveCCommand(CartesianPosition target, CartesianPosition aux, int speed, int zone);
   /**
    * @brief 目标笛卡尔点位
    */
   CartesianPosition target;
   /**
    * @brief 辅助点位
    */
   CartesianPosition aux;
 };

 /**
  * @class LogInfo
  * @brief 控制器日志信息
  */
 class LogInfo {
  public:
   enum Level {
     info, warning, error
   };
   LogInfo(int id, std::string ts, std::string ct, std::string r);
   const int id;                ///< 日志ID号
   const std::string timestamp; ///< 日期及时间
   const std::string content;   ///< 日志内容
   const std::string repair;    ///< 修复办法
 };

}  // namespace rokae

#endif //ROKAEAPI_INCLUDE_ROKAE_DATA_TYPES_H_
