/**
 * @file robot.h
 * @copyright Copyright (C) 2023 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#ifndef ROKAEAPI_ROBOT_H_
#define ROKAEAPI_ROBOT_H_

#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif // defined(_MSC_VER) && (_MSC_VER >= 1200)

#include "base.h"
#include "model.h"
#include "exception.h"
#include "motion_control_rt.h"
#include "planner.h"

namespace rokae {

 // forward declarations
 class BaseModel;

 /**
  * @struct Info
  * @brief 机器人基本信息，在与建立机器人连接后加载
  */
 struct Info {
   std::string id;      ///< 机器人uid, 可用于区分连接的机器人
   std::string version; ///< 控制器版本
   std::string type;    ///< 机器人机型名称
   int joint_num;       ///< 轴数
 };

 /**
  * @class BaseRobot
  * @brief 机器人通用接口
  */
 class ROKAE_EXPORT BaseRobot : public Base<BaseRobot> {

  public:

   // **********************************************************************
   // *************       Network communication methods      ***************
   // *************                网络通信接口                ***************

   /**
    * @brief 建立与机器人的连接。机器人地址和端口号为创建Robot实例时传入的
    * @param[out] ec 错误码
    */
   void connectToRobot(error_code &ec) noexcept;

   /**
    * @brief 断开与机器人连接。断开前会停止机器人运动, 请注意安全
    * @param[out] ec 错误码
    */
   void disconnectFromRobot(error_code &ec) noexcept;

   // **********************************************************************
   // ***************      Motors power on/off methods      ****************
   // ***************              电机上下电接口             *****************

   /**
    * @brief 机器人上下电以及急停状态
    * @param[out] ec 错误码
    * @return on-上电 | off-下电 | estop-急停 | gstop-安全门打开
    */
   PowerState powerState(error_code &ec) const noexcept;

   /**
   * @brief 机器人上下电。注: 只有无外接使能开关或示教器的机器人才能手动模式上电。
   * @param[in] on true-上电 | false-下电
   * @param[out] ec 错误码
   */
   void setPowerState(bool on, error_code &ec) noexcept;


   // **********************************************************************
   // ***************     Robot operation mode methods      ****************
   // ***************            机器人操作模式接口            ****************

   /**
    * @brief 查询机器人当前操作模式
    * @param[out] ec 错误码
    * @return 手动 | 自动
    */
   OperateMode operateMode(error_code &ec) const noexcept;

   /**
    * @brief 切换手自动模式
    * @param[in] mode 手动/自动
    * @param[out] ec 错误码
    */
   void setOperateMode(OperateMode mode, error_code &ec) noexcept;

   // **********************************************************************
   // **************       Get robot information methods      **************
   // **************           查询机器人信息及状态接口           **************
   /**
    * @brief 查询机器人基本信息
    * @param[out] ec 错误码
    * @return 机器人基本信息
    */
   Info robotInfo(error_code &ec) const noexcept;

   /**
    * @brief 查询机器人当前运行状态 (空闲,运动中, 拖动开启等)
    * @param[out] ec 错误码
    * @return 运行状态枚举类
    */
   OperationState operationState(error_code &ec) const noexcept;

   // **********************************************************************
   // ******************      Get robot posture methods      ***************
   // ******************           获取机器人位姿接口           ***************
   /**
    * @brief 机器人法兰相对于基坐标系位姿 \f$^{O}T_{F}~[m][rad]\f$.
    * @param[out] ec 错误码
    * @return double数组, 长度: \f$ \mathbb{R}^{6 \times 1} \f$ = \f$ \mathbb{R}^{3 \times 1} \f$
    * transformation and \f$ \mathbb{R}^{3 \times 1} \f$ rotation \f$ [x, y, z, a, b, c]^T \f$.
    */
   std::array<double, 6> flangePos(error_code &ec) noexcept;


   // **********************************************************************
   // ********************    Input/Output devices     *********************
   // ********************         输入/输出设备         *********************
   /**
    * @brief 查询数字量输入信号值
    * @param[in] board IO板序号
    * @param[in] port 信号端口号
    * @param[out] ec 错误码
    * @return true-开 | false-关
    */
   bool getDI(unsigned int board, unsigned int port, error_code &ec) noexcept;

   /**
    * @brief 设置数字量输出信号值
    * @param[in] board IO板序号
    * @param[in] port 信号端口号
    * @param[in] state true-开 | false-关
    * @param[out] ec 错误码
    */
   void setDO(unsigned int board, unsigned int port, bool state, error_code &ec) noexcept;

   /**
    * @brief 查询数字输出量信号值
    * @param[in] board IO板序号
    * @param[in] port 信号端口号
    * @param[out] ec 错误码
    * @return true-开 | false-关
    */
   bool getDO(unsigned int board, unsigned int port, error_code &ec) noexcept;

   /**
    * @brief 读取模拟量输入信号值
    * @param[in] board IO板序号
    * @param[in] port 信号端口号
    * @param[out] ec 错误码
    * @return 信号值
    */
   double getAI(unsigned board, unsigned port, error_code &ec) noexcept;

   /**
    * @brief 设置模拟量输出信号
    * @param[in] board IO板序号
    * @param[in] port 信号端口号
    * @param[in] value 输出值
    * @param[out] ec 错误码
    */
   void setAO(unsigned board, unsigned port, double value, error_code &ec) noexcept;

   /**
    * @brief 读取寄存器值。可读取单个寄存器，寄存器数组，或按索引读取寄存器数组。
    * 如果要读取整个寄存器数组，value传入对应类型的vector，index值被忽略。
    * @tparam T 读取数值类型
    * @param[in] name 寄存器名称
    * @param[in] index 按索引读取寄存器数组中元素，从0开始。
    *           下列两种情况会报错：1) 索引超出数组长度; 2) 寄存器不是数组但index大于0
    * @param[out] value 寄存器数值，允许的类型有bool/int/float
    * @param[out] ec 错误码
    */
   template<typename T>
   void readRegister(const std::string &name, unsigned index, T &value, error_code &ec) noexcept;

   /**
    * @brief 写寄存器值。可写入单个寄存器，或按索引写入寄存器数组中某一元素。
    * @tparam T 写入数值类型
    * @param[in] name 寄存器名称
    * @param[in] index 数组索引，从0开始。
    *           下列两种情况会报错：1) 索引超出数组长度; 2) 寄存器不是数组但index大于0
    * @param[in] value 写入的数值
    * @param[out] ec 错误码
    */
   template<typename T>
   void writeRegister(const std::string &name, unsigned index, T value, error_code &ec) noexcept;


   // **********************************************************************
   // ******************        Other operations      **********************
   // ******************             其他操作           **********************
   /**
    * @brief 清除伺服报警
    * @param[out] ec 错误码，当有伺服报警且清除失败的情况下错误码置为-1
    */
   void clearServoAlarm(error_code &ec) noexcept;

   /**
    * @brief 查询RokaeSDK版本
    * @return 版本号
    */
   static std::string sdkVersion() noexcept;

   /**
   * @brief 获取最新的错误码, 目前为运动指令的执行结果
   * @return 错误码,可调用message()获取详细信息
   */
   std::error_code lastErrorCode() noexcept;

   /**
    * @brief 查询控制器最新的日志
    * @param[in] count 查询个数，上限是10条
    * @param[in] level 指定日志等级，空集合代表不指定
    * @param[out] ec 错误码
    * @return 日志信息
    */
   std::vector<LogInfo> queryControllerLog(unsigned count, const std::set<LogInfo::Level>& level, error_code &ec);

   // **********************************************************************
   // ******************      Motion Controller        *********************
   // ******************        运动控制相关接口          *********************

   /**
    * @brief 设置运动控制模式
    * @note 在调用各运动控制接口之前, 必须设置对应的控制模式。
    * @param[in] mode 模式
    * @param[out] ec 错误码
    */
   void setMotionControlMode(MotionControlMode mode, error_code &ec) noexcept;

   //  --------------    MotionControlMode::NrtCommand   ------------------
   //  ----------------         非实时运动指令              ------------------
   /**
    * @brief 重置运动缓存
    * @note 清空已发送的运动指令, 默认速度重置为100(v100), 默认转弯区重置为0(fine).
    *       每次程序开始运行并第一次执行运动指令之前, 必须调用该函数来重置运动缓存, 否则控制器可能会报错
    * @param[out] ec 错误码
    */
   void moveReset(error_code &ec) noexcept;

   /**
    * @brief 停止机器人运动
    * @note 目前支持stop2停止类型, 规划停止不断电, 参见StopLevel。
    *       调用此接口后, 已经下发的运动指令会被清除, 不再执行。
    * @param[out] ec 错误码
    */
   void stop(error_code &ec) noexcept;

   /**
    * @brief 执行单条或多条运动指令，调用后机器人立刻开始运动
    * @note 须为同类型的指令;
    *       运动指令异步执行, 如发生执行错误, 需通过lastErrorCode()获取错误码。
    * @tparam Command 运动指令类: MoveJCommand | MoveAbsJCommand | MoveLCommand | MoveCCommand
    * @param[in] cmds 指令列表, 允许的个数为1-1000
    * @param[out] ec 错误码, 仅反馈执行前的错误, 包括:
    *                1) 网络连接问题; 2) 指令个数不符; 3) 机器人当前状态下无法运动，例如没有上电
    */
   template<class Command>
   void executeCommand(std::vector<Command> cmds, error_code &ec) noexcept;

   /**
   * @brief 执行单条或多条运动指令，调用后机器人立刻开始运动
   * @note 须为同类型的指令;
   *       运动指令异步执行, 如发生执行错误, 需通过lastErrorCode()获取错误码。
   * @tparam Command 运动指令类: MoveJCommand | MoveAbsJCommand | MoveLCommand | MoveCCommand
   * @param[in] cmds 指令列表, 允许的个数为1-1000
   * @param[out] ec 错误码, 仅反馈执行前的错误, 包括:
    *                1) 网络连接问题; 2) 指令个数不符; 3) 机器人当前状态下无法运动，例如没有上电
   */
   template<class Command>
   void executeCommand(std::initializer_list<Command> cmds, error_code &ec) noexcept;

   /**
    * @brief 将单条或多条运动指令加入到缓存中
    * @note 须为同类型的指令; 添加后机器人不会立刻开始运动，调用executeCommands()之后机器人开始运动。
    *       运动指令异步执行, 如发生执行错误, 需通过lastErrorCode 获取错误码。
    *       参数中传入的ec仅反馈执行前的错误, 如网络通信错误或参数个数不符。
    * @tparam Command 运动指令类: MoveJCommand | MoveAbsJCommand | MoveLCommand | MoveCCommand
    * @param[in] cmds 指令列表, 允许的个数为1-1000
    * @param[out] ec 错误码
    */
   template<class Command>
   [[deprecated("Use executeCommand(cmds) instead")]]
   void append(std::vector<Command> cmds, error_code &ec) noexcept;

   /**
   * @brief 将单条或多条运动指令加入到缓存中
   * @tparam Command 运动指令类: MoveJCommand | MoveAbsJCommand | MoveLCommand | MoveCCommand
   * @param[in] cmds 指令列表, 允许的个数为1-1000
   * @param[out] ec 错误码
   */
   template<class Command>
   [[deprecated("Use executeCommand(cmd) instead")]]
   void append(std::initializer_list<Command> cmds, error_code &ec) noexcept;

   /**
    * @brief 开始执行已加入到缓存中的运动指令
    * @param[in] ec 错误码
    */
   [[deprecated("Use executeCommand(cmd) instead")]]
   void executeCommands(error_code &ec) noexcept;

   /**
    * @brief 设定默认运动速度
    * @note 该数值表示末端最大线速度(单位mm/s), 自动计算对应末端旋转速度及轴速度. 若不设置, 则为v100。
    * @param[in] speed 该接口不对参数进行范围限制。末端线速度的实际有效范围分别是5-4000(协作), 5-7000(工业)。
    *              关节速度百分比划分为5个的范围:
    *                < 100 : 10%
    *            100 ~ 200 : 30%
    *            200 ~ 500 : 50%
    *            500 ~ 800 : 80%
    *                > 800 : 100%
    * @param[out] ec 错误码
    */
   void setDefaultSpeed(int speed, error_code &ec) noexcept;

   /**
    * @brief 设定默认转弯区
    * @note 该数值表示运动最大转弯区半径(单位:mm), 自动计算转弯百分比. 若不设置, 则为0 (fine, 无转弯区)
    * @param[in] zone 该接口不对参数进行范围限制。转弯区半径大小实际有效范围是0-200。
    *             转弯百分比划分4个范围:
    *               < 1 : 0 (fine)
    *            1 ~ 20 : 10%
    *           20 ~ 60 : 30%
    *              > 60 : 100%
    * @param[out] ec 错误码
    */
   void setDefaultZone(int zone, error_code &ec) noexcept;

   /**
    * @brief 开始jog机器人，需要切换到手动操作模式。
    * @note 调用此接口并且机器人开始运动后，无论机器人是否已经自行停止，都必须调用stop()来结束jog操作，否则机器人会一直处于jog的运行状态。
    * @param[in] space jog参考坐标系。工具/工件坐标系使用的是RobotAssist右上角设定的工具/工件。
    * @param[in] rate 速率, 范围 0.01 - 1
    * @param[in] step 步长。单位: 笛卡尔空间-毫米 | 轴空间-度。步长大于0即可，不设置上限，
    *             如果机器人机器人无法继续jog会自行停止运动。
    * @param[in] index 笛卡尔空间 - 0~5分别对应XYZABC | 轴空间 - 关节序号，从0开始计数
    * @param[in] direction 方向，true - 正向 | false - 负向
    * @param[out] ec 错误码
    */
   void startJog(JogOpt::Space space, double rate, double step, unsigned index, bool direction, error_code &ec) noexcept;


   //  --------------    MotionControlMode::NrtRLTask   ------------------
   //  ----------------         RL工程相关指令            -------------------

   /**
    * @brief 查询工控机中RL工程名称及任务
    * @param[out] ec 错误码
    * @return 工程信息列表，若没有创建工程则返回空列表
    */
   std::vector<RLProjectInfo> projectsInfo(error_code &ec) noexcept;

   /**
    * @brief 加载工程
    * @param[in] name 工程名称
    * @param[in] tasks 要运行的任务。该参数必须指定，不能为空，否则无法执行工程。
    * @param[out] ec 错误码
    */
   void loadProject(const std::string &name, const std::vector<std::string> &tasks, error_code &ec) noexcept;

   /**
    * @brief 程序指针跳转到main
    * @param[out] ec 错误码
    */
   void projectPointToMain(error_code &ec) noexcept;

   /**
    * @brief 开始运行当前加载的工程
    * @param[out] ec 错误码
    */
   void runProject(error_code &ec) noexcept;

   /**
    * @brief 暂停运行工程
    * @param[out] ec 错误码
    */
   void pauseProject(error_code &ec) noexcept;

   /**
    * @brief 更改工程的运行速度和循环模式
    * @param[in] rate 运行速率，范围 0.01 - 1
    * @param[in] loop true - 循环执行 | false - 单次执行
    * @param[out] ec 错误码
    */
   void setProjectRunningOpt(double rate, bool loop, error_code &ec) noexcept;

   /**
    * @brief 查询当前加载工程的工具信息
    * @param[out] ec 错误码
    * @return 工具信息列表, 若未加载任何工程或没有创建工具, 则返回默认工具tool0的信息
    */
   std::vector<WorkToolInfo> toolsInfo(std::error_code &ec) noexcept;

   /**
    * @brief 查询当前加载工程的工件信息
    * @param[out] ec 错误码
    * @return 工件信息列表, 若未加载任何工程或没有创建工件, 则返回空vector
    */
   std::vector<WorkToolInfo> wobjsInfo(std::error_code &ec) noexcept;

  protected:
   /**
    * @brief 创建机器人实例并建立连接
    * @param[in] robotIP 机器人IP地址
    * @throw NetworkException 网络连接异常
    */
   explicit BaseRobot(const std::string& robotIP);

   /**
    * @brief 析构Robot对象时会让机器人停止运动
    */
   ~BaseRobot() noexcept;

  public:
   /// @cond DO_NOT_DOCUMENT
   BaseRobot(const BaseRobot&) = delete;
   BaseRobot& operator= (BaseRobot&) = delete;
   /// @endcond

  ROKAE_DECLARE_IMPL
 };

 /**
  * @class Robot_T
  * @brief 机器人模板类
  * @tparam Wt 协作/工业类型
  * @tparam DoF 轴数
  */
 template<WorkType Wt, unsigned short DoF>
 class ROKAE_EXPORT Robot_T : public BaseRobot {

  public:
   /**
    * @brief 创建机器人实例, 并连接机器人
    * @param[in] remoteIP 机器人IP地址
    * @throw NetworkException 网络连接错误
    * @throw ExecutionException 机器人实例与连接机型不符
    */
   explicit Robot_T(const std::string &remoteIP);

   // **********************************************************************
   // ******************    Get robot joint state       ********************
   // ******************        获取机器人关节状态         *********************

   /**
    * @brief 机器人当前轴角度, 单位: \f$[rad]\f$
    * @param[out] ec 错误码
    * @return 长度: \f$ \mathbb{R}^{DoF \times 1} \f$.
    */
   std::array<double, DoF> jointPos(error_code &ec) noexcept;

   /**
    * @brief 机器人当前关节速度, 单位: \f$[\frac{rad}{s}]\f$
    * @param[out] ec 错误码
    * @return 长度: \f$ \mathbb{R}^{DoF \times 1} \f$.
    */
   std::array<double, DoF> jointVel(error_code &ec) noexcept;

   /**
    * @brief 关节力传感器数值, 单位: \f$[Nm]\f$
    * @param[out] ec 错误码
    * @return 长度: \f$ \mathbb{R}^{DoF \times 1} \f$.
    */
   std::array<double, DoF> jointTorque(error_code &ec) noexcept;

   // **********************************************************************
   // ********     Robot model for dynamic/kinematic calculation    ********
   // ********         获取机器人模型类, 用于运动学/动力学计算             ********

   /**
    * @brief 获取模型类
    * @return Model类
    */
   Model_T<DoF> model() noexcept;

 };

 /**
  * @class Cobot
  * @brief 协作机器人模板类, 提供协作机器人支持功能的接口
  * @tparam DoF 轴个数
  */
 template<unsigned short DoF>
 class ROKAE_EXPORT Cobot : public Robot_T<WorkType::collaborative, DoF> {
  public:
   /**
     * @brief 协作机器人
     * @param[in] remoteIP 机器人IP地址
     * @param[in] localIP 本机地址。实时模式下收发交互数据用，可不设置。
     * @throw NetworkException 地址格式错误或网络连接失败
     */
   Cobot(std::string remoteIP, std::string localIP = "");

   /**
    * @brief 析构，若此时机器人在运动则将停止运动
    */
   virtual ~Cobot();

   /**
    * @brief 打开拖动
    * @param[in] space 拖动空间. 轴空间拖动仅支持自由拖拽类型
    * @param[in] type 拖动类型
    * @param[out] ec 错误码
    */
   void enableDrag(DragParameter::Space space, DragParameter::Type type, error_code &ec) noexcept;

   /**
    * @brief 关闭拖动
    * @param[out] ec 错误码
    */
   void disableDrag(error_code &ec) noexcept;

   /**
    * @brief 开始录制路径
    * @param[in] duration 路径的时长，单位:秒，范围1~1800.此时长只做范围检查用，到时后控制器不会停止录制，需要调用stopRecordPath()来停止
    * @param[out] ec 错误码
    */
   void startRecordPath(int duration, error_code &ec) noexcept;

   /**
    * @brief 停止录制路径, 若录制成功(无错误码)则路径数据保存在缓存中
    * @param[out] ec 错误码
    */
   void stopRecordPath(error_code &ec) noexcept;

   /**
    * @brief 取消录制, 缓存的路径数据将被删除
    * @param[out] ec 错误码
    */
   void cancelRecordPath(error_code &ec) noexcept;

   /**
    * @brief 保存录制好的路径
    * @param[in] name 路径名称
    * @param[out] ec 错误码
    * @param[in] saveAs 重命名，可选参数。
    *               如果已录制好一条路径但没有保存，则用该名字保存路径。如果没有未保存的路径，则将已保存的名为"name"的路径重命名为"saveAs"
    */
   void saveRecordPath(const std::string &name, error_code &ec, const std::string &saveAs = "") noexcept;

   /**
    * @brief 运动指令-路径回放
    * @param[in] name 要回放的路径名称
    * @param[in] rate 回放速率, 应小于3.0, 1为路径原始速率。注意当速率大于1时，可能产生驱动器无法跟随错误
    * @param[out] ec 错误码
    */
   void replayPath(const std::string &name, double rate, error_code &ec) noexcept;

   /**
    * @brief 删除已保存的路径
    * @param[in] name 要删除的路径名称
    * @param[out] ec 错误码。若路径不存在，错误码不会被置位
    * @param[in] removeAll 是否删除所有路径, 可选参数, 默认为否
    */
   void removePath(const std::string &name, error_code &ec, bool removeAll = false) noexcept;

   /**
    * @brief 查询已保存的所有路径名称
    * @param[out] ec 错误码
    * @return 名称列表, 若没有路径则返回空列表
    */
   std::vector<std::string> queryPathLists(error_code &ec) noexcept;

   /**
    * @brief 设置xPanel对外供电模式。注：仅部分机型支持xPanel功能，不支持的机型会返回错误码
    * @param[in] opt 模式
    * @param[out] ec 错误码
    */
   void setxPanelVout(xPanelOpt::Vout opt, error_code &ec) noexcept;

   // **********************************************************
   // *****************    实时接口    ***************************

   /**
    * @brief 创建实时运动控制类(RtMotionController)实例，通过此实例指针进行实时模式相关的操作。
    * @note 除非重复调用此接口，客户端内部逻辑不会主动析构返回的对象，
    * 包括但不限于断开和机器人连接disconnectFromRobot()，切换到非实时运动控制模式等，但做上述操作之后再进行实时模式控制会产生异常。
    * @return 控制器对象
    * @throw RealtimeControlException 创建RtMotionControl实例失败，由于网络问题
    * @throw ExecutionException 没有切换到实时运动控制模式
    */
   std::weak_ptr<RtMotionControl<DoF>> getRtMotionController();

   /**
    * @brief 设置发送实时运动指令网络延迟阈值，即RobotAssist - RCI设置界面中的”包丢失阈值“。
    * 请在切换到RtCommand模式前进行设置，否则不生效。
    * @param[in] percent 允许的范围0 - 100
    * @param[out] ec 错误码
    */
   void setRtNetworkTolerance(unsigned percent, error_code &ec);

   /**
    * @brief 兼容RCI客户端设置的接口。通过SDK设置运动控制模式为实时模式之后，无法再使用原RCI客户端控制机器人。
    * 若有使用原版的需求，可在切换到非实时模式后，调用此接口。然后再在RobotAssist上打开RCI功能，即可使用RCI客户端。
    * @param[in] use true - 切换到使用第一代
    * @param[out] ec 错误码
    */
   void useRciClient(bool use, error_code &ec);

#ifdef XMATEMODEL_LIB_SUPPORTED
   /**
    * @brief 获取xMate模型类
    * @throw ExecutionException 从控制器读取模型参数失败
    */
   XMateModel<DoF> model();
#endif

  ROKAE_DECLARE_IMPLD
 };

 /**
  * @class IndustrialRobot
  * @brief 工业机器人模板类
  * @tparam DoF 轴数
  */
 template<unsigned short DoF>
 class ROKAE_EXPORT IndustrialRobot: public Robot_T<WorkType::industrial,DoF> {
  public:
   // 使用基类构造
   using Robot_T<WorkType::industrial,DoF>::Robot_T;
 };

 // ***********************************************************************
 // ***************      Robot classes for instantiate      ***************
 // ***************           可实例化的机器人类             ****************

 /**
  * @class XMateRobot
  * @brief 6轴协作机器人, 包括 xMateCR7/12, xMateSR3/4, xMateER3/7
  */
 class ROKAE_EXPORT XMateRobot : public Cobot<6> {
  public:
   // 使用基类构造
   using Cobot<6>::Cobot;
 };

 /**
  * @class XMateErProRobot
  * @brief 7轴协作机器人, 包括 xMateER3 Pro / xMateER7 Pro
  */
 class ROKAE_EXPORT XMateErProRobot : public Cobot<7> {
  public:
   // 使用基类构造
   using Cobot<7>::Cobot;
 };

 /**
  * @class StandardRobot
  * @brief 标准工业6轴机型
  */
 class ROKAE_EXPORT StandardRobot: public IndustrialRobot<6> {
  public:
   // 使用基类构造
   using IndustrialRobot<6>::IndustrialRobot;
 };

 /**
  * @class PCB3Robot
  * @brief PCB3轴机型
  */
 class ROKAE_EXPORT PCB3Robot: public IndustrialRobot<3> {
  public:
   using IndustrialRobot<3>::IndustrialRobot;
 };

 /**
  * @class PCB4Robot
  * @brief PCB4轴机型
  */
 class ROKAE_EXPORT PCB4Robot: public IndustrialRobot<4> {
  public:
   using IndustrialRobot<4>::IndustrialRobot;
 };

}  // namespace rokae

#endif // ROKAEAPI_ROBOT_H_
