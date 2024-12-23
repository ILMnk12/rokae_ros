
// ROS interface
#include <ros/ros.h>
#include <urdf/model.h>

#include <realtime_tools/realtime_publisher.h>

// ROS control interface
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_manager/controller_manager.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <iostream>
// Rokae sdk
#include <rokae/robot.h>
#include <rokae/data_types.h>
#include "rokae_msgs/ExternalForce.h"
#include "stdlib.h"

namespace rokae_hardware
{
    template <unsigned short DoF>
    class RokaeHardwareInterface : public hardware_interface::RobotHW
    {
    public:
        RokaeHardwareInterface() {
        }

        ~RokaeHardwareInterface()
        {
            robot_->setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
        }

        bool init(ros::NodeHandle &root_nh)
        {   
            setlocale(LC_ALL,"");
            // States initialization
            joint_position_state_.resize(num_joints_, 0.0);
            joint_velocity_state_.resize(num_joints_, 0.0);
            joint_torque_state_.resize(num_joints_, 0.0);

            // External force
            ext_force_in_stiff_.resize(6, 0.0);
            ext_force_in_base_.resize(6, 0.0);

            // Controller
            joint_position_controller_running_ = false;
            joint_velocity_controller_running_ = false;
            joint_torque_controller_running_ = false;
            controllers_initialized_ = false;

            // Commands
            joint_position_command_.resize(num_joints_, 0.0);
            joint_velocity_command_.resize(num_joints_, 0.0);
            joint_torque_command_.resize(num_joints_, 0.0);
            internal_joint_position_command_.resize(num_joints_, 0.0);

            if (!initParameters(root_nh))
            {
                ROS_ERROR("Failed to parse all required parameters.");
                return false;
            }

            if (!initROSInterface())
            {
                ROS_ERROR("Failed to ROS interface.");
                return false;
            }

            if (!initRobot())
            {
                ROS_ERROR("Failed to initialize robot.");
                return false;
            }

            // Publisher
            ext_force_in_stiff_pub_.reset(
                new realtime_tools::RealtimePublisher<rokae_msgs::ExternalForce>(
                    root_nh, "external_force_in_stiff", 1));
            ext_force_in_base_pub_.reset(
                new realtime_tools::RealtimePublisher<rokae_msgs::ExternalForce>(
                    root_nh, "external_force_in_base", 1));
            ROS_WARN("Controller init successful!");

            return true;
        }

        bool initParameters(ros::NodeHandle &root_nh)
        {
            // Get names of the joints given in the controller config file from param server
            if (!root_nh.getParam("/rokae_hardware_interface/joints", joint_names_) || joint_names_.size() != num_joints_)
            {
                ROS_ERROR("No joints names found on param server");

                return false;
            }

            // Get the robot_ip from the param server.
            if (!root_nh.getParam("/rokae_hardware/robot_ip", robot_ip_))
            {
                ROS_ERROR("No robot_ip found on param server");
                return false;
            }

            // Get the local_ip from the param server.
            if (!root_nh.getParam("/rokae_hardware/local_ip", local_ip_))
            {
                ROS_ERROR("No local_ip found on param server");
                return false;
            }

            return true;
        }

        bool initRobot()
        {
            // Instantiation.
            try
            {
                // robot_ = std::make_shared<rokae::Cobot<DoF>>(robot_ip_, local_ip_);
                robot_=std::make_shared<rokae::xMateRobot>(robot_ip_,local_ip_);


            }
            catch (const rokae::NetworkException &e)
            {
                ROS_ERROR("Robot instantiation failed: %s", e.what());
                return false;
            }

            // Connect to robot.
            try
            {
                robot_->connectToRobot(ec);
                // ROS_INFO_STREAM("ec:"<< ec.value());
                if (ec.value() != 0)
                    throw rokae::ExecutionException("Connection failed", "连接失败");
            }
            catch (const rokae::ExecutionException &e)
            {
                ROS_ERROR("Robot connection failed: %s", e.what());
                return false;
            }

            // Enable the robot.
            try
            {
                robot_->setOperateMode(rokae::OperateMode::automatic, ec);
                robot_->setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
                robot_->setPowerState(true, ec);
                // robot_->setRtNetworkTolerance(50, ec);
                ROS_INFO_STREAM("ec:" << ec.value());
                if (ec.value() != 0)
                    throw rokae::ExecutionException("Enable failed", "上电失败");
            }
            catch (const rokae::ExecutionException &e)
            {
                ROS_ERROR("Could not enable robot: %s", e.what());
                return false;
            }

            // Save initial position.
            setInitPosition();

            // Set realtime control.
            try
            {
                ROS_INFO_STREAM("RCI setting...");
                robot_->setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
                rci_ = robot_->getRtMotionController().lock();
                ROS_INFO_STREAM("getting RCI");

                rci_->startReceiveRobotState({rokae::RtSupportedFields::jointPos_m, rokae::RtSupportedFields::jointVel_m,
                                              rokae::RtSupportedFields::tau_m, rokae::RtSupportedFields::tauExt_inBase,
                                              rokae::RtSupportedFields::tauExt_inStiff});


                ROS_INFO_STREAM("RCI set sucessfully");
            }
            catch (const rokae::ExecutionException &e)
            {
                ROS_ERROR("Could not set realtime control mode: %s", e.what());
                return false;
            }

            // PVI
            // std::function<rokae::JointPosition()> callback2  = std::bind(&RokaeHardwareInterface::callback, this);
            // rci_->setControlLoop(callback2,0,true);
            // rci_->startMove(rokae::RtControllerMode::jointPosition);
            //     ROS_INFO_STREAM("PVI2");
            // rci_->startLoop(false);
            //     ROS_INFO_STREAM("PVI3");

            // ROS_INFO_STREAM("Robot is ready for operation!");

            return true;
        }

        void setInitPosition()
        {
            // Get joint position
            auto robot_joint_postion = robot_->jointPos(ec);
            for (std::size_t i = 0; i < num_joints_; i++)
            {
                joint_position_state_[i] = robot_joint_postion[i];
            }

            // Set the initial joint position as command
            joint_position_command_ = joint_position_state_;
        }

        bool initROSInterface()
        {
            for (std::size_t i = 0; i < num_joints_; i++)
            {
                try
                {
                    // Create joint state interface for all joints
                    joint_state_interface_.registerHandle(
                        hardware_interface::JointStateHandle(joint_names_[i], &joint_position_state_[i],
                                                             &joint_velocity_state_[i], &joint_torque_state_[i]));

                    // Create joint position interface
                    hardware_interface::JointHandle joint_handle_position = hardware_interface::JointHandle(joint_state_interface_.getHandle(
                                                                                                                joint_names_[i]),
                                                                                                            &joint_position_command_[i]);
                    joint_position_interface_.registerHandle(joint_handle_position);

                    // Create velocity  interface
                    hardware_interface::JointHandle joint_handle_velocity = hardware_interface::JointHandle(joint_state_interface_.getHandle(
                                                                                                                joint_names_[i]),
                                                                                                            &joint_velocity_command_[i]);
                    joint_velocity_interface_.registerHandle(joint_handle_velocity);

                    // Create joint torque interface
                    hardware_interface::JointHandle joint_handle_torque = hardware_interface::JointHandle(joint_state_interface_.getHandle(
                                                                                                              joint_names_[i]),
                                                                                                          &joint_torque_command_[i]);
                    joint_torque_interface_.registerHandle(joint_handle_torque);
                }
                catch (const hardware_interface::HardwareInterfaceException &e)
                {
                    std::cerr << e.what() << '\n';
                }
            }

            // Register interface
            registerInterface(&joint_state_interface_);
            registerInterface(&joint_position_interface_);
            registerInterface(&joint_velocity_interface_);
            registerInterface(&joint_torque_interface_);

            return true;
        }

        void publishExternalForce()
        {
            if (ext_force_in_stiff_pub_)
            {
                if (ext_force_in_stiff_pub_->trylock())
                {
                    ext_force_in_stiff_pub_->msg_.force.x = ext_force_in_stiff_[0];
                    ext_force_in_stiff_pub_->msg_.force.y = ext_force_in_stiff_[1];
                    ext_force_in_stiff_pub_->msg_.force.z = ext_force_in_stiff_[2];
                    ext_force_in_stiff_pub_->msg_.moment.x = ext_force_in_stiff_[3];
                    ext_force_in_stiff_pub_->msg_.moment.y = ext_force_in_stiff_[4];
                    ext_force_in_stiff_pub_->msg_.moment.z = ext_force_in_stiff_[5];
                    ext_force_in_stiff_pub_->unlockAndPublish();
                }
            }
            if (ext_force_in_base_pub_)
            {
                if (ext_force_in_base_pub_->trylock())
                {
                    ext_force_in_base_pub_->msg_.force.x = ext_force_in_base_[0];
                    ext_force_in_base_pub_->msg_.force.y = ext_force_in_base_[1];
                    ext_force_in_base_pub_->msg_.force.z = ext_force_in_base_[2];
                    ext_force_in_base_pub_->msg_.moment.x = ext_force_in_base_[3];
                    ext_force_in_base_pub_->msg_.moment.y = ext_force_in_base_[4];
                    ext_force_in_base_pub_->msg_.moment.z = ext_force_in_base_[5];
                    ext_force_in_base_pub_->unlockAndPublish();
                }
            }
        }

        void read()
        {
            // update robot state
            rci_->updateRobotState();
            std::array<double, num_joints_> robot_joint_postion;
            std::array<double, num_joints_> robot_joint_velocity;
            std::array<double, num_joints_> robot_joint_torque;
            // fixed size
            std::array<double, 6> robot_ext_force_base;
            std::array<double, 6> robot_ext_force_stiff;

            // Get joint position, velocity and torque.
            int pos_flag = rci_->getStateData("q_m", robot_joint_postion);

            int vel_flag = rci_->getStateData(rokae::RtSupportedFields::jointVel_m, robot_joint_velocity);
            int tor_flag = rci_->getStateData("tau_m", robot_joint_torque);
            int ext_base = rci_->getStateData(rokae::RtSupportedFields::tauExt_inBase, robot_ext_force_base);
            int ext_stiff = rci_->getStateData(rokae::RtSupportedFields::tauExt_inStiff, robot_ext_force_stiff);

            for (int i = 0; i < num_joints_; i++)
            {
                joint_position_state_[i] = robot_joint_postion[i];
                joint_velocity_state_[i] = robot_joint_velocity[i];
                joint_torque_state_[i] = robot_joint_torque[i];
            }
            for (int i = 0; i < 6; i++)
            {
                ext_force_in_base_[i] = robot_ext_force_base[i];
                ext_force_in_stiff_[i] = robot_ext_force_stiff[i];
            }

            internal_joint_position_command_ = joint_position_state_;

            // publishExternalForce();
        }

        void write(ros::Duration period)
        {
            // ros::Duration period(0.001);
            // std::array<double, 7> now_command_{};

            if (joint_position_controller_running_)
            {       if(times_loop_==0){

                        sleep(5);
                        times_loop_++;
                    }
                    rci_->updateRobotState();
                    std::function<rokae::JointPosition()> callback2 = std::bind(&RokaeHardwareInterface::callback, this);
                    rokae::JointPosition joints_value=callback2();
                    rci_->setControlLoop(callback2, 0, true);
                    // ROS_INFO_STREAM("PVI2");
                    rci_->startLoop(false); 
                    // rci_->stopMove();
           
            }
            else if (joint_velocity_controller_running_)
            {
                for (std::size_t i = 0; i < num_joints_; i++)
                {
                    internal_joint_position_command_[i] += joint_velocity_command_[i] * period.toSec();
                }
               
            }
            else if (joint_torque_controller_running_)
            {
                // rci_->appendCommand(rokae::Torque(joint_torque_command_));
            }
            else
            {
                // ROS_INFO_STREAM("ERROR IN WRITE!");
                return;
            }
        }

        void stop()
        {
            robot_->setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
            rci_->stopMove();

        }

        bool prepareSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                           const std::list<hardware_interface::ControllerInfo> & /*stop_list*/)
        {

            if (controllers_initialized_ && !start_list.empty())
            {
                for (auto &controller : start_list)
                {
                    std::cout << "namne is: " << controller.name << std::endl;
                    if (!controller.claimed_resources.empty())
                    {
                        ROS_ERROR("Resources have been claimed by another controller!!");
                        return false;
                    }
                }
            }
            controllers_initialized_ = true;
            ROS_INFO("Controllers are now initialized.");
            return true;
        }

        bool checkControllerClaims(const std::set<std::string> &claimed_resources)
        {
            for (const std::string &it : joint_names_)
            {
                for (const std::string &resource : claimed_resources)
                {
                    if (it == resource)
                    {
                        return true;
                    }
                }
            }

            return false;
        }

        void doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                      const std::list<hardware_interface::ControllerInfo> &stop_list)
        {
            for (auto &controller_it : stop_list)
            {
                for (auto &resource_it : controller_it.claimed_resources)
                {
                    if (checkControllerClaims(resource_it.resources))
                    {
                        if (resource_it.hardware_interface == "hardware_interface::PositionJointInterface")
                        {
                            joint_position_controller_running_ = false;
                        }
                        else if (resource_it.hardware_interface == "hardware_interface::VelocityJointInterface")
                        {
                            joint_velocity_controller_running_ = false;
                        }
                        else if (resource_it.hardware_interface == "hardware_interface::EffortJointInterface")
                        {
                            joint_torque_controller_running_ = false;
                        }
                    }
                }
            }

            for (auto &controller_it : start_list)
            {
                std::cout << "resource controllers check" << std::endl;
                for (auto &resource_it : controller_it.claimed_resources)
                {
                    if (checkControllerClaims(resource_it.resources))
                    {
                        if (resource_it.hardware_interface == "hardware_interface::PositionJointInterface")
                        {
                            joint_velocity_controller_running_ = false;
                            joint_torque_controller_running_ = false;
                            joint_position_controller_running_ = true;
                            if (joint_position_controller_running_)
                            {
                                // rci_->stopMove();
                                rci_->startMove(rokae::RtControllerMode::jointPosition);

                            }
                        }
                        else if (resource_it.hardware_interface == "hardware_interface::VelocityJointInterface")
                        {
                            joint_position_controller_running_ = false;
                            joint_torque_controller_running_ = false;
                            joint_velocity_controller_running_ = true;
                            if (joint_velocity_controller_running_)
                            {
                                //rci_->stopMove();
                                rci_->startMove(rokae::RtControllerMode::jointPosition);
                            }
                        }
                        else if (resource_it.hardware_interface == "hardware_interface::EffortJointInterface")
                        {
                            joint_position_controller_running_ = false;
                            joint_velocity_controller_running_ = false;
                            joint_torque_controller_running_ = true;
                            if (joint_torque_controller_running_)
                            {
                                rci_->stopMove();
                                rci_->startMove(rokae::RtControllerMode::torque);
                            }
                        }
                    }
                }
            }
        }

        rokae::JointPosition callback() 
        {

            // times_loop_ ++;
            // ROS_INFO_STREAM(times_loop_);
            rci_->updateRobotState();
            rokae::JointPosition jcmd(num_joints_);
            for (size_t i = 0; i < num_joints_; i++)
            {
                jcmd.joints[i] = joint_position_command_[i];

            }
           
            this->time_us2 = time_us1;
            this->time_us1 = getLocalTimeUs();
            
            usleep(50);
            //   ROS_INFO("time");
            //  ROS_INFO_STREAM(time_us1 - time_us2);/

            return jcmd;
        }

    public:
        // Hardware interface
        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::PositionJointInterface joint_position_interface_;
        hardware_interface::VelocityJointInterface joint_velocity_interface_;
        hardware_interface::EffortJointInterface joint_torque_interface_;

        // Configuration
        // std::shared_ptr<rokae::Cobot<DoF>> robot_;
        std::shared_ptr<rokae::xMateRobot> robot_;
        std::string robot_ip_;
        std::string local_ip_;
        std::error_code ec;
        static const size_t num_joints_ = DoF;
        std::vector<std::string> joint_names_;

        std::shared_ptr<rokae::RtMotionControl<rokae::WorkType::collaborative,DoF>> rci_;
        // std::shared_ptr<rokae::RtMotionControlCobot<(unsigned short)6U>> rci_;

        // template<unsigned short DoF> template<class Command> 
        // void rokae::RtMotionControl<DoF>::appendCommand(Command cmd)
        // States
        std::vector<double> joint_position_state_;
        std::vector<double> joint_velocity_state_;
        std::vector<double> joint_torque_state_;

        // External force
        std::vector<double> ext_force_in_stiff_;
        std::vector<double> ext_force_in_base_;

        // Commands
        std::vector<double> joint_position_command_;
        std::vector<double> joint_velocity_command_;
        std::vector<double> joint_torque_command_;
        std::vector<double> internal_joint_position_command_;

        // Controller
        bool joint_position_controller_running_;
        bool joint_velocity_controller_running_;
        bool joint_torque_controller_running_;
        bool controllers_initialized_;
        long times_loop_ = 0;
        // Publisher
        std::unique_ptr<realtime_tools::RealtimePublisher<rokae_msgs::ExternalForce>>
            ext_force_in_stiff_pub_;
        std::unique_ptr<realtime_tools::RealtimePublisher<rokae_msgs::ExternalForce>>
            ext_force_in_base_pub_;

            // pvi
            long long time_us1 = 0; 
            long long time_us2 = 0;
            unsigned long long getLocalTimeUs() // us
            {
                typedef std::chrono::time_point<std::chrono::system_clock, std::chrono::microseconds> microClock_type;
                // 获取当前时间点，windows system_clock是100纳秒级别的(不同系统不一样，自己按照介绍的方法测试)，所以要转换
                microClock_type tp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::system_clock::now());
                return tp.time_since_epoch().count();
            }
    };

}
