// Generated by gencpp from file rokae_msgs/RobotState.msg
// DO NOT EDIT!


#ifndef ROKAE_MSGS_MESSAGE_ROBOTSTATE_H
#define ROKAE_MSGS_MESSAGE_ROBOTSTATE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace rokae_msgs
{
template <class ContainerAllocator>
struct RobotState_
{
  typedef RobotState_<ContainerAllocator> Type;

  RobotState_()
    : timestamp(0.0)
    , joint_pose()
    , joint_velocity()
    , joint_cmd_acceleration()
    , joint_torque()
    , joint_filter_torque()
    , arm_angle(0.0)  {
    }
  RobotState_(const ContainerAllocator& _alloc)
    : timestamp(0.0)
    , joint_pose(_alloc)
    , joint_velocity(_alloc)
    , joint_cmd_acceleration(_alloc)
    , joint_torque(_alloc)
    , joint_filter_torque(_alloc)
    , arm_angle(0.0)  {
  (void)_alloc;
    }



   typedef double _timestamp_type;
  _timestamp_type timestamp;

   typedef std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> _joint_pose_type;
  _joint_pose_type joint_pose;

   typedef std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> _joint_velocity_type;
  _joint_velocity_type joint_velocity;

   typedef std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> _joint_cmd_acceleration_type;
  _joint_cmd_acceleration_type joint_cmd_acceleration;

   typedef std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> _joint_torque_type;
  _joint_torque_type joint_torque;

   typedef std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> _joint_filter_torque_type;
  _joint_filter_torque_type joint_filter_torque;

   typedef double _arm_angle_type;
  _arm_angle_type arm_angle;





  typedef boost::shared_ptr< ::rokae_msgs::RobotState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rokae_msgs::RobotState_<ContainerAllocator> const> ConstPtr;

}; // struct RobotState_

typedef ::rokae_msgs::RobotState_<std::allocator<void> > RobotState;

typedef boost::shared_ptr< ::rokae_msgs::RobotState > RobotStatePtr;
typedef boost::shared_ptr< ::rokae_msgs::RobotState const> RobotStateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rokae_msgs::RobotState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rokae_msgs::RobotState_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::rokae_msgs::RobotState_<ContainerAllocator1> & lhs, const ::rokae_msgs::RobotState_<ContainerAllocator2> & rhs)
{
  return lhs.timestamp == rhs.timestamp &&
    lhs.joint_pose == rhs.joint_pose &&
    lhs.joint_velocity == rhs.joint_velocity &&
    lhs.joint_cmd_acceleration == rhs.joint_cmd_acceleration &&
    lhs.joint_torque == rhs.joint_torque &&
    lhs.joint_filter_torque == rhs.joint_filter_torque &&
    lhs.arm_angle == rhs.arm_angle;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::rokae_msgs::RobotState_<ContainerAllocator1> & lhs, const ::rokae_msgs::RobotState_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace rokae_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::rokae_msgs::RobotState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rokae_msgs::RobotState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rokae_msgs::RobotState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rokae_msgs::RobotState_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rokae_msgs::RobotState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rokae_msgs::RobotState_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rokae_msgs::RobotState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bd80e953e8359840e704f48ebdde5ced";
  }

  static const char* value(const ::rokae_msgs::RobotState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xbd80e953e8359840ULL;
  static const uint64_t static_value2 = 0xe704f48ebdde5cedULL;
};

template<class ContainerAllocator>
struct DataType< ::rokae_msgs::RobotState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rokae_msgs/RobotState";
  }

  static const char* value(const ::rokae_msgs::RobotState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rokae_msgs::RobotState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Data structure for robot status description\n"
"\n"
"# timestamp\n"
"float64 timestamp                    # time stamp   \n"
"\n"
"# joint status\n"
"float64[] joint_pose                 # joint_pose\n"
"float64[] joint_velocity             # joint_velocity\n"
"float64[] joint_cmd_acceleration         # joint_acceleration\n"
"float64[] joint_torque               # joint_torque\n"
"float64[] joint_filter_torque        # joint_filter_torque\n"
"\n"
"# arm angle state\n"
"float64 arm_angle                    # arm angle\n"
;
  }

  static const char* value(const ::rokae_msgs::RobotState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rokae_msgs::RobotState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.timestamp);
      stream.next(m.joint_pose);
      stream.next(m.joint_velocity);
      stream.next(m.joint_cmd_acceleration);
      stream.next(m.joint_torque);
      stream.next(m.joint_filter_torque);
      stream.next(m.arm_angle);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RobotState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rokae_msgs::RobotState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rokae_msgs::RobotState_<ContainerAllocator>& v)
  {
    s << indent << "timestamp: ";
    Printer<double>::stream(s, indent + "  ", v.timestamp);
    s << indent << "joint_pose[]" << std::endl;
    for (size_t i = 0; i < v.joint_pose.size(); ++i)
    {
      s << indent << "  joint_pose[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.joint_pose[i]);
    }
    s << indent << "joint_velocity[]" << std::endl;
    for (size_t i = 0; i < v.joint_velocity.size(); ++i)
    {
      s << indent << "  joint_velocity[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.joint_velocity[i]);
    }
    s << indent << "joint_cmd_acceleration[]" << std::endl;
    for (size_t i = 0; i < v.joint_cmd_acceleration.size(); ++i)
    {
      s << indent << "  joint_cmd_acceleration[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.joint_cmd_acceleration[i]);
    }
    s << indent << "joint_torque[]" << std::endl;
    for (size_t i = 0; i < v.joint_torque.size(); ++i)
    {
      s << indent << "  joint_torque[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.joint_torque[i]);
    }
    s << indent << "joint_filter_torque[]" << std::endl;
    for (size_t i = 0; i < v.joint_filter_torque.size(); ++i)
    {
      s << indent << "  joint_filter_torque[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.joint_filter_torque[i]);
    }
    s << indent << "arm_angle: ";
    Printer<double>::stream(s, indent + "  ", v.arm_angle);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROKAE_MSGS_MESSAGE_ROBOTSTATE_H