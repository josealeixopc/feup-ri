// Generated by gencpp from file openai_ros/RLExperimentInfo.msg
// DO NOT EDIT!


#ifndef OPENAI_ROS_MESSAGE_RLEXPERIMENTINFO_H
#define OPENAI_ROS_MESSAGE_RLEXPERIMENTINFO_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace openai_ros
{
template <class ContainerAllocator>
struct RLExperimentInfo_
{
  typedef RLExperimentInfo_<ContainerAllocator> Type;

  RLExperimentInfo_()
    : episode_number(0)
    , episode_reward(0.0)  {
    }
  RLExperimentInfo_(const ContainerAllocator& _alloc)
    : episode_number(0)
    , episode_reward(0.0)  {
  (void)_alloc;
    }



   typedef int32_t _episode_number_type;
  _episode_number_type episode_number;

   typedef float _episode_reward_type;
  _episode_reward_type episode_reward;





  typedef boost::shared_ptr< ::openai_ros::RLExperimentInfo_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::openai_ros::RLExperimentInfo_<ContainerAllocator> const> ConstPtr;

}; // struct RLExperimentInfo_

typedef ::openai_ros::RLExperimentInfo_<std::allocator<void> > RLExperimentInfo;

typedef boost::shared_ptr< ::openai_ros::RLExperimentInfo > RLExperimentInfoPtr;
typedef boost::shared_ptr< ::openai_ros::RLExperimentInfo const> RLExperimentInfoConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::openai_ros::RLExperimentInfo_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::openai_ros::RLExperimentInfo_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace openai_ros

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'openai_ros': ['/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/src/external-packages/openai_ros/openai_ros/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::openai_ros::RLExperimentInfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::openai_ros::RLExperimentInfo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::openai_ros::RLExperimentInfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::openai_ros::RLExperimentInfo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::openai_ros::RLExperimentInfo_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::openai_ros::RLExperimentInfo_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::openai_ros::RLExperimentInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "117729220546455cc216a7df0b6e91d0";
  }

  static const char* value(const ::openai_ros::RLExperimentInfo_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x117729220546455cULL;
  static const uint64_t static_value2 = 0xc216a7df0b6e91d0ULL;
};

template<class ContainerAllocator>
struct DataType< ::openai_ros::RLExperimentInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "openai_ros/RLExperimentInfo";
  }

  static const char* value(const ::openai_ros::RLExperimentInfo_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::openai_ros::RLExperimentInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 episode_number\n"
"float32 episode_reward\n"
;
  }

  static const char* value(const ::openai_ros::RLExperimentInfo_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::openai_ros::RLExperimentInfo_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.episode_number);
      stream.next(m.episode_reward);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RLExperimentInfo_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::openai_ros::RLExperimentInfo_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::openai_ros::RLExperimentInfo_<ContainerAllocator>& v)
  {
    s << indent << "episode_number: ";
    Printer<int32_t>::stream(s, indent + "  ", v.episode_number);
    s << indent << "episode_reward: ";
    Printer<float>::stream(s, indent + "  ", v.episode_reward);
  }
};

} // namespace message_operations
} // namespace ros

#endif // OPENAI_ROS_MESSAGE_RLEXPERIMENTINFO_H
