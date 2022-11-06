// Generated by gencpp from file kinova_msgs/StopResponse.msg
// DO NOT EDIT!


#ifndef KINOVA_MSGS_MESSAGE_STOPRESPONSE_H
#define KINOVA_MSGS_MESSAGE_STOPRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace kinova_msgs
{
template <class ContainerAllocator>
struct StopResponse_
{
  typedef StopResponse_<ContainerAllocator> Type;

  StopResponse_()
    : stop_result()  {
    }
  StopResponse_(const ContainerAllocator& _alloc)
    : stop_result(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _stop_result_type;
  _stop_result_type stop_result;





  typedef boost::shared_ptr< ::kinova_msgs::StopResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kinova_msgs::StopResponse_<ContainerAllocator> const> ConstPtr;

}; // struct StopResponse_

typedef ::kinova_msgs::StopResponse_<std::allocator<void> > StopResponse;

typedef boost::shared_ptr< ::kinova_msgs::StopResponse > StopResponsePtr;
typedef boost::shared_ptr< ::kinova_msgs::StopResponse const> StopResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kinova_msgs::StopResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kinova_msgs::StopResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kinova_msgs::StopResponse_<ContainerAllocator1> & lhs, const ::kinova_msgs::StopResponse_<ContainerAllocator2> & rhs)
{
  return lhs.stop_result == rhs.stop_result;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kinova_msgs::StopResponse_<ContainerAllocator1> & lhs, const ::kinova_msgs::StopResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kinova_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kinova_msgs::StopResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kinova_msgs::StopResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kinova_msgs::StopResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kinova_msgs::StopResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kinova_msgs::StopResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kinova_msgs::StopResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kinova_msgs::StopResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "585dc4164508d473dff8f8b67a05d2ad";
  }

  static const char* value(const ::kinova_msgs::StopResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x585dc4164508d473ULL;
  static const uint64_t static_value2 = 0xdff8f8b67a05d2adULL;
};

template<class ContainerAllocator>
struct DataType< ::kinova_msgs::StopResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kinova_msgs/StopResponse";
  }

  static const char* value(const ::kinova_msgs::StopResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kinova_msgs::StopResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string stop_result\n"
"\n"
;
  }

  static const char* value(const ::kinova_msgs::StopResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kinova_msgs::StopResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.stop_result);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct StopResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kinova_msgs::StopResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kinova_msgs::StopResponse_<ContainerAllocator>& v)
  {
    s << indent << "stop_result: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.stop_result);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KINOVA_MSGS_MESSAGE_STOPRESPONSE_H
