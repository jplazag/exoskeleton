// Generated by gencpp from file tutorial_2/SetBoolRequest.msg
// DO NOT EDIT!


#ifndef TUTORIAL_2_MESSAGE_SETBOOLREQUEST_H
#define TUTORIAL_2_MESSAGE_SETBOOLREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace tutorial_2
{
template <class ContainerAllocator>
struct SetBoolRequest_
{
  typedef SetBoolRequest_<ContainerAllocator> Type;

  SetBoolRequest_()
    : data(false)  {
    }
  SetBoolRequest_(const ContainerAllocator& _alloc)
    : data(false)  {
  (void)_alloc;
    }



   typedef uint8_t _data_type;
  _data_type data;





  typedef boost::shared_ptr< ::tutorial_2::SetBoolRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::tutorial_2::SetBoolRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SetBoolRequest_

typedef ::tutorial_2::SetBoolRequest_<std::allocator<void> > SetBoolRequest;

typedef boost::shared_ptr< ::tutorial_2::SetBoolRequest > SetBoolRequestPtr;
typedef boost::shared_ptr< ::tutorial_2::SetBoolRequest const> SetBoolRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::tutorial_2::SetBoolRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::tutorial_2::SetBoolRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::tutorial_2::SetBoolRequest_<ContainerAllocator1> & lhs, const ::tutorial_2::SetBoolRequest_<ContainerAllocator2> & rhs)
{
  return lhs.data == rhs.data;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::tutorial_2::SetBoolRequest_<ContainerAllocator1> & lhs, const ::tutorial_2::SetBoolRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace tutorial_2

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::tutorial_2::SetBoolRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::tutorial_2::SetBoolRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::tutorial_2::SetBoolRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::tutorial_2::SetBoolRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tutorial_2::SetBoolRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tutorial_2::SetBoolRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::tutorial_2::SetBoolRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8b94c1b53db61fb6aed406028ad6332a";
  }

  static const char* value(const ::tutorial_2::SetBoolRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8b94c1b53db61fb6ULL;
  static const uint64_t static_value2 = 0xaed406028ad6332aULL;
};

template<class ContainerAllocator>
struct DataType< ::tutorial_2::SetBoolRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "tutorial_2/SetBoolRequest";
  }

  static const char* value(const ::tutorial_2::SetBoolRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::tutorial_2::SetBoolRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool data # e.g. for hardware enabling / disabling\n"
;
  }

  static const char* value(const ::tutorial_2::SetBoolRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::tutorial_2::SetBoolRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.data);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetBoolRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::tutorial_2::SetBoolRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::tutorial_2::SetBoolRequest_<ContainerAllocator>& v)
  {
    s << indent << "data: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.data);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TUTORIAL_2_MESSAGE_SETBOOLREQUEST_H
