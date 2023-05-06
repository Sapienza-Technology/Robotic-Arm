// Generated by gencpp from file braccio_urdf_description/Float6Array.msg
// DO NOT EDIT!


#ifndef BRACCIO_URDF_DESCRIPTION_MESSAGE_FLOAT6ARRAY_H
#define BRACCIO_URDF_DESCRIPTION_MESSAGE_FLOAT6ARRAY_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace braccio_urdf_description
{
template <class ContainerAllocator>
struct Float6Array_
{
  typedef Float6Array_<ContainerAllocator> Type;

  Float6Array_()
    : x1(0.0)
    , x2(0.0)
    , x3(0.0)
    , x4(0.0)
    , x5(0.0)
    , x6(0.0)  {
    }
  Float6Array_(const ContainerAllocator& _alloc)
    : x1(0.0)
    , x2(0.0)
    , x3(0.0)
    , x4(0.0)
    , x5(0.0)
    , x6(0.0)  {
  (void)_alloc;
    }



   typedef double _x1_type;
  _x1_type x1;

   typedef double _x2_type;
  _x2_type x2;

   typedef double _x3_type;
  _x3_type x3;

   typedef double _x4_type;
  _x4_type x4;

   typedef double _x5_type;
  _x5_type x5;

   typedef double _x6_type;
  _x6_type x6;





  typedef boost::shared_ptr< ::braccio_urdf_description::Float6Array_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::braccio_urdf_description::Float6Array_<ContainerAllocator> const> ConstPtr;

}; // struct Float6Array_

typedef ::braccio_urdf_description::Float6Array_<std::allocator<void> > Float6Array;

typedef boost::shared_ptr< ::braccio_urdf_description::Float6Array > Float6ArrayPtr;
typedef boost::shared_ptr< ::braccio_urdf_description::Float6Array const> Float6ArrayConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::braccio_urdf_description::Float6Array_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::braccio_urdf_description::Float6Array_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::braccio_urdf_description::Float6Array_<ContainerAllocator1> & lhs, const ::braccio_urdf_description::Float6Array_<ContainerAllocator2> & rhs)
{
  return lhs.x1 == rhs.x1 &&
    lhs.x2 == rhs.x2 &&
    lhs.x3 == rhs.x3 &&
    lhs.x4 == rhs.x4 &&
    lhs.x5 == rhs.x5 &&
    lhs.x6 == rhs.x6;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::braccio_urdf_description::Float6Array_<ContainerAllocator1> & lhs, const ::braccio_urdf_description::Float6Array_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace braccio_urdf_description

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::braccio_urdf_description::Float6Array_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::braccio_urdf_description::Float6Array_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::braccio_urdf_description::Float6Array_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::braccio_urdf_description::Float6Array_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::braccio_urdf_description::Float6Array_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::braccio_urdf_description::Float6Array_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::braccio_urdf_description::Float6Array_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bd6ec219eb9feb2baf2bfcaae69e9c5e";
  }

  static const char* value(const ::braccio_urdf_description::Float6Array_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xbd6ec219eb9feb2bULL;
  static const uint64_t static_value2 = 0xaf2bfcaae69e9c5eULL;
};

template<class ContainerAllocator>
struct DataType< ::braccio_urdf_description::Float6Array_<ContainerAllocator> >
{
  static const char* value()
  {
    return "braccio_urdf_description/Float6Array";
  }

  static const char* value(const ::braccio_urdf_description::Float6Array_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::braccio_urdf_description::Float6Array_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 x1\n"
"float64 x2\n"
"float64 x3\n"
"float64 x4\n"
"float64 x5\n"
"float64 x6\n"
;
  }

  static const char* value(const ::braccio_urdf_description::Float6Array_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::braccio_urdf_description::Float6Array_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x1);
      stream.next(m.x2);
      stream.next(m.x3);
      stream.next(m.x4);
      stream.next(m.x5);
      stream.next(m.x6);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Float6Array_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::braccio_urdf_description::Float6Array_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::braccio_urdf_description::Float6Array_<ContainerAllocator>& v)
  {
    s << indent << "x1: ";
    Printer<double>::stream(s, indent + "  ", v.x1);
    s << indent << "x2: ";
    Printer<double>::stream(s, indent + "  ", v.x2);
    s << indent << "x3: ";
    Printer<double>::stream(s, indent + "  ", v.x3);
    s << indent << "x4: ";
    Printer<double>::stream(s, indent + "  ", v.x4);
    s << indent << "x5: ";
    Printer<double>::stream(s, indent + "  ", v.x5);
    s << indent << "x6: ";
    Printer<double>::stream(s, indent + "  ", v.x6);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BRACCIO_URDF_DESCRIPTION_MESSAGE_FLOAT6ARRAY_H
