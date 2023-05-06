// Generated by gencpp from file zed_interfaces/PlaneStamped.msg
// DO NOT EDIT!


#ifndef ZED_INTERFACES_MESSAGE_PLANESTAMPED_H
#define ZED_INTERFACES_MESSAGE_PLANESTAMPED_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <shape_msgs/Mesh.h>
#include <shape_msgs/Plane.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Polygon.h>

namespace zed_interfaces
{
template <class ContainerAllocator>
struct PlaneStamped_
{
  typedef PlaneStamped_<ContainerAllocator> Type;

  PlaneStamped_()
    : header()
    , mesh()
    , coefficients()
    , normal()
    , center()
    , pose()
    , extents()
    , bounds()  {
      extents.assign(0.0);
  }
  PlaneStamped_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , mesh(_alloc)
    , coefficients(_alloc)
    , normal(_alloc)
    , center(_alloc)
    , pose(_alloc)
    , extents()
    , bounds(_alloc)  {
  (void)_alloc;
      extents.assign(0.0);
  }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::shape_msgs::Mesh_<ContainerAllocator>  _mesh_type;
  _mesh_type mesh;

   typedef  ::shape_msgs::Plane_<ContainerAllocator>  _coefficients_type;
  _coefficients_type coefficients;

   typedef  ::geometry_msgs::Point32_<ContainerAllocator>  _normal_type;
  _normal_type normal;

   typedef  ::geometry_msgs::Point32_<ContainerAllocator>  _center_type;
  _center_type center;

   typedef  ::geometry_msgs::Transform_<ContainerAllocator>  _pose_type;
  _pose_type pose;

   typedef boost::array<float, 2>  _extents_type;
  _extents_type extents;

   typedef  ::geometry_msgs::Polygon_<ContainerAllocator>  _bounds_type;
  _bounds_type bounds;





  typedef boost::shared_ptr< ::zed_interfaces::PlaneStamped_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::zed_interfaces::PlaneStamped_<ContainerAllocator> const> ConstPtr;

}; // struct PlaneStamped_

typedef ::zed_interfaces::PlaneStamped_<std::allocator<void> > PlaneStamped;

typedef boost::shared_ptr< ::zed_interfaces::PlaneStamped > PlaneStampedPtr;
typedef boost::shared_ptr< ::zed_interfaces::PlaneStamped const> PlaneStampedConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::zed_interfaces::PlaneStamped_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::zed_interfaces::PlaneStamped_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::zed_interfaces::PlaneStamped_<ContainerAllocator1> & lhs, const ::zed_interfaces::PlaneStamped_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.mesh == rhs.mesh &&
    lhs.coefficients == rhs.coefficients &&
    lhs.normal == rhs.normal &&
    lhs.center == rhs.center &&
    lhs.pose == rhs.pose &&
    lhs.extents == rhs.extents &&
    lhs.bounds == rhs.bounds;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::zed_interfaces::PlaneStamped_<ContainerAllocator1> & lhs, const ::zed_interfaces::PlaneStamped_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace zed_interfaces

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::zed_interfaces::PlaneStamped_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::zed_interfaces::PlaneStamped_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::zed_interfaces::PlaneStamped_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::zed_interfaces::PlaneStamped_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::zed_interfaces::PlaneStamped_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::zed_interfaces::PlaneStamped_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::zed_interfaces::PlaneStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1ae4cda68097919aea83add5dc8f86c1";
  }

  static const char* value(const ::zed_interfaces::PlaneStamped_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1ae4cda68097919aULL;
  static const uint64_t static_value2 = 0xea83add5dc8f86c1ULL;
};

template<class ContainerAllocator>
struct DataType< ::zed_interfaces::PlaneStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "zed_interfaces/PlaneStamped";
  }

  static const char* value(const ::zed_interfaces::PlaneStamped_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::zed_interfaces::PlaneStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Standard Header\n"
"std_msgs/Header header\n"
"\n"
"# Mesh of the place\n"
"shape_msgs/Mesh mesh\n"
"\n"
"# Representation of a plane, using the plane equation ax + by + cz + d = 0\n"
"shape_msgs/Plane coefficients\n"
"\n"
"# Normal vector\n"
"geometry_msgs/Point32 normal\n"
"\n"
"# Center point \n"
"geometry_msgs/Point32 center\n"
"\n"
"# Plane pose relative to the global reference frame\n"
"geometry_msgs/Transform pose\n"
"\n"
"# Width and height of the bounding rectangle around the plane contours\n"
"float32[2] extents\n"
"\n"
"# The polygon bounds of the plane\n"
"geometry_msgs/Polygon bounds\n"
"\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: shape_msgs/Mesh\n"
"# Definition of a mesh\n"
"\n"
"# list of triangles; the index values refer to positions in vertices[]\n"
"MeshTriangle[] triangles\n"
"\n"
"# the actual vertices that make up the mesh\n"
"geometry_msgs/Point[] vertices\n"
"\n"
"================================================================================\n"
"MSG: shape_msgs/MeshTriangle\n"
"# Definition of a triangle's vertices\n"
"uint32[3] vertex_indices\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: shape_msgs/Plane\n"
"# Representation of a plane, using the plane equation ax + by + cz + d = 0\n"
"\n"
"# a := coef[0]\n"
"# b := coef[1]\n"
"# c := coef[2]\n"
"# d := coef[3]\n"
"\n"
"float64[4] coef\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point32\n"
"# This contains the position of a point in free space(with 32 bits of precision).\n"
"# It is recommeded to use Point wherever possible instead of Point32.  \n"
"# \n"
"# This recommendation is to promote interoperability.  \n"
"#\n"
"# This message is designed to take up less space when sending\n"
"# lots of points at once, as in the case of a PointCloud.  \n"
"\n"
"float32 x\n"
"float32 y\n"
"float32 z\n"
"================================================================================\n"
"MSG: geometry_msgs/Transform\n"
"# This represents the transform between two coordinate frames in free space.\n"
"\n"
"Vector3 translation\n"
"Quaternion rotation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Polygon\n"
"#A specification of a polygon where the first and last points are assumed to be connected\n"
"Point32[] points\n"
;
  }

  static const char* value(const ::zed_interfaces::PlaneStamped_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::zed_interfaces::PlaneStamped_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.mesh);
      stream.next(m.coefficients);
      stream.next(m.normal);
      stream.next(m.center);
      stream.next(m.pose);
      stream.next(m.extents);
      stream.next(m.bounds);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PlaneStamped_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::zed_interfaces::PlaneStamped_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::zed_interfaces::PlaneStamped_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "mesh: ";
    s << std::endl;
    Printer< ::shape_msgs::Mesh_<ContainerAllocator> >::stream(s, indent + "  ", v.mesh);
    s << indent << "coefficients: ";
    s << std::endl;
    Printer< ::shape_msgs::Plane_<ContainerAllocator> >::stream(s, indent + "  ", v.coefficients);
    s << indent << "normal: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point32_<ContainerAllocator> >::stream(s, indent + "  ", v.normal);
    s << indent << "center: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point32_<ContainerAllocator> >::stream(s, indent + "  ", v.center);
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::Transform_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
    s << indent << "extents[]" << std::endl;
    for (size_t i = 0; i < v.extents.size(); ++i)
    {
      s << indent << "  extents[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.extents[i]);
    }
    s << indent << "bounds: ";
    s << std::endl;
    Printer< ::geometry_msgs::Polygon_<ContainerAllocator> >::stream(s, indent + "  ", v.bounds);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ZED_INTERFACES_MESSAGE_PLANESTAMPED_H
