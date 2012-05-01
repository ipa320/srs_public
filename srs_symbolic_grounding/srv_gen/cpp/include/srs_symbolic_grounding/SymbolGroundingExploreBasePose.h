/* Auto-generated by genmsg_cpp for file /home/beisheng/git/care-o-bot/srs_public/srs_symbolic_grounding/srv/SymbolGroundingExploreBasePose.srv */
#ifndef SRS_SYMBOLIC_GROUNDING_SERVICE_SYMBOLGROUNDINGEXPLOREBASEPOSE_H
#define SRS_SYMBOLIC_GROUNDING_SERVICE_SYMBOLGROUNDINGEXPLOREBASEPOSE_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "ros/service_traits.h"

#include "srs_symbolic_grounding/SRSFurnitureGeometry.h"
#include "srs_symbolic_grounding/SRSFurnitureGeometry.h"


#include "geometry_msgs/Pose2D.h"

namespace srs_symbolic_grounding
{
template <class ContainerAllocator>
struct SymbolGroundingExploreBasePoseRequest_ {
  typedef SymbolGroundingExploreBasePoseRequest_<ContainerAllocator> Type;

  SymbolGroundingExploreBasePoseRequest_()
  : parent_obj_geometry()
  , furniture_geometry_list()
  {
  }

  SymbolGroundingExploreBasePoseRequest_(const ContainerAllocator& _alloc)
  : parent_obj_geometry(_alloc)
  , furniture_geometry_list(_alloc)
  {
  }

  typedef  ::srs_symbolic_grounding::SRSFurnitureGeometry_<ContainerAllocator>  _parent_obj_geometry_type;
   ::srs_symbolic_grounding::SRSFurnitureGeometry_<ContainerAllocator>  parent_obj_geometry;

  typedef std::vector< ::srs_symbolic_grounding::SRSFurnitureGeometry_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::srs_symbolic_grounding::SRSFurnitureGeometry_<ContainerAllocator> >::other >  _furniture_geometry_list_type;
  std::vector< ::srs_symbolic_grounding::SRSFurnitureGeometry_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::srs_symbolic_grounding::SRSFurnitureGeometry_<ContainerAllocator> >::other >  furniture_geometry_list;


  ROS_DEPRECATED uint32_t get_furniture_geometry_list_size() const { return (uint32_t)furniture_geometry_list.size(); }
  ROS_DEPRECATED void set_furniture_geometry_list_size(uint32_t size) { furniture_geometry_list.resize((size_t)size); }
  ROS_DEPRECATED void get_furniture_geometry_list_vec(std::vector< ::srs_symbolic_grounding::SRSFurnitureGeometry_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::srs_symbolic_grounding::SRSFurnitureGeometry_<ContainerAllocator> >::other > & vec) const { vec = this->furniture_geometry_list; }
  ROS_DEPRECATED void set_furniture_geometry_list_vec(const std::vector< ::srs_symbolic_grounding::SRSFurnitureGeometry_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::srs_symbolic_grounding::SRSFurnitureGeometry_<ContainerAllocator> >::other > & vec) { this->furniture_geometry_list = vec; }
private:
  static const char* __s_getDataType_() { return "srs_symbolic_grounding/SymbolGroundingExploreBasePoseRequest"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "23cc208c15dce6f450656fc4c209b68c"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "159a1f856b1040c3078859c07478c7c0"; }
public:
  ROS_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROS_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "SRSFurnitureGeometry parent_obj_geometry\n\
SRSFurnitureGeometry[] furniture_geometry_list\n\
\n\
================================================================================\n\
MSG: srs_symbolic_grounding/SRSFurnitureGeometry\n\
# Point point\n\
# Orientation angles\n\
float32 l\n\
float32 w\n\
float32 h\n\
\n\
geometry_msgs/Pose pose\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of postion and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, parent_obj_geometry);
    ros::serialization::serialize(stream, furniture_geometry_list);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, parent_obj_geometry);
    ros::serialization::deserialize(stream, furniture_geometry_list);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(parent_obj_geometry);
    size += ros::serialization::serializationLength(furniture_geometry_list);
    return size;
  }

  typedef boost::shared_ptr< ::srs_symbolic_grounding::SymbolGroundingExploreBasePoseRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::srs_symbolic_grounding::SymbolGroundingExploreBasePoseRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct SymbolGroundingExploreBasePoseRequest
typedef  ::srs_symbolic_grounding::SymbolGroundingExploreBasePoseRequest_<std::allocator<void> > SymbolGroundingExploreBasePoseRequest;

typedef boost::shared_ptr< ::srs_symbolic_grounding::SymbolGroundingExploreBasePoseRequest> SymbolGroundingExploreBasePoseRequestPtr;
typedef boost::shared_ptr< ::srs_symbolic_grounding::SymbolGroundingExploreBasePoseRequest const> SymbolGroundingExploreBasePoseRequestConstPtr;


template <class ContainerAllocator>
struct SymbolGroundingExploreBasePoseResponse_ {
  typedef SymbolGroundingExploreBasePoseResponse_<ContainerAllocator> Type;

  SymbolGroundingExploreBasePoseResponse_()
  : explore_base_pose_list()
  {
  }

  SymbolGroundingExploreBasePoseResponse_(const ContainerAllocator& _alloc)
  : explore_base_pose_list(_alloc)
  {
  }

  typedef std::vector< ::geometry_msgs::Pose2D_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Pose2D_<ContainerAllocator> >::other >  _explore_base_pose_list_type;
  std::vector< ::geometry_msgs::Pose2D_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Pose2D_<ContainerAllocator> >::other >  explore_base_pose_list;


  ROS_DEPRECATED uint32_t get_explore_base_pose_list_size() const { return (uint32_t)explore_base_pose_list.size(); }
  ROS_DEPRECATED void set_explore_base_pose_list_size(uint32_t size) { explore_base_pose_list.resize((size_t)size); }
  ROS_DEPRECATED void get_explore_base_pose_list_vec(std::vector< ::geometry_msgs::Pose2D_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Pose2D_<ContainerAllocator> >::other > & vec) const { vec = this->explore_base_pose_list; }
  ROS_DEPRECATED void set_explore_base_pose_list_vec(const std::vector< ::geometry_msgs::Pose2D_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Pose2D_<ContainerAllocator> >::other > & vec) { this->explore_base_pose_list = vec; }
private:
  static const char* __s_getDataType_() { return "srs_symbolic_grounding/SymbolGroundingExploreBasePoseResponse"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "769d7fb980986ff55d6f22225f033b69"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "159a1f856b1040c3078859c07478c7c0"; }
public:
  ROS_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROS_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "geometry_msgs/Pose2D[] explore_base_pose_list\n\
\n\
\n\
\n\
\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose2D\n\
# This expresses a position and orientation on a 2D manifold.\n\
\n\
float64 x\n\
float64 y\n\
float64 theta\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, explore_base_pose_list);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, explore_base_pose_list);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(explore_base_pose_list);
    return size;
  }

  typedef boost::shared_ptr< ::srs_symbolic_grounding::SymbolGroundingExploreBasePoseResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::srs_symbolic_grounding::SymbolGroundingExploreBasePoseResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct SymbolGroundingExploreBasePoseResponse
typedef  ::srs_symbolic_grounding::SymbolGroundingExploreBasePoseResponse_<std::allocator<void> > SymbolGroundingExploreBasePoseResponse;

typedef boost::shared_ptr< ::srs_symbolic_grounding::SymbolGroundingExploreBasePoseResponse> SymbolGroundingExploreBasePoseResponsePtr;
typedef boost::shared_ptr< ::srs_symbolic_grounding::SymbolGroundingExploreBasePoseResponse const> SymbolGroundingExploreBasePoseResponseConstPtr;

struct SymbolGroundingExploreBasePose
{

typedef SymbolGroundingExploreBasePoseRequest Request;
typedef SymbolGroundingExploreBasePoseResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct SymbolGroundingExploreBasePose
} // namespace srs_symbolic_grounding

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::srs_symbolic_grounding::SymbolGroundingExploreBasePoseRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::srs_symbolic_grounding::SymbolGroundingExploreBasePoseRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::srs_symbolic_grounding::SymbolGroundingExploreBasePoseRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "23cc208c15dce6f450656fc4c209b68c";
  }

  static const char* value(const  ::srs_symbolic_grounding::SymbolGroundingExploreBasePoseRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x23cc208c15dce6f4ULL;
  static const uint64_t static_value2 = 0x50656fc4c209b68cULL;
};

template<class ContainerAllocator>
struct DataType< ::srs_symbolic_grounding::SymbolGroundingExploreBasePoseRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "srs_symbolic_grounding/SymbolGroundingExploreBasePoseRequest";
  }

  static const char* value(const  ::srs_symbolic_grounding::SymbolGroundingExploreBasePoseRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::srs_symbolic_grounding::SymbolGroundingExploreBasePoseRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "SRSFurnitureGeometry parent_obj_geometry\n\
SRSFurnitureGeometry[] furniture_geometry_list\n\
\n\
================================================================================\n\
MSG: srs_symbolic_grounding/SRSFurnitureGeometry\n\
# Point point\n\
# Orientation angles\n\
float32 l\n\
float32 w\n\
float32 h\n\
\n\
geometry_msgs/Pose pose\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of postion and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
";
  }

  static const char* value(const  ::srs_symbolic_grounding::SymbolGroundingExploreBasePoseRequest_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::srs_symbolic_grounding::SymbolGroundingExploreBasePoseResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::srs_symbolic_grounding::SymbolGroundingExploreBasePoseResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::srs_symbolic_grounding::SymbolGroundingExploreBasePoseResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "769d7fb980986ff55d6f22225f033b69";
  }

  static const char* value(const  ::srs_symbolic_grounding::SymbolGroundingExploreBasePoseResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x769d7fb980986ff5ULL;
  static const uint64_t static_value2 = 0x5d6f22225f033b69ULL;
};

template<class ContainerAllocator>
struct DataType< ::srs_symbolic_grounding::SymbolGroundingExploreBasePoseResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "srs_symbolic_grounding/SymbolGroundingExploreBasePoseResponse";
  }

  static const char* value(const  ::srs_symbolic_grounding::SymbolGroundingExploreBasePoseResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::srs_symbolic_grounding::SymbolGroundingExploreBasePoseResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "geometry_msgs/Pose2D[] explore_base_pose_list\n\
\n\
\n\
\n\
\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose2D\n\
# This expresses a position and orientation on a 2D manifold.\n\
\n\
float64 x\n\
float64 y\n\
float64 theta\n\
";
  }

  static const char* value(const  ::srs_symbolic_grounding::SymbolGroundingExploreBasePoseResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::srs_symbolic_grounding::SymbolGroundingExploreBasePoseRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.parent_obj_geometry);
    stream.next(m.furniture_geometry_list);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct SymbolGroundingExploreBasePoseRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::srs_symbolic_grounding::SymbolGroundingExploreBasePoseResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.explore_base_pose_list);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct SymbolGroundingExploreBasePoseResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<srs_symbolic_grounding::SymbolGroundingExploreBasePose> {
  static const char* value() 
  {
    return "159a1f856b1040c3078859c07478c7c0";
  }

  static const char* value(const srs_symbolic_grounding::SymbolGroundingExploreBasePose&) { return value(); } 
};

template<>
struct DataType<srs_symbolic_grounding::SymbolGroundingExploreBasePose> {
  static const char* value() 
  {
    return "srs_symbolic_grounding/SymbolGroundingExploreBasePose";
  }

  static const char* value(const srs_symbolic_grounding::SymbolGroundingExploreBasePose&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<srs_symbolic_grounding::SymbolGroundingExploreBasePoseRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "159a1f856b1040c3078859c07478c7c0";
  }

  static const char* value(const srs_symbolic_grounding::SymbolGroundingExploreBasePoseRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<srs_symbolic_grounding::SymbolGroundingExploreBasePoseRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "srs_symbolic_grounding/SymbolGroundingExploreBasePose";
  }

  static const char* value(const srs_symbolic_grounding::SymbolGroundingExploreBasePoseRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<srs_symbolic_grounding::SymbolGroundingExploreBasePoseResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "159a1f856b1040c3078859c07478c7c0";
  }

  static const char* value(const srs_symbolic_grounding::SymbolGroundingExploreBasePoseResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<srs_symbolic_grounding::SymbolGroundingExploreBasePoseResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "srs_symbolic_grounding/SymbolGroundingExploreBasePose";
  }

  static const char* value(const srs_symbolic_grounding::SymbolGroundingExploreBasePoseResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // SRS_SYMBOLIC_GROUNDING_SERVICE_SYMBOLGROUNDINGEXPLOREBASEPOSE_H
