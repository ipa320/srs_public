#ifndef PLANE_EXT_H
#define PLANE_EXT_H

#include <but_segmentation/normals.h>
#include <srs_env_model_percp/but_plane_detector/clipper.hpp>
#include <srs_env_model_percp/but_plane_detector/polypartition.h>
#include <visualization_msgs/Marker.h>
#include <cob_3d_mapping_msgs/Shape.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <pcl_ros/transforms.h>

// conversion between any type to ClipperLib long long type and back
#define CONVERT_TO_LONG(a) a * 1000000
#define CONVERT_FROM_LONG(a) (float)a / 1000000

namespace srs_env_model_percp
{

class PlaneExt : public but_plane_detector::Plane<float>
{
public:
	typedef std::vector<cob_3d_mapping_msgs::Shape, Eigen::aligned_allocator<cob_3d_mapping_msgs::Shape> > tShapeMarker;

//	typedef std::vector<pcl::Vertices, Eigen::aligned_allocator<pcl::Vertices> > tVertices;
	typedef std::vector<pcl::Vertices> tVertices;

public:
	PlaneExt(but_plane_detector::Plane<float> plane);

	visualization_msgs::Marker NewPlanePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud);
	visualization_msgs::Marker AddPlanePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud);
	// Adds new points to current polygon

	ClipperLib::ExPolygon 	   &getPolygon();
	std::list<TPPLPoly>   	   &getMesh();

	visualization_msgs::Marker &getMeshMarker();

	tShapeMarker &getShapeMarker();

protected:
	// Computes concave hull of set of points
	tVertices ComputeConcaveHull(pcl::PointCloud<pcl::PointXYZ>::Ptr &plane_cloud, 
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr &plane_hull
                                     );

	// Computes hull U current polygon
	void ConcaveHullJoinCurrent(pcl::PointCloud<pcl::PointXYZ>::Ptr &plane_hull, 
                                    tVertices &polygon_indices
                                    );

	// Rewrites current plane with this hull
	void ConcaveHullRewrite(pcl::PointCloud<pcl::PointXYZ>::Ptr &plane_hull, 
                                tVertices &polygon_indices
                                );

	ClipperLib::ExPolygons PolygonizeConcaveHull(pcl::PointCloud<pcl::PointXYZ>::Ptr &plane_hull, 
                                                     tVertices &polygon_indices
                                                     );

	// Simplifies plane polygon
	//void SimplifyPolygon();

	// Triangulates plane polygon
	void TriangulatePlanePolygon();

	ClipperLib::ExPolygons	planePolygonsClipper;

	Eigen::Affine3f		planeTransXY;

	double planeShift;

	pcl::ModelCoefficients::Ptr planeCoefficients;

	visualization_msgs::Marker 	planeTriangles;

	tShapeMarker planeTrianglesSRS;

	Eigen::Quaternion<float> rotationQuaternion;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


}

#endif

