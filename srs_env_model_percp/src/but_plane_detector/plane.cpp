#include <srs_env_model_percp/but_plane_detector/plane.h>

namespace srs_env_model_percp {

PlaneExt::PlaneExt(but_plane_detector::Plane<float> plane) : but_plane_detector::Plane<float> (0.0, 0.0, 0.0, 0.0), planeCoefficients(new pcl::ModelCoefficients())
{
	a = plane.a;
	b = plane.b;
	c = plane.c;
	d = plane.d;
	norm = plane.norm;

	// Create a quaternion for rotation into XY plane
	float x, y, z, w;
	Eigen::Vector3f current(a, b, c);
	Eigen::Vector3f target(0.0, 0.0, 1.0);
	Eigen::Quaternion<float> q;
	q.setFromTwoVectors(current, target);
	planeTransXY = q.toRotationMatrix();

	planeShift = -d;


	// Plane coefficients pre-calculation...
	planeCoefficients->values.push_back(a);
	planeCoefficients->values.push_back(b);
	planeCoefficients->values.push_back(c);
	planeCoefficients->values.push_back(d);

}

PlaneExt::tVertices PlaneExt::ComputeConcaveHull(pcl::PointCloud<pcl::PointXYZ>::Ptr &plane_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &plane_hull)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ> ());

    // project all points onto current plane
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (plane_cloud);
    proj.setModelCoefficients (planeCoefficients);
    proj.filter(*cloud_projected);

    // create the concave hull of the plane
    pcl::ConcaveHull<pcl::PointXYZ > chull;
    chull.setInputCloud (cloud_projected);
    chull.setAlpha(0.05);

    tVertices polys;
    chull.reconstruct(*plane_hull, polys);

    return polys;
}

ClipperLib::ExPolygons PlaneExt::PolygonizeConcaveHull(pcl::PointCloud<pcl::PointXYZ>::Ptr &plane_hull, tVertices &polygon_indices)
{
	ClipperLib::ExPolygons polygon;

	// Create transformed point cloud (polygon is aligned with XY plane
	pcl::PointCloud<pcl::PointXYZ>::Ptr plane_hull_proj (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::transformPointCloud(*plane_hull, *plane_hull_proj, Eigen::Vector3f(-a*planeShift, -b*planeShift, -c*planeShift), Eigen::Quaternion<float>(0,0,0,0));
	pcl::transformPointCloud(*plane_hull_proj, *plane_hull_proj, planeTransXY);

	// save each point into Clipper lib polygon structure
	if (plane_hull_proj->points.size() > 0)
	{
		for (unsigned int i = 0; i < polygon_indices.size(); ++i)
		{
			ClipperLib::ExPolygon clipperPoly;
			clipperPoly.outer.resize(polygon_indices[i].vertices.size());

			for (unsigned int j = 0; j < polygon_indices[i].vertices.size(); ++j)
			{
				clipperPoly.outer[j].X = CONVERT_TO_LONG(plane_hull_proj->points[polygon_indices[i].vertices[j]].x);
				clipperPoly.outer[j].Y = CONVERT_TO_LONG(plane_hull_proj->points[polygon_indices[i].vertices[j]].y);
			}

			// Orientation check
			if (!ClipperLib::Orientation(clipperPoly.outer))
			{
				ClipperLib::ReversePolygon(clipperPoly.outer);
			}
	    	polygon.push_back(clipperPoly);
		}
	}
	return polygon;
}

void PlaneExt::ConcaveHullRewrite(pcl::PointCloud<pcl::PointXYZ>::Ptr &plane_hull, tVertices &polygon_indices)
{
	// Rewrite saved polygon (previous points will be deleted)
	planePolygonsClipper = PolygonizeConcaveHull(plane_hull, polygon_indices);
}

void PlaneExt::ConcaveHullJoinCurrent(pcl::PointCloud<pcl::PointXYZ>::Ptr &plane_hull, tVertices &polygon_indices)
{
	// Join new polygon with tShapeMarkercurrent
	ClipperLib::ExPolygons newPoly = PolygonizeConcaveHull(plane_hull, polygon_indices);

	ClipperLib::Clipper clipper;
	// insert all existing polygons
	for (unsigned int i = 0; i < planePolygonsClipper.size(); ++i)
		clipper.AddPolygon(planePolygonsClipper[i].outer, ClipperLib::ptSubject);

	// insert all new polygons
	for (unsigned int i = 0; i < newPoly.size(); ++i)
		clipper.AddPolygon(newPoly[i].outer, ClipperLib::ptClip);

	// execute join operation
	clipper.Execute(ClipperLib::ctUnion, planePolygonsClipper);
}

void PlaneExt::TriangulatePlanePolygon()
{
	// clear Marker and Shape messages
	planeTriangles.points.clear();
	planeTrianglesSRS.clear();

	// for all polygons representing this plane
	for (unsigned int polygon_i = 0; polygon_i < planePolygonsClipper.size(); ++polygon_i)
	{
		planeTrianglesSRS.push_back(cob_3d_mapping_msgs::Shape());
		planeTrianglesSRS.back().type = cob_3d_mapping_msgs::Shape::POLYGON;
		planeTrianglesSRS.back().params.push_back(a);
		planeTrianglesSRS.back().params.push_back(b);
		planeTrianglesSRS.back().params.push_back(c);
		planeTrianglesSRS.back().params.push_back(d);
		planeTrianglesSRS.back().holes.push_back(false);

		// convert each polygon to polygonizer DS
    	TPPLPoly triPolygon;
    	triPolygon.Init(planePolygonsClipper[polygon_i].outer.size());
    	pcl::PointCloud<pcl::PointXYZ> current_point_cloud;

		for (unsigned int i = 0; i < planePolygonsClipper[polygon_i].outer.size(); ++i)
		{
			triPolygon[i].x = CONVERT_FROM_LONG(planePolygonsClipper[polygon_i].outer[i].X);
			triPolygon[i].y = CONVERT_FROM_LONG(planePolygonsClipper[polygon_i].outer[i].Y);

			// additionaly, insert this point into shape message
			pcl::PointXYZ point;
			point.x = triPolygon[i].x;
			point.y = triPolygon[i].y;
			point.z = 0;
			current_point_cloud.push_back(point);
		}

		// triangulate
		TPPLPartition triangulation;
		std::list<TPPLPoly> triangles;
		triangulation.Triangulate_EC(&triPolygon, &triangles);

		// create a message object
		for (std::list<TPPLPoly>::iterator it = triangles.begin(); it != triangles.end(); ++it)
		{
			for (unsigned int j = 0; j < it->GetNumPoints(); ++j)
			{
				Eigen::Vector3f vec;
				vec(0) = it->GetPoint(j).x;
				vec(1) = it->GetPoint(j).y;
				vec(2) = 0;
				vec = planeTransXY.inverse() * vec;
				geometry_msgs::Point p;
				p.x = vec(0) + planeShift*a;
				p.y = vec(1) + planeShift*b;
				p.z = vec(2) + planeShift*c;

				planeTriangles.points.push_back(p);
			}
		}

		// insert polygon point cloud into Shape message
		pcl::transformPointCloud(current_point_cloud, current_point_cloud, planeTransXY.inverse());
		pcl::transformPointCloud(current_point_cloud, current_point_cloud, Eigen::Vector3f(a*planeShift, b*planeShift, c*planeShift), Eigen::Quaternion<float>(0,0,0,0));

		sensor_msgs::PointCloud2 cloud;
		pcl::toROSMsg(current_point_cloud, cloud);
		planeTrianglesSRS.back().points.push_back(cloud);
		planeTrianglesSRS.back().centroid.x = planeShift*a;
		planeTrianglesSRS.back().centroid.y = planeShift*b;
		planeTrianglesSRS.back().centroid.z = planeShift*c;
		planeTrianglesSRS.back().color.r = abs(a) / 2.0 + 0.2;
		planeTrianglesSRS.back().color.g = abs(b) / 2.0 + 0.2;
		planeTrianglesSRS.back().color.b = abs(d) / 10.0 + 0.2;
		planeTrianglesSRS.back().color.a = 1.0;
	}

	// compute centroid
}

visualization_msgs::Marker PlaneExt::NewPlanePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud)
{
	// Init a new plane hull (previous will be deleted
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>());
	tVertices polygons = ComputeConcaveHull(plane_cloud, cloud_hull);

	// revrite hull
	ConcaveHullRewrite(cloud_hull, polygons);

	// triangulate
	TriangulatePlanePolygon();

	return planeTriangles;
}

visualization_msgs::Marker PlaneExt::AddPlanePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud)
{
	// Add new plane hull
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>());
	tVertices polygons = ComputeConcaveHull(plane_cloud, cloud_hull);

	// Join with current
	ConcaveHullJoinCurrent(cloud_hull, polygons);

	// Triangulate
	TriangulatePlanePolygon();

	return planeTriangles;
}

visualization_msgs::Marker &PlaneExt::getMeshMarker()
{
	return planeTriangles;
}

PlaneExt::tShapeMarker &PlaneExt::getShapeMarker()
{
	return planeTrianglesSRS;
}

}
