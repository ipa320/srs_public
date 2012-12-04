/******************************************************************************
 * \file
 *
 * $Id: DynModelExporter.cpp 814 2012-05-22 14:00:19Z ihulik $
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Rostislav Hulik (ihulik@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 11.01.2012 (version 0.8)
 * 
 * This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with this file.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * Description:
 *	 Encapsulates a class of plane exporter (export to but_gui module/interactive markers)
 */

#include <srs_env_model_percp/but_plane_detector/dyn_model_exporter.h>
#include <srs_env_model_percp/topics_list.h>
#include <srs_env_model_percp/services_list.h>


#include <srs_interaction_primitives/AddPlane.h>
#include <srs_interaction_primitives/RemovePrimitive.h>
#include <srs_interaction_primitives/plane.h>
#include <pcl/filters/project_inliers.h>

#include <srs_env_model/InsertPlanes.h>
#include <pcl/point_cloud.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl_ros/transforms.h>

#include <libxml2/libxml/parser.h>
#include <libxml2/libxml/tree.h>

using namespace pcl;
using namespace but_plane_detector;
using namespace cv;

namespace srs_env_model_percp
{

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Updates sent planes using but environment model server
	// @param planes Vector of found planes
	// @param scene_cloud point cloud of the scene
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void DynModelExporter::update(tPlanes & planes, Normals &normals, std::string color_method, cv::Mat rgb)
	{
		if (m_keep_tracking == 0)
			displayed_planes.clear();
		std::vector<PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<PointCloud<pcl::PointXYZ> > > planesInPCloud(planes.size());
		std::vector<std_msgs::ColorRGBA, Eigen::aligned_allocator<std_msgs::ColorRGBA> > colors(planes.size());

		for (int i = 0; i < normals.m_points.rows; ++i)
		for (int j = 0; j < normals.m_points.cols; ++j)
		{
			Vec3f point = normals.m_points.at<Vec3f>(i, j);
			if (point[0] != 0.0 || point[1] != 0.0 || point[2] != 0.0)
			{
				cv::Vec4f localPlane = normals.m_planes.at<cv::Vec4f>(i, j);
				Plane<float> aaa(localPlane[0], localPlane[1], localPlane[2], localPlane[3]);

				// find all planes that fit for this point
				for (unsigned int a = 0; a < planes.size(); ++a)
				{
					if (planes[a].distance(point) < m_max_distance && planes[a].isSimilar(aaa, m_max_plane_normal_dev, m_max_plane_shift_dev))
					{
						PointXYZ pclpoint(point[0], point[1], point[2]);
						planesInPCloud[a].push_back(pclpoint);

						if (color_method == "mean_color")
						{
							cv::Vec<unsigned char, 3> color = rgb.at<cv::Vec<unsigned char, 3> >(i, j);
							colors[a].r += (float)color[0]/255.0;
							colors[a].g += (float)color[1]/255.0;
							colors[a].b += (float)color[2]/255.0;
						}
						else if (color_method == "mean_color")
						{
							cv::Vec<unsigned char, 3> color = rgb.at<cv::Vec<unsigned char, 3> >(i, j);
							colors[a].r += point[0] / 5.0;
							colors[a].g += point[1] / 5.0;
							colors[a].b += point[2] / 5.0;
						}
					}
				}
			}
		}
//		for (int i = 0; i < normals.m_points.rows; ++i)
//		for (int j = 0; j < normals.m_points.cols; ++j)
//		{
//			Vec3f point = normals.m_points.at<Vec3f>(i, j);
//			if (point[0] != 0.0 || point[1] != 0.0 || point[2] != 0.0)
//			{
//				cv::Vec4f localPlane = normals.m_planes.at<cv::Vec4f>(i, j);
//				Plane<float> aaa(localPlane[0], localPlane[1], localPlane[2], localPlane[3]);
//
//				double dist = DBL_MAX;
//
//				int chosen = -1;
//				// find the best plane
//				for (unsigned int a = 0; a < planes.size(); ++a)
//				{
//					if (planes[a].distance(point) < dist && planes[a].distance(point) < m_max_distance &&
//						planes[a].isSimilar(aaa, m_max_plane_normal_dev, m_max_plane_shift_dev))
//					{
//						dist = planes[a].distance(point);
//						chosen = a;
//					}
//				}
//
//				// if there is good plane, insert point into point cloud
//				if (chosen > -1)
//				{
//					PointXYZ pclpoint(point[0], point[1], point[2]);
//					planesInPCloud[chosen].push_back(pclpoint);
//
//					if (color_method == "mean_color")
//					{
//						cv::Vec<unsigned char, 3> color = rgb.at<cv::Vec<unsigned char, 3> >(i, j);
//						colors[chosen].r += (float)color[0]/255.0;
//						colors[chosen].g += (float)color[1]/255.0;
//						colors[chosen].b += (float)color[2]/255.0;
//						//std::cerr << color[0] << " " << color[1] << " " << color[2] << std::endl;
//					}
//					else if (color_method == "mean_color")
//					{
//						cv::Vec<unsigned char, 3> color = rgb.at<cv::Vec<unsigned char, 3> >(i, j);
//						colors[chosen].r += point[0] / 5.0;
//						colors[chosen].g += point[1] / 5.0;
//						colors[chosen].b += point[2] / 5.0;
//											//std::cerr << color[0] << " " << color[1] << " " << color[2] << std::endl;
//					}
////					if (point[2] > 0.9 && point[2] < 1.0 && (planes[chosen].d < -0.5 || planes[chosen].d > 0.5) && (planes[chosen].c < -0.5 || planes[chosen].c > 0.5))
////					{
////						std::cerr << planes[chosen].a << " " << planes[chosen].b << " " << planes[chosen].c << " " << planes[chosen].d << " --- > ";
////						std::cerr << point[0] << " " << point[1] << " " << point[2] << std::endl;
////					}
//				}
//			}
//		}

		if (color_method == "mean_color" || color_method == "centroid")
		{
			for (unsigned int i = 0; i < colors.size(); ++i)
				if(planesInPCloud.size() > 0)
				{
					colors[i].r /= planesInPCloud[i].size();
					colors[i].g /= planesInPCloud[i].size();
					colors[i].b /= planesInPCloud[i].size();
					colors[i].a = 1.0;
				}
		}
		// Indexed in point cloud
		////////////////////////////////////////////////////////////////////////////////////////////////

		for (unsigned int j = 0; j < planesInPCloud.size(); ++j)
		{
			if (planesInPCloud[j].size() > 20)
			{
				double maxangle = DBL_MAX;
				double maxdist = DBL_MAX;
				int index = -1;
				for (unsigned int i = 0; i < displayed_planes.size(); ++i)
				{
					if (!(displayed_planes[i].is_deleted))
					{
						double angle = acos(((planes[j].a * displayed_planes[i].plane.a) + (planes[j].b * displayed_planes[i].plane.b) + (planes[j].c * displayed_planes[i].plane.c)));
						double xd = planes[j].d - displayed_planes[i].plane.d;
						xd = (xd > 0 ? xd : - xd);

						// Pretty nasty workaround... todo
						if (angle != angle) angle = 0.0;

						if (angle <= maxangle  && xd <= maxdist && angle < m_max_plane_normal_dev  && xd < m_max_plane_shift_dev)
						{
							maxangle = angle;
							maxdist = xd;
							index = i;
						}
					}
				}

				if (index >= 0)
				{
					DynModelExporter::addMarkerToConcaveHull(planesInPCloud[j], displayed_planes[index].plane);
					displayed_planes[index].update = ros::Time::now();
				}
				else
				{
					ExportedPlane newplane;
					newplane.update = ros::Time::now();
					newplane.plane = PlaneExt(planes[j]);
					newplane.is_deleted = false;
					newplane.to_be_deleted = false;
					DynModelExporter::createMarkerForConcaveHull(planesInPCloud[j], newplane.plane);

					newplane.id = displayed_planes.size();
					newplane.plane.getMeshMarker().id = newplane.id;
					if (color_method == "mean_color")
					{
						std::cerr << "setting color: " << colors[j].r << " " << colors[j].g << " " << colors[j].b << std::endl;
						newplane.plane.setColor(colors[j]);
					}
					else if (color_method == "random")
					{
						colors[j].r = (float)rand()/INT_MAX * 0.5 + 0.2;
						colors[j].g = (float)rand()/INT_MAX * 0.5 + 0.2;
						colors[j].b = (float)rand()/INT_MAX * 0.5 + 0.2;
						colors[j].a = 1.0;
						newplane.plane.setColor(colors[j]);
					}
					//newplane.plane.getShapeMarker().id = newplane.id;
					displayed_planes.push_back(newplane);
				}
			}
		}

		// TTL fix
		if (m_plane_ttl > 0)
		{
			for (unsigned int i = 0; i < displayed_planes.size(); ++i)
			{
				if (!displayed_planes[i].is_deleted && !displayed_planes[i].to_be_deleted && (ros::Time::now() - displayed_planes[i].update).sec > m_plane_ttl)
				{
					std::cerr << (ros::Time::now() - displayed_planes[i].update).sec << " ";
					displayed_planes[i].is_deleted = true;
					displayed_planes[i].to_be_deleted = true;
				}
				else if (displayed_planes[i].is_deleted && displayed_planes[i].to_be_deleted)
				{
					displayed_planes[i].to_be_deleted = false;
				}
			}
		}
	}




	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Initialization
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	DynModelExporter::DynModelExporter(ros::NodeHandle *node,
	                                   const std::string& original_frame,
	                                   const std::string& output_frame,
	                                   int minOutputCount,
	                                   double max_distance,
	                                   double max_plane_normal_dev,
	                                   double max_plane_shift_dev,
	                                   int keep_tracking,
	                                   int ttl
	                                   )
	    : original_frame_(original_frame)
	    , output_frame_(output_frame)
	{
		n = node;
		m_minOutputCount = minOutputCount;
		m_max_distance = max_distance;
		m_max_plane_normal_dev = max_plane_normal_dev;
		m_max_plane_shift_dev = max_plane_shift_dev;
		m_keep_tracking = keep_tracking;
		m_plane_ttl = ttl;
	}

	void DynModelExporter::createMarkerForConcaveHull(pcl::PointCloud<pcl::PointXYZ>& plane_cloud, srs_env_model_percp::PlaneExt& plane)
	{
		plane.NewPlanePoints(plane_cloud.makeShared());
	    plane.getMeshMarker().type = visualization_msgs::Marker::TRIANGLE_LIST;
	    plane.getMeshMarker().action = visualization_msgs::Marker::ADD;

	    plane.getMeshMarker().ns = "Normals";
	    plane.getMeshMarker().pose.position.x = 0.0;
	    plane.getMeshMarker().pose.position.y = 0.0;
	    plane.getMeshMarker().pose.position.z = 0.0;
	    plane.getMeshMarker().pose.orientation.x = 0.0;
	    plane.getMeshMarker().pose.orientation.y = 0.0;
	    plane.getMeshMarker().pose.orientation.z = 0.0;
	    plane.getMeshMarker().pose.orientation.w = 1.0;
	    plane.getMeshMarker().scale.x = 1.00;
	    plane.getMeshMarker().scale.y = 1.00;
	    plane.getMeshMarker().scale.z = 1.00;
//

	}

	void DynModelExporter::addMarkerToConcaveHull(pcl::PointCloud<pcl::PointXYZ>& plane_cloud, srs_env_model_percp::PlaneExt& plane)
	{
		plane.AddPlanePoints(plane_cloud.makeShared());
	}

	void DynModelExporter::xmlFileExport(std::string filename)
	{
		xmlDocPtr document = NULL;
		xmlNodePtr root_node = NULL;
		xmlNodePtr plane_node = NULL;
		xmlNodePtr polys_node = NULL;
		xmlNodePtr poly_node = NULL;
		xmlNodePtr outer_node = NULL;
		xmlNodePtr holes_node = NULL;
		xmlNodePtr hole_node = NULL;
		xmlNodePtr points_node = NULL;
		xmlNodePtr node = NULL;

		/////////////////////////////////////////////////////
		// XML structure
		//
		// <planes>
		//	 <plane id=... poly_number=...>
		//	   <equation a=... b=... c=... d=... />
		//     <polygons>
		//	     <polygon>
		//	       <outer>
		//		     <points>
		//		       <point x=.. y=..>
		//		     </points>
		//	       </outer>
		//	       <holes>
		//  	     <hole>
		//		       <points>
		//		         <point x=.. y=..>
		//		       </points>
		//  	     </hole>
		//			 .......
		//	       </holes>
		//	     </polygon>
		//       ......
		//     </polygons>
		//	 </plane>
		//   ......
		// </planes>
		/////////////////////////////////////////////////////
		// Create a new XML document
		document = xmlNewDoc(BAD_CAST "1.0");
		root_node = xmlNewNode(NULL, BAD_CAST "planes");
		xmlDocSetRootElement(document, root_node);

		for (unsigned int i = 0; i < displayed_planes.size(); ++i)
		{
			if (!displayed_planes[i].is_deleted)
			{
				ClipperLib::ExPolygons polygons = displayed_planes[i].plane.getPolygons();

				// save plane global info
				plane_node = xmlNewChild(root_node, NULL, BAD_CAST "plane", NULL);

				std::stringstream str;
				str << i;
				xmlNewProp(plane_node, BAD_CAST "id", BAD_CAST str.str().c_str());

				str.str("");
				str << polygons.size();
				xmlNewProp(plane_node, BAD_CAST "poly_number", BAD_CAST str.str().c_str());

				// plane equation
				node = xmlNewChild(plane_node, NULL, BAD_CAST "equation", NULL);

				str.str("");
				str << displayed_planes[i].plane.a;
				xmlNewProp(node, BAD_CAST "a", BAD_CAST str.str().c_str());

				str.str("");
				str << displayed_planes[i].plane.b;
				xmlNewProp(node, BAD_CAST "b", BAD_CAST str.str().c_str());

				str.str("");
				str << displayed_planes[i].plane.c;
				xmlNewProp(node, BAD_CAST "c", BAD_CAST str.str().c_str());

				str.str("");
				str << displayed_planes[i].plane.d;
				xmlNewProp(node, BAD_CAST "d", BAD_CAST str.str().c_str());

				// color
				node = xmlNewChild(plane_node, NULL, BAD_CAST "color", NULL);

				str.str("");
				str << displayed_planes[i].plane.color.r;
				xmlNewProp(node, BAD_CAST "r", BAD_CAST str.str().c_str());

				str.str("");
				str << displayed_planes[i].plane.color.g;
				xmlNewProp(node, BAD_CAST "g", BAD_CAST str.str().c_str());

				str.str("");
				str << displayed_planes[i].plane.color.b;
				xmlNewProp(node, BAD_CAST "b", BAD_CAST str.str().c_str());

				str.str("");
				str << displayed_planes[i].plane.color.a;
				xmlNewProp(node, BAD_CAST "a", BAD_CAST str.str().c_str());

				// save polygons
				polys_node = xmlNewChild(plane_node, NULL, BAD_CAST "polygons", NULL);
				for (unsigned int j = 0; j < polygons.size(); ++j)
				{
					poly_node = xmlNewChild(polys_node, NULL, BAD_CAST "polygon", NULL);

					str.str("");
					str << j;
					xmlNewProp(poly_node, BAD_CAST "id", BAD_CAST str.str().c_str());

					// outer body
					outer_node = xmlNewChild(poly_node, NULL, BAD_CAST "outer", NULL);
					points_node = xmlNewChild(outer_node, NULL, BAD_CAST "points", NULL);
					for (unsigned int k = 0; k < polygons[j].outer.size(); ++k)
					{
						node = xmlNewChild(points_node, NULL, BAD_CAST "point", NULL);

						str.str("");
						str << polygons[j].outer[k].X;
						xmlNewProp(node, BAD_CAST "x", BAD_CAST str.str().c_str());

						str.str("");
						str << polygons[j].outer[k].Y;
						xmlNewProp(node, BAD_CAST "y", BAD_CAST str.str().c_str());
					}

	//				// export holes
	//				holes_node = xmlNewChild(poly_node, NULL, BAD_CAST "holes", NULL);
	//				for (unsigned int k = 0; k < polygons[j].holes.size(); ++k)
	//				{
	//					hole_node = xmlNewChild(holes_node, NULL, BAD_CAST "hole", NULL);
	//					points_node = xmlNewChild(hole_node, NULL, BAD_CAST "points", NULL);
	//					for (unsigned int l = 0; l < polygons[j].holes[k].size(); ++l)
	//					{
	//						node = xmlNewChild(points_node, NULL, BAD_CAST "point", NULL);
	//
	//						str.str("");
	//						str <<  polygons[j].holes[k][l].X;
	//						xmlNewProp(node, BAD_CAST "x", BAD_CAST str.str().c_str());
	//
	//						str.str("");
	//						str <<  polygons[j].holes[k][l].Y;
	//						xmlNewProp(node, BAD_CAST "y", BAD_CAST str.str().c_str());
	//					}
	//				}

				}
			}
		}

		xmlSaveFormatFileEnc(filename.c_str(), document, "UTF-8", 1);
	    xmlFreeDoc(document);
	    xmlCleanupParser();
	}

	void DynModelExporter::xmlFileImport(std::string filename)
	{
		/////////////////////////////////////////////////////
		// XML structure
		//
		// <planes>
		//	 <plane id=... poly_number=...>
		//	   <equation a=... b=... c=... d=... />
		//     <polygons>
		//	     <polygon>
		//	       <outer>
		//		     <points>
		//		       <point x=.. y=..>
		//		     </points>
		//	       </outer>
		//	       <holes>
		//  	     <hole>
		//		       <points>
		//		         <point x=.. y=..>
		//		       </points>
		//  	     </hole>
		//			 .......
		//	       </holes>
		//	     </polygon>
		//       ......
		//     </polygons>
		//	 </plane>
		//   ......
		// </planes>
		/////////////////////////////////////////////////////
		xmlDocPtr document = xmlParseFile(filename.c_str());

		if (document->children && xmlStrEqual(document->children->name, (const xmlChar *)"planes"))
		{
			// pass each node
			for (xmlNodePtr plane = document->children->children; plane; plane = plane->next)
			{

				if (xmlStrEqual(plane->name, (const xmlChar *)"plane"))
				{
					float a = 0.0;
					float b = 0.0;
					float c = 0.0;
					float d = 0.0;
					std_msgs::ColorRGBA color;
					color.r = 0.0;
					color.g = 0.0;
					color.b = 0.0;
					color.a = 0.0;
					int id = 0;
					for (xmlAttrPtr attribute = plane->properties; attribute; attribute = attribute->next)
					{
						// get id
						if (xmlStrEqual(attribute->name, (const xmlChar *)"id"))
						{
							id = atoi(reinterpret_cast<const char*>(xmlNodeListGetString(document, attribute->children, 1)));
						}
					}

					ClipperLib::ExPolygons polygons;
					for (xmlNodePtr plane_child = plane->children; plane_child; plane_child = plane_child->next)
					{
						// get equation
						if (xmlStrEqual(plane_child->name, (const xmlChar *)"equation"))
							for (xmlAttrPtr attribute = plane_child->properties; attribute; attribute = attribute->next)
							{
								if (xmlStrEqual(attribute->name, (const xmlChar *)"a"))
									a = atof(reinterpret_cast<const char*>(xmlNodeListGetString(document, attribute->children, 1)));

								if (xmlStrEqual(attribute->name, (const xmlChar *)"b"))
									b = atof(reinterpret_cast<const char*>(xmlNodeListGetString(document, attribute->children, 1)));

								if (xmlStrEqual(attribute->name, (const xmlChar *)"c"))
									c = atof(reinterpret_cast<const char*>(xmlNodeListGetString(document, attribute->children, 1)));

								if (xmlStrEqual(attribute->name, (const xmlChar *)"d"))
									d = atof(reinterpret_cast<const char*>(xmlNodeListGetString(document, attribute->children, 1)));
							}

						// get color
						if (xmlStrEqual(plane_child->name, (const xmlChar *)"color"))
							for (xmlAttrPtr attribute = plane_child->properties; attribute; attribute = attribute->next)
							{
								if (xmlStrEqual(attribute->name, (const xmlChar *)"r"))
									color.r = atof(reinterpret_cast<const char*>(xmlNodeListGetString(document, attribute->children, 1)));

								if (xmlStrEqual(attribute->name, (const xmlChar *)"g"))
									color.g = atof(reinterpret_cast<const char*>(xmlNodeListGetString(document, attribute->children, 1)));

								if (xmlStrEqual(attribute->name, (const xmlChar *)"b"))
									color.b = atof(reinterpret_cast<const char*>(xmlNodeListGetString(document, attribute->children, 1)));

								if (xmlStrEqual(attribute->name, (const xmlChar *)"a"))
									color.a = atof(reinterpret_cast<const char*>(xmlNodeListGetString(document, attribute->children, 1)));
							}

						// <POLYGONS>
						if (xmlStrEqual(plane_child->name, (const xmlChar *)"polygons"))
						for (xmlNodePtr polygon = plane_child->children; polygon; polygon = polygon->next)
						{
							// <POLYGON>
							if (xmlStrEqual(polygon->name, (const xmlChar *)"polygon") )
							{
								polygons.push_back(ClipperLib::ExPolygon());
								for (xmlNodePtr node = polygon->children; node; node = node->next)
								{
									// <OUTER>
									if (xmlStrEqual(node->name, (const xmlChar *)"outer"))
									{
										// <POINTS>
										for (xmlNodePtr points = node->children; points; points = points->next)
										if (xmlStrEqual(points->name, (const xmlChar *)"points"))
										{
											// <POINT>
											for (xmlNodePtr point = points->children; point; point = point->next)
											if (xmlStrEqual(point->name, (const xmlChar *)"point"))
											{
												int x, y;
												for (xmlAttrPtr attribute = point->properties; attribute; attribute = attribute->next)
												{
													if (xmlStrEqual(attribute->name, (const xmlChar *)"x"))
														x = atoi(reinterpret_cast<const char*>(xmlNodeListGetString(document, attribute->children, 1)));
													if (xmlStrEqual(attribute->name, (const xmlChar *)"y"))
														y = atoi(reinterpret_cast<const char*>(xmlNodeListGetString(document, attribute->children, 1)));
												}
												polygons.back().outer.push_back(ClipperLib::IntPoint(x, y));
											} // point
										} // points
									} // outer
//									//<HOLES>
//									else if (xmlStrEqual(node->name, (const xmlChar *)"holes"))
//									{
//										// <HOLE>
//										for (xmlNodePtr hole = node->children; hole; hole = node->next)
//										if (xmlStrEqual(hole->name, (const xmlChar *)"hole"))
//										{
//											polygons.back().holes.push_back(ClipperLib::Polygon());
//											// <POINTS>
//											for (xmlNodePtr points = hole->children; points; points = points->next)
//											if (xmlStrEqual(points->name, (const xmlChar *)"points"))
//											{
//												// <POINT>
//												for (xmlNodePtr point = points->children; point; point = point->next)
//												{
//													int x, y;
//													for (xmlAttrPtr attribute = point->properties; attribute; attribute = attribute->next)
//													{
//														if (xmlStrEqual(attribute->name, (const xmlChar *)"x"))
//															x = atoi(reinterpret_cast<const char*>(xmlNodeListGetString(document, attribute->children, 1)));
//														if (xmlStrEqual(attribute->name, (const xmlChar *)"y"))
//															y = atoi(reinterpret_cast<const char*>(xmlNodeListGetString(document, attribute->children, 1)));
//													}
//													polygons.back().holes.back().push_back(ClipperLib::IntPoint(x, y));
//												} // point
//											} // points
//										} // hole
//									} // holes
								} // polygon children
							} // polygon
						} // polygons
					} // plane

					// save new plane
					displayed_planes.push_back(ExportedPlane());
					displayed_planes.back().id = id;
					displayed_planes.back().plane = PlaneExt(Plane<float>(a, b, c, d));
					displayed_planes.back().plane.setColor(color);
					displayed_planes.back().is_deleted = false;
					displayed_planes.back().to_be_deleted = false;
					displayed_planes.back().update = ros::Time::now();

					displayed_planes.back().plane.setPolygons(polygons);
					displayed_planes.back().plane.TriangulatePlanePolygon();

					displayed_planes.back().plane.getMeshMarker().type = visualization_msgs::Marker::TRIANGLE_LIST;
					displayed_planes.back().plane.getMeshMarker().action = visualization_msgs::Marker::ADD;

					displayed_planes.back().plane.getMeshMarker().ns = "Normals";
					displayed_planes.back().plane.getMeshMarker().pose.position.x = 0.0;
					displayed_planes.back().plane.getMeshMarker().pose.position.y = 0.0;
					displayed_planes.back().plane.getMeshMarker().pose.position.z = 0.0;
					displayed_planes.back().plane.getMeshMarker().pose.orientation.x = 0.0;
					displayed_planes.back().plane.getMeshMarker().pose.orientation.y = 0.0;
					displayed_planes.back().plane.getMeshMarker().pose.orientation.z = 0.0;
					displayed_planes.back().plane.getMeshMarker().pose.orientation.w = 1.0;
					displayed_planes.back().plane.getMeshMarker().scale.x = 1.00;
					displayed_planes.back().plane.getMeshMarker().scale.y = 1.00;
					displayed_planes.back().plane.getMeshMarker().scale.z = 1.00;
					displayed_planes.back().plane.getMeshMarker().id = id;

				} // planes
			}
		}
	}

	void DynModelExporter::getMarkerArray(visualization_msgs::MarkerArray &message, std::string output_frame_id)
	{
       	for (unsigned int i = 0; i < displayed_planes.size(); ++i)
    	{
       		if (!displayed_planes[i].is_deleted)
       		{
       			message.markers.push_back(displayed_planes[i].plane.getMeshMarker());
       			message.markers.back().header.frame_id = output_frame_id;
       			message.markers.back().header.stamp = ros::Time::now();
       		}
       		else if (displayed_planes[i].to_be_deleted)
       		{
       			message.markers.push_back(displayed_planes[i].plane.getMeshMarker());
       			message.markers.back().header.frame_id = output_frame_id;
       			message.markers.back().header.stamp = ros::Time::now();
       			message.markers.back().action = visualization_msgs::Marker::DELETE;
       		}
    	}
	}

	void DynModelExporter::getShapeArray(cob_3d_mapping_msgs::ShapeArray &message, std::string output_frame_id)
	{
    	message.header.frame_id = output_frame_id;

    	for (unsigned int i = 0; i < displayed_planes.size(); ++i)
    	{
    		if (!displayed_planes[i].is_deleted)
    		{
    			PlaneExt::tShapeMarker shapes = displayed_planes[i].plane.getShapeMarker();
    			for (unsigned int j = 0; j < shapes.size(); ++j)
    			{
    				message.shapes.push_back(shapes[j]);
    				message.shapes.back().header.frame_id=output_frame_id;
    				message.shapes.back().id = i * 100 + j;
    			}
    		}
    	}
	}


}// but_scenemodel
