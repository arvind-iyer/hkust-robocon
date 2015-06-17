#include "stdafx.h"
#define NOMINMAX
#include <Windows.h>
#include <Kinect.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <vtkRenderWindow.h>
#include <vtkCamera.h>
#include "calibration_processing.h"
#include "point_cloud_tracking.h"
#include <chrono>
#include <limits>
#include <cmath>
#include <tuple>
#include <fstream>
#include <Shlwapi.h>

namespace {
	bool calibration_mode = true;
	bool calibration_edge_trigger = true;
	bool tracking_mode = false;

	std::pair<pcl::PointXYZ, pcl::PointXYZ> create_line_endpoints(pcl::PointXYZ start, pcl::PointXYZ end) {
		return std::make_pair(start, end);
	}

	std::pair<pcl::PointXYZ, pcl::PointXYZ> net_line = std::pair<pcl::PointXYZ, pcl::PointXYZ>();
	std::pair<pcl::PointXYZ, pcl::PointXYZ> pole_line = std::pair<pcl::PointXYZ, pcl::PointXYZ>();
	Eigen::Matrix3f coord_conversion_matrix = Eigen::Matrix3f();
	Eigen::Affine3d transformation = Eigen::Affine3d::Identity();
}

template <class Interface>
inline void SafeRelease( Interface *& pInterfaceToRelease )
{
	if( pInterfaceToRelease != NULL ){
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

void cloud_processing(pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud, pcl::visualization::PCLVisualizer& viewer)
{
	static bool removal_required = true;
	if (calibration_mode) {
		if (removal_required) {
			viewer.removeAllPointClouds();
			viewer.addPointCloud(point_cloud, "Cloud");
			removal_required = false;
		}
		else {
			if (!viewer.updatePointCloud(point_cloud, "Cloud")) {
				// Show Point Cloud on Cloud Viewer
				viewer.addPointCloud(point_cloud, "Cloud");
				viewer.resetCameraViewpoint("Cloud");
			}
		}
	}
	else {
		if (!removal_required)
		{
			viewer.removePointCloud("Cloud");
			viewer.resetCamera();
			removal_required = true;
		}
		point_cloud_tracking::track_clusters(point_cloud);
		point_cloud_tracking::display_clusters(viewer);
		//		std::cout << "Extracted clusters: " << j << std::endl;
	}
}

void point_cloud_calibration(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<UINT16>& depthBuffer, ICoordinateMapper*& pCoordinateMapper, int depthWidth, int depthHeight, pcl::visualization::PCLVisualizer& viewer)
{
	if (calibration_edge_trigger) {
		viewer.removeAllShapes();
		calibration_edge_trigger = false;
	}
	viewer.removeShape("net_line");
	viewer.removeShape("pole_line");
	viewer.removeShape("second_net_line");
	viewer.removeShape("second_pole_line");
	//	viewer.removeShape("sphere");
	viewer.addLine<pcl::PointXYZ, pcl::PointXYZ>(net_line.first, net_line.second, 102.0 / 255.0, 1.0, 0.0, "net_line");
	viewer.addLine<pcl::PointXYZ, pcl::PointXYZ>(pole_line.first, pole_line.second, 102.0 / 255.0, 1.0, 0.0, "pole_line");
	pcl::PointXYZ second_pole_base(net_line.first.x + pole_line.second.x - pole_line.first.x,
		net_line.first.y + pole_line.second.y - pole_line.first.y,
		net_line.first.z + pole_line.second.z - pole_line.first.z);
	viewer.addLine<pcl::PointXYZ, pcl::PointXYZ>(second_pole_base, net_line.first, 102.0 / 255.0, 1.0, 0.0, "second_pole_line");
	viewer.addLine<pcl::PointXYZ, pcl::PointXYZ>(second_pole_base, pole_line.second, 102.0 / 255.0, 1.0, 0.0, "second_net_line");

	//	viewer.addSphere(pcl::PointXYZ(0.0f, 0.0f, 0.0f), 0.2, 0.5, 0.5, 0.0, "sphere");

	for (int y = 0; y < depthHeight; y++){
		for (int x = 0; x < depthWidth; x++){
			pcl::PointXYZ point;

			DepthSpacePoint depthSpacePoint = { static_cast<float>(x), static_cast<float>(y) };
			UINT16 depth = depthBuffer[y * depthWidth + x];

			// Coordinate Mapping Depth to Camera Space, and Setting PointCloud XYZ
			CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };

			if (!(pCoordinateMapper->MapDepthPointToCameraSpace(depthSpacePoint, depth, &cameraSpacePoint) == S_OK)) {
				continue;
			}

			if (std::isinf(cameraSpacePoint.X) || std::isinf(cameraSpacePoint.Y) || std::isinf(cameraSpacePoint.Z)) {
				continue;
			}
			point.x = cameraSpacePoint.X;
			point.y = cameraSpacePoint.Y;
			point.z = cameraSpacePoint.Z;
			cloud->push_back(point);
		}
	}

	transformation = calibration_processing::calculate_transformation(net_line, pole_line);

	cloud_processing(cloud, viewer);
}

inline bool point_filtering(Eigen::Vector3d& point_vector, float x, float y, float z)
{
	point_vector << x, y, z;

	point_vector = transformation * point_vector;

	// height filter
	// lower bound (filters anything below the net)
	if (point_vector(2) < 1570.0) {
		return false;
	}

	// bound filtering
	if (point_vector(0) > 3050.0 || point_vector(0) < -3050.0) {
		return false;
	}

	if (point_vector(1) > 8000.0 || point_vector(1) < 0.0) {
		return false;
	}

	return true;
}

pcl::PolygonMesh generate_quad_mesh(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud)
{
	std::vector<pcl::Vertices> polygon;
	pcl::Vertices first_triangle;
	first_triangle.vertices.push_back(0); first_triangle.vertices.push_back(1); first_triangle.vertices.push_back(2);
	polygon.push_back(first_triangle);
	pcl::Vertices second_triangle;
	second_triangle.vertices.push_back(0); second_triangle.vertices.push_back(2); second_triangle.vertices.push_back(3);
	polygon.push_back(second_triangle);

	pcl::PolygonMesh mesh;
	pcl::PCLPointCloud2 cloud2;
	pcl::toPCLPointCloud2(*cloud, cloud2);

	mesh.cloud = cloud2;
	mesh.polygons = polygon;

	return mesh;
}

enum class Point_Color {green, white, yellow, grey};

inline void create_quad(std::vector<pcl::Vertices>& v, int v1, int v2, int v3, int v4)
{
	pcl::Vertices vertex;
	vertex.vertices.push_back(v1);
	vertex.vertices.push_back(v2);
	vertex.vertices.push_back(v3);
	v.push_back(vertex);

	pcl::Vertices vertex2;
	vertex2.vertices.push_back(v2);
	vertex2.vertices.push_back(v3);
	vertex2.vertices.push_back(v4);
	v.push_back(vertex2);
}

inline pcl::PointXYZRGBA create_point(float x, float y, float z, Point_Color color)
{
	pcl::PointXYZRGBA point;
	point.x = x;
	point.y = y;
	point.z = z;

	switch (color) {
	case Point_Color::green:
		point.r = 0;
		point.g = 102;
		point.b = 0;
		point.a = 255;
		break;
	case Point_Color::white:
		point.r = 255;
		point.g = 255;
		point.b = 255;
		point.a = 255;
		break;
	case Point_Color::yellow:
		point.r = 255;
		point.g = 255;
		point.b = 0;
		point.a = 255;
		break;
	case Point_Color::grey:
		point.r = 192;
		point.g = 192;
		point.b = 192;
		point.a = 255;
		break;
	default:
		point.r = 0;
		point.g = 0;
		point.b = 0;
		point.a = 255;
		break;
	}

	return point;
}

void add_game_field_shapes(pcl::visualization::PCLVisualizer& viewer)
{
	// net
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr net_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointXYZRGBA net_bottom_left;
	net_bottom_left.x = -3050.0f; net_bottom_left.y = 6700.0f; net_bottom_left.z = 0.0f;
	net_bottom_left.r = 255; net_bottom_left.g = 255; net_bottom_left.b = 255; net_bottom_left.a = 255;
	net_cloud->points.push_back(net_bottom_left);

	pcl::PointXYZRGBA net_bottom_right;
	net_bottom_left.x = 3050.0f; net_bottom_left.y = 6700.0f; net_bottom_left.z = 0.0f;
	net_bottom_left.r = 255; net_bottom_left.g = 255; net_bottom_left.b = 255; net_bottom_left.a = 255;
	net_cloud->points.push_back(net_bottom_left);

	pcl::PointXYZRGBA net_top_right;
	net_bottom_left.x = 3050.0f; net_bottom_left.y = 6700.0f; net_bottom_left.z = 1550.0f;
	net_bottom_left.r = 255; net_bottom_left.g = 255; net_bottom_left.b = 255; net_bottom_left.a = 255;
	net_cloud->points.push_back(net_bottom_left);

	pcl::PointXYZRGBA net_top_left;
	net_bottom_left.x = -3050.0f; net_bottom_left.y = 6700.0f; net_bottom_left.z = 1550.0f;
	net_bottom_left.r = 255; net_bottom_left.g = 255; net_bottom_left.b = 255; net_bottom_left.a = 255;
	net_cloud->points.push_back(net_bottom_left);

	viewer.addPolygonMesh(generate_quad_mesh(net_cloud), "net mesh");

	// field

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr field_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

	// left right down up

	// lvl 0 segment 0
	field_cloud->points.push_back(create_point(-3050.0f, 0, 0, Point_Color::white));
	field_cloud->points.push_back(create_point(3050.0f, 0, 0, Point_Color::white));
	field_cloud->points.push_back(create_point(-3050.0f, 40.0f, 0, Point_Color::white));
	field_cloud->points.push_back(create_point(3050.0f, 40.0f, 0, Point_Color::white));

	// lvl 1 segment 0
	// -3050 40 (point 2)
	field_cloud->points.push_back(create_point(-3010.0f, 40.0f, 0, Point_Color::white));
	field_cloud->points.push_back(create_point(-3050.0f, 760.0f, 0, Point_Color::white));
	field_cloud->points.push_back(create_point(-3010.0f, 760.0f, 0, Point_Color::white));

	// lvl 1 segment 1
	field_cloud->points.push_back(create_point(-3010.0f, 40.0f, 0, Point_Color::green));
	field_cloud->points.push_back(create_point(-2590.0f, 40.0f, 0, Point_Color::green));
	field_cloud->points.push_back(create_point(-3010.0f, 760.0f, 0, Point_Color::green));
	field_cloud->points.push_back(create_point(-2590.0f, 760.0f, 0, Point_Color::green));

	// lvl 1 segment 2
	field_cloud->points.push_back(create_point(-2590.0f, 40.0f, 0, Point_Color::white));
	field_cloud->points.push_back(create_point(-2550.0f, 40.0f, 0, Point_Color::white));
	field_cloud->points.push_back(create_point(-2590.0f, 760.0f, 0, Point_Color::white));
	field_cloud->points.push_back(create_point(-2550.0f, 760.0f, 0, Point_Color::white));

	// lvl 1 segment 3
	field_cloud->points.push_back(create_point(-2550.0f, 40.0f, 0, Point_Color::green));
	field_cloud->points.push_back(create_point(-20.0f, 40.0f, 0, Point_Color::green));
	field_cloud->points.push_back(create_point(-2550.0f, 760.0f, 0, Point_Color::green));
	field_cloud->points.push_back(create_point(-20.0f, 760.0f, 0, Point_Color::green));

	// lvl 1 segment 4
	field_cloud->points.push_back(create_point(-20.0f, 40.0f, 0, Point_Color::white));
	field_cloud->points.push_back(create_point(20.0f, 40.0f, 0, Point_Color::white));
	field_cloud->points.push_back(create_point(-20.0f, 760.0f, 0, Point_Color::white));
	field_cloud->points.push_back(create_point(20.0f, 760.0f, 0, Point_Color::white));

	// lvl 1 segment 5
	field_cloud->points.push_back(create_point(20.0f, 40.0f, 0, Point_Color::green));
	field_cloud->points.push_back(create_point(2550.0f, 40.0f, 0, Point_Color::green));
	field_cloud->points.push_back(create_point(20.0f, 760.0f, 0, Point_Color::green));
	field_cloud->points.push_back(create_point(2550.0f, 760.0f, 0, Point_Color::green));

	// lvl 1 segment 6
	field_cloud->points.push_back(create_point(2550.0f, 40.0f, 0, Point_Color::white));
	field_cloud->points.push_back(create_point(2590.0f, 40.0f, 0, Point_Color::white));
	field_cloud->points.push_back(create_point(2550.0f, 760.0f, 0, Point_Color::white));
	field_cloud->points.push_back(create_point(2590.0f, 760.0f, 0, Point_Color::white));

	// lvl 1 segment 7
	field_cloud->points.push_back(create_point(2590.0f, 40.0f, 0, Point_Color::green));
	field_cloud->points.push_back(create_point(3010.0f, 40.0f, 0, Point_Color::green));
	field_cloud->points.push_back(create_point(2590.0f, 760.0f, 0, Point_Color::green));
	field_cloud->points.push_back(create_point(3010.0f, 760.0f, 0, Point_Color::green));

	// lvl 1 segment 8
	field_cloud->points.push_back(create_point(3010.0f, 40.0f, 0, Point_Color::white));
	// 3050 40 (point 3)
	field_cloud->points.push_back(create_point(3010.0f, 760.0f, 0, Point_Color::white));
	field_cloud->points.push_back(create_point(3050.0f, 760.0f, 0, Point_Color::white));

	// lvl 2 segment 0
	// -3050 760 (point 5)
	// 3050, 760 (point 37)
	field_cloud->points.push_back(create_point(-3050.0f, 800.0f, 0, Point_Color::white));
	field_cloud->points.push_back(create_point(3050.0f, 800.0f, 0, Point_Color::white)); // point 39

	// lvl 3 segment 0
	// -3050, 800 (point 38)
	field_cloud->points.push_back(create_point(-3010.0f, 800.0f, 0, Point_Color::white));
	field_cloud->points.push_back(create_point(-3050.0f, 4680.0f, 0, Point_Color::white));
	field_cloud->points.push_back(create_point(-3010.0f, 4680.0f, 0, Point_Color::white));

	// lvl 3 segment 1
	field_cloud->points.push_back(create_point(-3010.0f, 800.0f, 0, Point_Color::green));
	field_cloud->points.push_back(create_point(-2590.0f, 800.0f, 0, Point_Color::green));
	field_cloud->points.push_back(create_point(-3010.0f, 4680.0f, 0, Point_Color::green));
	field_cloud->points.push_back(create_point(-2590.0f, 4680.0f, 0, Point_Color::green));

	// lvl 3 segment 2
	field_cloud->points.push_back(create_point(-2590.0f, 800.0f, 0, Point_Color::white));
	field_cloud->points.push_back(create_point(-2550.0f, 800.0f, 0, Point_Color::white));
	field_cloud->points.push_back(create_point(-2590.0f, 4680.0f, 0, Point_Color::white));
	field_cloud->points.push_back(create_point(-2550.0f, 4680.0f, 0, Point_Color::white));

	// lvl 3 segment 3
	field_cloud->points.push_back(create_point(-2550.0f, 800.0f, 0, Point_Color::green));
	field_cloud->points.push_back(create_point(-20.0f, 800.0f, 0, Point_Color::green));
	field_cloud->points.push_back(create_point(-2550.0f, 4680.0f, 0, Point_Color::green));
	field_cloud->points.push_back(create_point(-20.0f, 4680.0f, 0, Point_Color::green));

	// lvl 3 segment 4
	field_cloud->points.push_back(create_point(-20.0f, 800.0f, 0, Point_Color::white));
	field_cloud->points.push_back(create_point(20.0f, 800.0f, 0, Point_Color::white));
	field_cloud->points.push_back(create_point(-20.0f, 4680.0f, 0, Point_Color::white));
	field_cloud->points.push_back(create_point(20.0f, 4680.0f, 0, Point_Color::white));

	// lvl 3 segment 5a
	field_cloud->points.push_back(create_point(20.0f, 800.0f, 0, Point_Color::green));
	field_cloud->points.push_back(create_point(2550.0f, 800.0f, 0, Point_Color::green));
	field_cloud->points.push_back(create_point(20.0f, 1505.0f, 0, Point_Color::green)); // point 61
	field_cloud->points.push_back(create_point(2550.0f, 1505.0f, 0, Point_Color::green));

	// lvl 3 segment 5b
	// 20, 1505 (point 61)
	field_cloud->points.push_back(create_point(550.0f, 1505.0f, 0, Point_Color::green));
	field_cloud->points.push_back(create_point(20.0f, 2755.0f, 0, Point_Color::green));
	field_cloud->points.push_back(create_point(550.0f, 2755.0f, 0, Point_Color::green));

	// lvl 3 segment 5c (yellow zone)
	field_cloud->points.push_back(create_point(550.0f, 1505.0f, 0, Point_Color::yellow));
	field_cloud->points.push_back(create_point(2550.0f, 1505.0f, 0, Point_Color::yellow));
	field_cloud->points.push_back(create_point(550.0f, 2755.0f, 0, Point_Color::yellow));
	field_cloud->points.push_back(create_point(2550.0f, 2755.0f, 0, Point_Color::yellow));

	// lvl 3 segment 5d
	// 20, 2755 (point 64)
	field_cloud->points.push_back(create_point(2550.0f, 2755.0f, 0, Point_Color::green));
	field_cloud->points.push_back(create_point(20.0f, 4680.0f, 0, Point_Color::green));
	field_cloud->points.push_back(create_point(2550.0f, 4680.0f, 0, Point_Color::green));

	// lvl 3 segment 6
	field_cloud->points.push_back(create_point(2550.0f, 800.0f, 0, Point_Color::white));
	field_cloud->points.push_back(create_point(2590.0f, 800.0f, 0, Point_Color::white));
	field_cloud->points.push_back(create_point(2550.0f, 4680.0f, 0, Point_Color::white));
	field_cloud->points.push_back(create_point(2590.0f, 4680.0f, 0, Point_Color::white));

	// lvl 3 segment 7

	field_cloud->points.push_back(create_point(2590.0f, 800.0f, 0, Point_Color::green));
	field_cloud->points.push_back(create_point(3010.0f, 800.0f, 0, Point_Color::green));
	field_cloud->points.push_back(create_point(2590.0f, 4680.0f, 0, Point_Color::green));
	field_cloud->points.push_back(create_point(3010.0f, 4680.0f, 0, Point_Color::green));

	// lvl 3 segment 8
	field_cloud->points.push_back(create_point(3010.0f, 800.0f, 0, Point_Color::white));
	// 3050, 800 (point 39)
	field_cloud->points.push_back(create_point(3010.0f, 4680.0f, 0, Point_Color::white));
	field_cloud->points.push_back(create_point(3050.0f, 4680.0f, 0, Point_Color::white)); // point 83

	// lvl 4 segment 0
	// -3050, 4680 (point 41)
	// 3050, 4680 (point 83)
	field_cloud->points.push_back(create_point(-3050.0f, 4720.0f, 0, Point_Color::white));
	field_cloud->points.push_back(create_point(3050.0f, 4720.0f, 0, Point_Color::white));

	// lvl 5 segment 0
	// -3050, 4720 (point 84)
	field_cloud->points.push_back(create_point(-3010.0f, 4720.0f, 0, Point_Color::white));
	field_cloud->points.push_back(create_point(-3050.0f, 6680.0f, 0, Point_Color::white));
	field_cloud->points.push_back(create_point(-3010.0f, 6680.0f, 0, Point_Color::white));

	// lvl 5 segment 1
	field_cloud->points.push_back(create_point(-3010.0f, 4720.0f, 0, Point_Color::green));
	field_cloud->points.push_back(create_point(-2590.0f, 4720.0f, 0, Point_Color::green));
	field_cloud->points.push_back(create_point(-3010.0f, 6680.0f, 0, Point_Color::green));
	field_cloud->points.push_back(create_point(-2590.0f, 6680.0f, 0, Point_Color::green));

	// lvl 5 segment 2
	field_cloud->points.push_back(create_point(-2590.0f, 4720.0f, 0, Point_Color::white));
	field_cloud->points.push_back(create_point(-2550.0f, 4720.0f, 0, Point_Color::white));
	field_cloud->points.push_back(create_point(-2590.0f, 6680.0f, 0, Point_Color::white));
	field_cloud->points.push_back(create_point(-2550.0f, 6680.0f, 0, Point_Color::white));

	// lvl 5 segment 3
	field_cloud->points.push_back(create_point(-2550.0f, 4720.0f, 0, Point_Color::green));
	field_cloud->points.push_back(create_point(2550.0f, 4720.0f, 0, Point_Color::green));
	field_cloud->points.push_back(create_point(-2550.0f, 6680.0f, 0, Point_Color::green));
	field_cloud->points.push_back(create_point(2550.0f, 6680.0f, 0, Point_Color::green));

	// lvl 5 segment 4
	field_cloud->points.push_back(create_point(2550.0f, 4720.0f, 0, Point_Color::white));
	field_cloud->points.push_back(create_point(2590.0f, 4720.0f, 0, Point_Color::white));
	field_cloud->points.push_back(create_point(2550.0f, 6680.0f, 0, Point_Color::white));
	field_cloud->points.push_back(create_point(2590.0f, 6680.0f, 0, Point_Color::white));

	// lvl 5 segment 5
	field_cloud->points.push_back(create_point(2590.0f, 4720.0f, 0, Point_Color::green));
	field_cloud->points.push_back(create_point(3010.0f, 4720.0f, 0, Point_Color::green));
	field_cloud->points.push_back(create_point(2590.0f, 6680.0f, 0, Point_Color::green));
	field_cloud->points.push_back(create_point(3010.0f, 6680.0f, 0, Point_Color::green));

	// lvl 5 segment 6
	field_cloud->points.push_back(create_point(3010.0f, 4720.0f, 0, Point_Color::white));
	// 3050, 4720 (point 85)
	field_cloud->points.push_back(create_point(3010.0f, 6680.0f, 0, Point_Color::white));
	field_cloud->points.push_back(create_point(3050.0f, 6680.0f, 0, Point_Color::white)); //point 111

	// lvl 6 segment 0 (aka the net)

	// -3050, 6680 (point 87)
	// 3050, 6680 (point 111)
	field_cloud->points.push_back(create_point(-3050.0f, 6720.0f, 0, Point_Color::white));
	field_cloud->points.push_back(create_point(3050.0f, 6720.0f, 0, Point_Color::white));

	// lvl 7 segment 0
	// grey
	field_cloud->points.push_back(create_point(-3050.0f, 6720.0f, 0, Point_Color::grey));
	field_cloud->points.push_back(create_point(3050.0f, 6720.0f, 0, Point_Color::grey));
	field_cloud->points.push_back(create_point(-3050.0f, 13400.0f, 0, Point_Color::grey));
	field_cloud->points.push_back(create_point(3050.0f, 13400.0f, 0, Point_Color::grey));

	std::vector<pcl::Vertices> polygon;
	create_quad(polygon, 0, 1, 2, 3);
	create_quad(polygon, 2, 4, 5, 6);
	create_quad(polygon, 7, 8, 9, 10);
	create_quad(polygon, 11, 12, 13, 14);
	create_quad(polygon, 15, 16, 17, 18);
	create_quad(polygon, 19, 20, 21, 22);
	create_quad(polygon, 23, 24, 25, 26);
	create_quad(polygon, 27, 28, 29, 30);
	create_quad(polygon, 31, 32, 33, 34);
	create_quad(polygon, 35, 3, 36, 37);
	create_quad(polygon, 5, 37, 38, 39);
	create_quad(polygon, 38, 40, 41, 42);
	create_quad(polygon, 43, 44, 45, 46);
	create_quad(polygon, 47, 48, 49, 50);
	create_quad(polygon, 51, 52, 53, 54);
	create_quad(polygon, 55, 56, 57, 58);
	create_quad(polygon, 59, 60, 61, 62);
	create_quad(polygon, 61, 63, 64, 65);
	create_quad(polygon, 66, 67, 68, 69);
	create_quad(polygon, 64, 70, 71, 72);
	create_quad(polygon, 73, 74, 75, 76);
	create_quad(polygon, 77, 78, 79, 80);
	create_quad(polygon, 81, 39, 82, 83);
	create_quad(polygon, 41, 83, 84, 85);
	create_quad(polygon, 84, 86, 87, 88);
	create_quad(polygon, 89, 90, 91, 92);
	create_quad(polygon, 93, 94, 95, 96);
	create_quad(polygon, 97, 98, 99, 100);
	create_quad(polygon, 101, 102, 103, 104);
	create_quad(polygon, 105, 106, 107, 108);
	create_quad(polygon, 109, 85, 110, 111);
	create_quad(polygon, 87, 111, 112, 113);
	create_quad(polygon, 114, 115, 116, 117);

	pcl::PolygonMesh field_mesh;
	pcl::PCLPointCloud2 field_cloud2;
	pcl::toPCLPointCloud2(*field_cloud, field_cloud2);

	field_mesh.cloud = field_cloud2;
	field_mesh.polygons = polygon;

	viewer.addPolygonMesh(field_mesh, "field mesh");

	/*

	pcl::PointXYZRGBA field_near_left;
	field_near_left.x = -3050.0f; field_near_left.y = 0.0f; field_near_left.z = 0.0f;
	field_near_left.r = 255; field_near_left.g = 255; field_near_left.b = 255; field_near_left.a = 255;
	field_cloud->points.push_back(field_near_left);

	pcl::PointXYZRGBA field_near_right;
	field_near_right.x = 3050.0f; field_near_right.y = 0.0f; field_near_right.z = 0.0f;
	field_near_right.r = 255; field_near_right.g = 255; field_near_right.b = 255; field_near_right.a = 255;
	field_cloud->points.push_back(field_near_right);

	pcl::PointXYZRGBA field_far_right;
	field_far_right.x = 3050.0f; field_far_right.y = 13400.0f; field_far_right.z = 0.0f;
	field_far_right.r = 255; field_far_right.g = 255; field_far_right.b = 255; field_far_right.a = 255;
	field_cloud->points.push_back(field_far_right);

	pcl::PointXYZRGBA field_far_left;
	field_far_left.x = -3050.0f; field_far_left.y = 13400.0f; field_far_left.z = 0.0f;
	field_far_left.r = 255; field_far_left.g = 255; field_far_left.b = 255; field_far_left.a = 255;
	field_cloud->points.push_back(field_far_left);

	viewer.addPolygonMesh(generate_quad_mesh(field_cloud), "field mesh");

	*/

	/*

	// near yellow zone
	
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr near_yellow_zone_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

	pcl::PointXYZRGBA near_yellow_zone_bottom_left;
	near_yellow_zone_bottom_left.x = -2570.0f; near_yellow_zone_bottom_left.y = 6700.0f - 5195.0f; near_yellow_zone_bottom_left.z = 10.0f;
	near_yellow_zone_bottom_left.r = 255; near_yellow_zone_bottom_left.g = 255; near_yellow_zone_bottom_left.b = 0; near_yellow_zone_bottom_left.a = 255;
	near_yellow_zone_cloud->points.push_back(near_yellow_zone_bottom_left);

	pcl::PointXYZRGBA near_yellow_zone_bottom_right;
	near_yellow_zone_bottom_right.x = -550.0f; near_yellow_zone_bottom_right.y = 6700.0f - 5195.0f; near_yellow_zone_bottom_right.z = 10.0f;
	near_yellow_zone_bottom_right.r = 255; near_yellow_zone_bottom_right.g = 255; near_yellow_zone_bottom_right.b = 0; near_yellow_zone_bottom_right.a = 255;
	near_yellow_zone_cloud->points.push_back(near_yellow_zone_bottom_right);

	pcl::PointXYZRGBA near_yellow_zone_top_right;
	near_yellow_zone_top_right.x = -550.0f; near_yellow_zone_top_right.y = 6700.0f - 3945.0f; near_yellow_zone_top_right.z = 10.0f;
	near_yellow_zone_top_right.r = 255; near_yellow_zone_top_right.g = 255; near_yellow_zone_top_right.b = 0; near_yellow_zone_top_right.a = 255;
	near_yellow_zone_cloud->points.push_back(near_yellow_zone_top_right);

	pcl::PointXYZRGBA near_yellow_zone_top_left;
	near_yellow_zone_top_left.x = -2570.0f; near_yellow_zone_top_left.y = 6700.0f - 3945.0f; near_yellow_zone_top_left.z = 10.0f;
	near_yellow_zone_top_left.r = 255; near_yellow_zone_top_left.g = 255; near_yellow_zone_top_left.b = 0; near_yellow_zone_top_left.a = 255;
	near_yellow_zone_cloud->points.push_back(near_yellow_zone_top_left);

	viewer.addPolygonMesh(generate_quad_mesh(near_yellow_zone_cloud), "near yellow zone mesh");
	
	*/

}

void point_cloud_rendering(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<UINT16>& depthBuffer, ICoordinateMapper*& pCoordinateMapper, int depthWidth, int depthHeight, pcl::visualization::PCLVisualizer& viewer)
{
	if (calibration_edge_trigger) {
		viewer.removeAllShapes();
		// add shapes for game field
		add_game_field_shapes(viewer);
		calibration_edge_trigger = false;
	}
	if (point_cloud_tracking::icp_depth_image_processing(cloud, depthBuffer, pCoordinateMapper, depthWidth, depthHeight, viewer, transformation)) {

	}
	else {
		for (int y = 0; y < depthHeight; y++){
			for (int x = 0; x < depthWidth; x++){
				pcl::PointXYZ point;

				DepthSpacePoint depthSpacePoint = { static_cast<float>(x), static_cast<float>(y) };
				UINT16 depth = depthBuffer[y * depthWidth + x];

				// Coordinate Mapping Depth to Camera Space, and Setting PointCloud XYZ
				CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };
				if (!(pCoordinateMapper->MapDepthPointToCameraSpace(depthSpacePoint, depth, &cameraSpacePoint) == S_OK)) {
					continue;
				}
				if (std::isinf(cameraSpacePoint.X) || std::isinf(cameraSpacePoint.Y) || std::isinf(cameraSpacePoint.Z)) {
					continue;
				}

				// Coordinate-based filtering
				Eigen::Vector3d point_vector;

				if (!point_filtering(point_vector, cameraSpacePoint.X, cameraSpacePoint.Y, cameraSpacePoint.Z))
					continue;

				point.x = point_vector.x();//cameraSpacePoint.X;
				point.y = point_vector.y();//cameraSpacePoint.Y;
				point.z = point_vector.z();//cameraSpacePoint.Z;
				cloud->push_back(point);
			}
		}
		cloud_processing(cloud, viewer);
	}
}

void keyboard_callback(const pcl::visualization::KeyboardEvent& event, void*)
{
	if (event.getKeyCode() && event.keyDown()){
		std::cout << "Key : " << event.getKeyCode() << std::endl;
		if (event.getKeyCode() == '\t') {
			if (calibration_mode) {
				std::cout << "Calibrate mode off" << std::endl;
				calibration_mode = false;
			}
			else {
				std::cout << "Calibrate mode on" << std::endl;
				calibration_mode = true;
			}
			calibration_edge_trigger = true;
		}

		switch (event.getKeyCode()) {
			case '-': point_cloud_tracking::decrease_max(); break;
			case '=': point_cloud_tracking::increase_max(); break;
			case '_': point_cloud_tracking::decrease_min(); break;
			case '+': point_cloud_tracking::increase_min(); break;
		}

		if (calibration_mode) {
			switch (event.getKeyCode()) {
				case 'j': net_line.first.x -= 0.01f; break;
				case 'J': net_line.first.x += 0.01f; break;
				case 'k': net_line.first.y -= 0.01f; break;
				case 'K': net_line.first.y += 0.01f; break;
				case 'l': net_line.first.z -= 0.01f; break;
				case 'L': net_line.first.z += 0.01f; break;
				case 'b': net_line.second.x -= 0.01f; pole_line.first.x -= 0.01f; break;
				case 'B': net_line.second.x += 0.01f; pole_line.first.x += 0.01f; break;
				case 'n': net_line.second.y -= 0.01f; pole_line.first.y -= 0.01f; break;
				case 'N': net_line.second.y += 0.01f; pole_line.first.y += 0.01f; break;
				case 'm': net_line.second.z -= 0.01f; pole_line.first.z -= 0.01f; break;
				case 'M': net_line.second.z += 0.01f; pole_line.first.z += 0.01f; break;
				case 'a': pole_line.second.x -= 0.01f; break;
				case 'A': pole_line.second.x += 0.01f; break;
				case 's': pole_line.second.y -= 0.01f; break;
				case 'S': pole_line.second.y += 0.01f; break;
				case 'd': pole_line.second.z -= 0.01f; break;
				case 'D': pole_line.second.z += 0.01f; break;
			}
			std::cout << "Net First point: " << net_line.first << " Second point: " << net_line.second << std::endl;
			std::cout << "Pole First point: " << pole_line.first << " Second point: " << pole_line.second << std::endl;

		}
	}
}

void mouse_callback(const pcl::visualization::MouseEvent& event, void*)
{
	if (event.getType() == pcl::visualization::MouseEvent::MouseButtonPress && event.getButton() == pcl::visualization::MouseEvent::LeftButton){
		std::cout << "Mouse : " << event.getX() << ", " << event.getY() << std::endl;
		std::cout << "Net Distance: " << std::sqrt(std::pow(net_line.second.x - net_line.first.x, 2.0) + std::pow(net_line.second.y - net_line.first.y, 2.0) + std::pow(net_line.second.z - net_line.first.z, 2.0)) << std::endl;
	}
}

int _tmain(int argc, _TCHAR* argv[])
{
	// change to correct directory and load points into program from file if they exist
	TCHAR path[MAX_PATH] = { 0 };
	GetModuleFileName(NULL, path, MAX_PATH);
	PathRemoveFileSpec(path);
	SetCurrentDirectory(path);

	// read in data from file
	{
		std::ifstream file;
		file.open("saved_points.txt");
		std::vector<float> point_vector;

		if (file.is_open()) {
			OutputDebugString(_T("Loading saved points\n"));
			std::string line;
			while (std::getline(file, line)) {
				point_vector.push_back(std::stof(line));
			}
		}
		// add default points if loading fails;
		if (point_vector.size() != 9) {
			point_vector.clear();

			// point 1
			point_vector.push_back(0.94f);
			point_vector.push_back(0.32f);
			point_vector.push_back(1.99f);

			// point 2
			point_vector.push_back(-2.16f);
			point_vector.push_back(-0.07f);
			point_vector.push_back(7.13f);

			// point 3
			point_vector.push_back(-2.21f);
			point_vector.push_back(-1.59f);
			point_vector.push_back(6.99f);
		}

		pcl::PointXYZ pt1(point_vector[0], point_vector[1], point_vector[2]);
		pcl::PointXYZ pt2(point_vector[3], point_vector[4], point_vector[5]);
		pcl::PointXYZ pt3(point_vector[6], point_vector[7], point_vector[8]);

		net_line = create_line_endpoints(pt1, pt2);
		pole_line = create_line_endpoints(pt2, pt3);
	}

	// Create Sensor Instance
	IKinectSensor* pSensor;
	HRESULT hResult = S_OK;
	hResult = GetDefaultKinectSensor( &pSensor );
	if( FAILED( hResult ) ){
		std::cerr << "Error : GetDefaultKinectSensor" << std::endl;
		return -1;
	}

	// Open Sensor
	hResult = pSensor->Open();
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::Open()" << std::endl;
		return -1;
	}

	// Retrieved Coordinate Mapper
	ICoordinateMapper* pCoordinateMapper;
	hResult = pSensor->get_CoordinateMapper( &pCoordinateMapper );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::get_CoordinateMapper()" << std::endl;
		return -1;
	}

	// Retrieved Color Frame Source
	IColorFrameSource* pColorSource;
	hResult = pSensor->get_ColorFrameSource( &pColorSource );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::get_ColorFrameSource()" << std::endl;
		return -1;
	}

	// Retrieved Depth Frame Source
	IDepthFrameSource* pDepthSource;
	hResult = pSensor->get_DepthFrameSource( &pDepthSource );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::get_DepthFrameSource()" << std::endl;
		return -1;
	}

	// Open Color Frame Reader
	IColorFrameReader* pColorReader;
	hResult = pColorSource->OpenReader( &pColorReader );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IColorFrameSource::OpenReader()" << std::endl;
		return -1;
	}

	// Open Depth Frame Reader
	IDepthFrameReader* pDepthReader;
	hResult = pDepthSource->OpenReader( &pDepthReader );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IDepthFrameSource::OpenReader()" << std::endl;
		return -1;
	}

	// Retrieved Color Frame Size
	IFrameDescription* pColorDescription;
	hResult = pColorSource->get_FrameDescription( &pColorDescription );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IColorFrameSource::get_FrameDescription()" << std::endl;
		return -1;
	}
	int colorWidth = 0;
	int colorHeight = 0;
	pColorDescription->get_Width( &colorWidth ); // 1920
	pColorDescription->get_Height( &colorHeight ); // 1080

	// To Reserve Color Frame Buffer
	std::vector<RGBQUAD> colorBuffer( colorWidth * colorHeight );

	// Retrieved Depth Frame Size
	IFrameDescription* pDepthDescription;
	hResult = pDepthSource->get_FrameDescription( &pDepthDescription );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IDepthFrameSource::get_FrameDescription()" << std::endl;
		return -1;
	}
	int depthWidth = 0;
	int depthHeight = 0;
	pDepthDescription->get_Width( &depthWidth ); // 512
	pDepthDescription->get_Height( &depthHeight ); // 424

	// To Reserve Depth Frame Buffer
	std::vector<UINT16> depthBuffer( depthWidth * depthHeight );

	// Create Cloud Viewer
	pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");

	typedef std::conditional <
		std::chrono::high_resolution_clock::is_steady,
		std::chrono::high_resolution_clock,
		std::chrono::steady_clock > ::type clock;

	auto current_time = clock::now();

	unsigned char counter = 0;

	viewer.registerKeyboardCallback(keyboard_callback);
	viewer.registerMouseCallback(mouse_callback);
	viewer.getRenderWindow()->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->SetParallelProjection(1);

	while( !viewer.wasStopped() ){
		viewer.spinOnce();

		// Acquire Latest Depth Frame
		IDepthFrame* pDepthFrame = nullptr;
		hResult = pDepthReader->AcquireLatestFrame( &pDepthFrame );
		if( SUCCEEDED( hResult ) ){
			// Retrieved Depth Data
			hResult = pDepthFrame->CopyFrameDataToArray( static_cast<unsigned int>(depthBuffer.size()), &depthBuffer[0] );
			if( FAILED( hResult ) ){
				std::cerr << "Error : IDepthFrame::CopyFrameDataToArray()" << std::endl;
			}
		}
		SafeRelease( pDepthFrame );

		pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>());
		// Create Point Cloud
		pointcloud->width = static_cast<uint32_t>(depthWidth);
		pointcloud->height = static_cast<uint32_t>(depthHeight);
		pointcloud->is_dense = true;

		if (calibration_mode) {
			point_cloud_calibration(pointcloud, depthBuffer, pCoordinateMapper, depthWidth, depthHeight, viewer);
		}
		else {
			point_cloud_rendering(pointcloud, depthBuffer, pCoordinateMapper, depthWidth, depthHeight, viewer);
		}
		// Input Key ( Exit ESC key )
		if( GetKeyState( VK_ESCAPE ) < 0 ){
			break;
		}
		++counter;
		if (counter == 60) {
			counter = 0;
			auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(clock::now() - current_time).count();
			std::basic_ostringstream<TCHAR> oss;
			oss << "Framerate: " << 60.0f * 1000.0f / duration << std::endl;
			OutputDebugString(oss.str().c_str());
			current_time = clock::now();
		}
	}

	// End Processing

	// write out data
	{
		std::ofstream file;
		file.open("saved_points.txt");

		file << net_line.first.x << std::endl;
		file << net_line.first.y << std::endl;
		file << net_line.first.z << std::endl;

		file << net_line.second.x << std::endl;
		file << net_line.second.y << std::endl;
		file << net_line.second.z << std::endl;

		file << pole_line.second.x << std::endl;
		file << pole_line.second.y << std::endl;
		file << pole_line.second.z << std::endl;

		OutputDebugString(_T("Saving points\n"));
	}

	SafeRelease( pColorSource );
	SafeRelease( pDepthSource );
	SafeRelease( pColorReader );
	SafeRelease( pDepthReader );
	SafeRelease( pColorDescription );
	SafeRelease( pDepthDescription );
	SafeRelease( pCoordinateMapper );
	if( pSensor ){
		pSensor->Close();
	}
	SafeRelease( pSensor );

	return 0;
}