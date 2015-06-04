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
#include <chrono>
#include <limits>
#include <cmath>
#include <tuple>
#include <concurrent_vector.h>

namespace {
	float min_y_cutoff = 5.0f;
	float max_y_cutoff = std::numeric_limits<float>::infinity();
	float net_angle = 0.0f;
	bool calibration_mode = true;
	bool calibration_edge_trigger = true;
	std::pair<pcl::PointXYZ, pcl::PointXYZ> create_net_line_endpoints() {
		pcl::PointXYZ start_point = pcl::PointXYZ(1.22f, 0.37f, 1.76f);

		pcl::PointXYZ end_point = pcl::PointXYZ(0.14f, -0.20f, 7.68f);
		return std::make_pair(start_point, end_point);
	}

	std::pair<pcl::PointXYZ, pcl::PointXYZ> create_pole_line_endpoints() {
		pcl::PointXYZ start_point = pcl::PointXYZ(0.14f, -0.20f, 7.68f);

		pcl::PointXYZ end_point = pcl::PointXYZ(0.16f, -1.62f, 7.42f);

		return std::make_pair(start_point, end_point);
	}

	std::pair<pcl::PointXYZ, pcl::PointXYZ> net_line = create_net_line_endpoints();
	std::pair<pcl::PointXYZ, pcl::PointXYZ> pole_line = create_pole_line_endpoints();
	Eigen::Matrix3f coord_conversion_matrix = Eigen::Matrix3f();
	Eigen::Affine3d transformation;
}

template <class Interface>
inline void SafeRelease( Interface *& pInterfaceToRelease )
{
	if( pInterfaceToRelease != NULL ){
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

void point_cloud_calibration(pcl::PointCloud<pcl::PointXYZ>* cloud, std::vector<UINT16>& depthBuffer, ICoordinateMapper*& pCoordinateMapper, int depthWidth, int depthHeight, pcl::visualization::PCLVisualizer& viewer)
{
	viewer.removeShape("net_line");
	viewer.removeShape("pole_line");
	viewer.removeShape("pole_line2");
	//	viewer.removeShape("sphere");
	viewer.addLine<pcl::PointXYZ, pcl::PointXYZ>(net_line.first, net_line.second, "net_line");
	viewer.addLine<pcl::PointXYZ, pcl::PointXYZ>(pole_line.first, pole_line.second, "pole_line");
	//	viewer.addSphere(pcl::PointXYZ(0.0f, 0.0f, 0.0f), 0.2, 0.5, 0.5, 0.0, "sphere");

	// calculate translation matrix

	Eigen::Vector3d net_line_first, net_line_second, pole_line_second;

	net_line_first << net_line.first.x, net_line.first.y, net_line.first.z;
	net_line_second << net_line.second.x, net_line.second.y, net_line.second.z;
	pole_line_second << pole_line.second.x, pole_line.second.y, pole_line.second.z;

	// change origin to net_line_second
	Eigen::Vector3d net, height, back;
	net = net_line_first - net_line_second;
	height = pole_line_second - net_line_second;

	back = net.cross(height);
	back.normalize();
	Eigen::Affine3d t(Eigen::Scaling(6.7 * net.norm() / 6.1));
	back = t * back;

	// find origin
	Eigen::Vector3d origin = back + 0.5 * net + height;

	// change origin to origin
	Eigen::Vector3d net2 = net - origin;
	Eigen::Vector3d height2 = height - origin;
	Eigen::Vector3d net_height = -origin;

	transformation = Eigen::Affine3d::Identity();
	transformation.translation() = -net_line_second - origin;

	// test translation by applying it

	Eigen::Vector3d net_test = transformation * net_line_first;
	Eigen::Vector3d height_test = transformation * net_line_second;
	Eigen::Vector3d net_height_test = transformation * pole_line_second;

	/*
	std::cout << "Manual: ";
	std::cout << net2 << " " << height2 << " " << net_height << " " << std::endl;
	std::cout << "Affine: ";
	std::cout << net_test << " " << height_test << " " << net_height_test << " " << std::endl;
	std::cout << "Affine Transformation: ";
	std::cout << transformation.matrix() << std::endl;
	*/
	for (int y = 0; y < depthHeight; y++){
		for (int x = 0; x < depthWidth; x++){
			pcl::PointXYZ point;

			DepthSpacePoint depthSpacePoint = { static_cast<float>(x), static_cast<float>(y) };
			UINT16 depth = depthBuffer[y * depthWidth + x];

			// Coordinate Mapping Depth to Camera Space, and Setting PointCloud XYZ
			CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };
			pCoordinateMapper->MapDepthPointToCameraSpace(depthSpacePoint, depth, &cameraSpacePoint);
			if (std::isinf(cameraSpacePoint.X) || std::isinf(cameraSpacePoint.Y) || std::isinf(cameraSpacePoint.Z)) {
				continue;
			}
			point.x = cameraSpacePoint.X;
			point.y = cameraSpacePoint.Y;
			point.z = cameraSpacePoint.Z;
			cloud->push_back(point);
		}
	}
	/*
	// find origin point in current coordinate system
	// calculate midpoint of the net
	Eigen::Vector2d net_midpoint;
	net_midpoint << (net_line.second.x + net_line.first.x) / 2.0, (net_line.second.y + net_line.second.x) / 2.0;

	// find perpendicular line slope
	double perpendicular_line_slope = -1.0 / ((net_line.second.y - net_line.first.y) / (net_line.second.x - net_line.first.x));
	// find length to origin
	double length_to_origin = 6.7 * std::sqrt(std::pow(net_line.second.x - net_line.first.x, 2.0) + std::pow(net_line.second.y - net_line.first.y, 2.0) + std::pow(net_line.second.z - net_line.first.z, 2.0)) / 6.1;
	// complex equation to find x and y coordinate
	double x = (std::pow(perpendicular_line_slope, 2.0) * net_midpoint(0) + length_to_origin * std::sqrt(std::pow(perpendicular_line_slope, 2.0) + 1.0) + net_midpoint(0)) / (std::pow(perpendicular_line_slope, 2.0) + 1.0);
	double y = (std::pow(perpendicular_line_slope, 2.0) * net_midpoint(1) + length_to_origin * std::sqrt(std::pow(perpendicular_line_slope, 4.0) + std::pow(perpendicular_line_slope, 2.0)) + net_midpoint(1)) / (std::pow(perpendicular_line_slope, 2.0) + 1.0);
	*/


	Eigen::Matrix3d kinect_basis;
	/*	
		kinect_basis << net_line.first.x, net_line.second.x, pole_line.second.x,
		net_line.first.y, net_line.second.y, pole_line.second.y,
		net_line.first.z, net_line.second.z, pole_line.second.z;
	*/
	kinect_basis << net2, net_height, height2;

	Eigen::Matrix3d gyro_basis;
	gyro_basis << -3050.0, 3050.0, 3050.0,
		6700.0, 6700.0, 6700.0,
		1550.0, 1550.0, 0.0;

	transformation = gyro_basis * kinect_basis.inverse() * transformation;
}

bool point_filtering(Eigen::Vector3d& point_vector, float x, float y, float z)
{
	point_vector << x, y, z;

	point_vector = transformation * point_vector;

	// height filter
	// lower bound (filters anything below the net)
	if (point_vector(2) < 1570.0) {
		return false;
	}
	// upper bound (filter ceiling - not needed for actual competition)
	if (point_vector(2) > 2800.0) {
		return false;
	}

	// y-axis filter (filters walls - probably not needed for actual competition)
	if (point_vector(1) > 9500.0) {
		return false;
	}
	return true;
}

void point_cloud_rendering(pcl::PointCloud<pcl::PointXYZ>* cloud, std::vector<UINT16>& depthBuffer, ICoordinateMapper*& pCoordinateMapper, int depthWidth, int depthHeight, pcl::visualization::PCLVisualizer& viewer)
{
	if (calibration_edge_trigger) {
		viewer.removeShape("net_line");
		viewer.removeShape("pole_line2");
		viewer.removeShape("pole_line");
		viewer.addLine<pcl::PointXYZ, pcl::PointXYZ>(pcl::PointXYZ(-3500.0f, 6700.0f, 1550.0f), pcl::PointXYZ(3500.0f, 6700.0f, 1550.0f), "net_line");
		viewer.addLine<pcl::PointXYZ, pcl::PointXYZ>(pcl::PointXYZ(3500.0f, 6700.0f, 1550.0f), pcl::PointXYZ(3500.0f, 6700.0f, 0.0f), "pole_line");
		viewer.addLine<pcl::PointXYZ, pcl::PointXYZ>(pcl::PointXYZ(-3500.0f, 6700.0f, 1550.0f), pcl::PointXYZ(-3500.0f, 6700.0f, 0.0f), "pole_line2");
		calibration_edge_trigger = false;
	}
	
	for (int y = 0; y < depthHeight; y++){
		for (int x = 0; x < depthWidth; x++){
			pcl::PointXYZ point;

			DepthSpacePoint depthSpacePoint = { static_cast<float>(x), static_cast<float>(y) };
			UINT16 depth = depthBuffer[y * depthWidth + x];

			// Coordinate Mapping Depth to Camera Space, and Setting PointCloud XYZ
			CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };
			pCoordinateMapper->MapDepthPointToCameraSpace(depthSpacePoint, depth, &cameraSpacePoint);
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
		if (calibration_mode) {
			switch (event.getKeyCode()) {
				case 'j': net_line.first.x -= 0.02f; break;
				case 'J': net_line.first.x += 0.02f; break;
				case 'k': net_line.first.y -= 0.02f; break;
				case 'K': net_line.first.y += 0.02f; break;
				case 'l': net_line.first.z -= 0.02f; break;
				case 'L': net_line.first.z += 0.02f; break;
				case 'b': net_line.second.x -= 0.02f; pole_line.first.x -= 0.02f; break;
				case 'B': net_line.second.x += 0.02f; pole_line.first.x += 0.02f; break;
				case 'n': net_line.second.y -= 0.02f; pole_line.first.y -= 0.02f; break;
				case 'N': net_line.second.y += 0.02f; pole_line.first.y += 0.02f; break;
				case 'm': net_line.second.z -= 0.02f; pole_line.first.z -= 0.02f; break;
				case 'M': net_line.second.z += 0.02f; pole_line.first.z += 0.02f; break;
				case 'a': pole_line.second.x -= 0.02f; break;
				case 'A': pole_line.second.x += 0.02f; break;
				case 's': pole_line.second.y -= 0.02f; break;
				case 'S': pole_line.second.y += 0.02f; break;
				case 'd': pole_line.second.z -= 0.02f; break;
				case 'D': pole_line.second.z += 0.02f; break;
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

	std::conditional <
		std::chrono::high_resolution_clock::is_steady,
		std::chrono::high_resolution_clock,
		std::chrono::steady_clock > ::type clock;

	auto current_time = clock.now();

	unsigned char counter = 0;

	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-1.5f, 1.5f);

	viewer.registerKeyboardCallback(keyboard_callback);
	viewer.registerMouseCallback(mouse_callback);

	pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);


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

		// Create Point Cloud
		pointcloud->width = static_cast<uint32_t>(depthWidth);
		pointcloud->height = static_cast<uint32_t>(depthHeight);
		pointcloud->is_dense = true;

		if (calibration_mode) {
			point_cloud_calibration(pointcloud.get(), depthBuffer, pCoordinateMapper, depthWidth, depthHeight, viewer);
		}
		else {
			point_cloud_rendering(pointcloud.get(), depthBuffer, pCoordinateMapper, depthWidth, depthHeight, viewer);
		}

		// do post processing of cloud

		if (!viewer.updatePointCloud(pointcloud, "Cloud")) {
			// Show Point Cloud on Cloud Viewer
			viewer.addPointCloud(pointcloud, "Cloud");
			viewer.resetCameraViewpoint("Cloud");
		}
		// Input Key ( Exit ESC key )
		if( GetKeyState( VK_ESCAPE ) < 0 ){
			break;
		}
		++counter;
		if (counter == 60) {
			counter = 0;
			auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(clock.now() - current_time).count();
			std::basic_ostringstream<TCHAR> oss;
			oss << "Framerate: " << 60.0f * 1000.0f / duration << std::endl;
			OutputDebugString(oss.str().c_str());
			current_time = clock.now();
		}
		pointcloud->clear();
	}

	// End Processing
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