#include "stdafx.h"
#include "calibration_processing.h"


calibration_processing::calibration_processing()
{
}


calibration_processing::~calibration_processing()
{
}

Eigen::Affine3d calibration_processing::calculate_transformation(std::pair<pcl::PointXYZ, pcl::PointXYZ> net_line, std::pair<pcl::PointXYZ, pcl::PointXYZ> pole_line)
{
	Eigen::Affine3d transformation;
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


	Eigen::Matrix3d kinect_basis;
	kinect_basis << net2, net_height, height2;

	Eigen::Matrix3d gyro_basis;
	gyro_basis << -3050.0, 3050.0, 3050.0,
		6700.0, 6700.0, 6700.0,
		1550.0, 1550.0, 0.0;

	return gyro_basis * kinect_basis.inverse() * transformation;
}