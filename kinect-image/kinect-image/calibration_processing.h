#pragma once

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <chrono>
#include <limits>
#include <cmath>
#include <tuple>

class calibration_processing
{
public:
	calibration_processing();
	~calibration_processing();
	static Eigen::Affine3d calculate_transformation(std::pair<pcl::PointXYZ, pcl::PointXYZ> net_line, std::pair<pcl::PointXYZ, pcl::PointXYZ> pole_line);
};

