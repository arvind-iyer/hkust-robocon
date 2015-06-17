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
#include <list>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_multifit_nlin.h>
#include <unsupported/Eigen/NonLinearOptimization>

static int SHUTTLE_POINTS_MAX = 100;
static int SHUTTLE_POINTS_MIN = 10;
const static int SHUTTLE_TOLERANCE = 50;

class point_cloud_tracking
{
private:
	static pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	static int prev_cloud_number;
	static int prev_centroid_number;
	static std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;
	const static std::vector<std::tuple<unsigned char, unsigned char, unsigned char>> color_vector;
	static bool fast_image_processing_enabled;

	struct data {
		size_t n;
		double* x_pos;
		double* y_pos;
		double* sigma;
	};

	static inline void generate_clusters(pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud)
	{
		clouds.clear();

		// Creating the KdTree object for the search method of the extraction
		if (point_cloud->empty()) {
			return;
		}

		ec.getSearchMethod()->setInputCloud(point_cloud);

		std::vector<pcl::PointIndices> cluster_indices;
		ec.setInputCloud(point_cloud);
		ec.extract(cluster_indices);

		int j = 0;
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
			for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
				cloud_cluster->points.push_back(point_cloud->points[*pit]);
			cloud_cluster->width = static_cast<uint32_t>(cloud_cluster->points.size());
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;
			clouds.push_back(cloud_cluster);
			++j;
		}
	}

	static int shuttlecock_trajectory_equation_f(const gsl_vector* estimated_constants, void* position_data, gsl_vector* f)
	{
		size_t n = static_cast<struct data*>(position_data)->n;
		double* x_pos = static_cast<struct data*>(position_data)->x_pos;
		double* y_pos = static_cast<struct data*>(position_data)->y_pos;
		double* sigma = static_cast<struct data*>(position_data)->sigma;

		double v_xi = gsl_vector_get(estimated_constants, 0);
		double v_yi = gsl_vector_get(estimated_constants, 1);
		double v_tprime = gsl_vector_get(estimated_constants, 2);

		const static double grav_acc = 9.81;

		for (int i = 0; i < n; ++i)
		{
			// model equation
			double x = x_pos[i];
			double y = std::pow(v_tprime, 2.0) * 
				std::log((std::sin(v_tprime * (std::exp(grav_acc * x / std::pow(v_tprime, 2.0) - 1.0)) / v_xi) + 
				std::atan(v_tprime/v_yi)) / 
				std::sin(std::atan(v_tprime/v_yi)))/ grav_acc;
			gsl_vector_set(f, i, (y - y_pos[i])/sigma[i]);
		}

		return GSL_SUCCESS;
	}

	static int shuttlecock_trajectory_equation_df(const gsl_vector* estimated_constants, void* position_data, gsl_matrix* J)
	{
		size_t n = static_cast<struct data*>(position_data)->n;
		double* x_pos = static_cast<struct data*>(position_data)->x_pos;
		double* y_pos = static_cast<struct data*>(position_data)->y_pos;
		double* sigma = static_cast<struct data*>(position_data)->sigma;

		double v_xi = gsl_vector_get(estimated_constants, 0);
		double v_yi = gsl_vector_get(estimated_constants, 1);
		double v_tprime = gsl_vector_get(estimated_constants, 2);

		const static double grav_acc = 9.81;

		double vt2vy2p1 = (v_tprime * v_tprime) / (v_yi * v_yi) + 1.0;
		double sqrtvt2vy2p1 = std::sqrt((v_tprime * v_tprime) / (v_yi * v_yi) + 1.0);

		for (int i = 0; i < n; ++i)
		{
			double natpowm1 = std::exp(grav_acc * x_pos[i] / (v_tprime * v_tprime));
			double natpow = std::exp(grav_acc * x_pos[i] / (v_tprime * v_tprime)) - 1.0;
			double sintan = std::sin(v_tprime * natpow / v_xi) + std::atan(v_tprime / v_yi);

			// partial derivative with respect to v_xi
			double dx = -std::pow(v_tprime, 3.0) * natpow * std::cos(v_tprime * natpow / v_xi) /
				(grav_acc * sigma[i] * v_xi * v_xi * sintan);

			// partial derivative with respect to v_yi
			double dy = std::pow(v_tprime, 3.0) * (sqrtvt2vy2p1 * sintan / v_tprime 
				- v_tprime * sintan / (v_yi * v_yi * sqrtvt2vy2p1) 
				- 1 / (v_yi * sqrtvt2vy2p1)) / 
				(grav_acc * sigma[i] * v_yi * sqrtvt2vy2p1 * sintan);

			// partial derivative with respect to v_tprime
			double dt = (2 * v_tprime * std::log(v_yi * sqrtvt2vy2p1 * sintan / v_tprime) / grav_acc +
				(std::pow(v_tprime, 3.0) * 
					((v_yi * sqrtvt2vy2p1 *
						(natpow / v_xi - 2 * grav_acc * x_pos[i] * natpowm1 / (v_tprime * v_tprime * v_xi))) *
						std::cos(v_tprime * natpow / v_xi) + 1 / (v_yi * vt2vy2p1)) / v_tprime - 
					v_yi * sqrtvt2vy2p1 * sintan / (v_tprime * v_tprime) +
					sintan / (v_yi * sqrtvt2vy2p1)) / 
				(grav_acc * v_yi * sqrtvt2vy2p1 * sintan)) / sigma[i];
			gsl_matrix_set(J, i, 0, dx);
			gsl_matrix_set(J, i, 1, dy);
			gsl_matrix_set(J, i, 2, dt);
		}
		return GSL_SUCCESS;
	}

	static int shuttlecock_trajectory_equation_fdf(const gsl_vector* estimated_constants, void* position_data, gsl_vector* f, gsl_matrix* J)
	{
		size_t n = static_cast<struct data*>(position_data)->n;
		double* x_pos = static_cast<struct data*>(position_data)->x_pos;
		double* y_pos = static_cast<struct data*>(position_data)->y_pos;
		double* sigma = static_cast<struct data*>(position_data)->sigma;

		double v_xi = gsl_vector_get(estimated_constants, 0);
		double v_yi = gsl_vector_get(estimated_constants, 1);
		double v_tprime = gsl_vector_get(estimated_constants, 2);

		const static double grav_acc = 9.81;

		double vt2vy2p1 = (v_tprime * v_tprime) / (v_yi * v_yi) + 1.0;
		double sqrtvt2vy2p1 = std::sqrt((v_tprime * v_tprime) / (v_yi * v_yi) + 1.0);

		for (int i = 0; i < n; ++i)
		{
			// precomputed for optimization
			double natpowm1 = std::exp(grav_acc * x_pos[i] / (v_tprime * v_tprime));
			double natpow = std::exp(grav_acc * x_pos[i] / (v_tprime * v_tprime)) - 1.0;
			double sintan = std::sin(v_tprime * natpow / v_xi) + std::atan(v_tprime / v_yi);

			// model equation
			double y = v_tprime * v_tprime *
				std::log(sintan / std::sin(std::atan(v_tprime / v_yi))) / grav_acc;
			gsl_vector_set(f, i, (y - y_pos[i]) / sigma[i]);

			// partial derivative with respect to v_xi
			double dx = -std::pow(v_tprime, 3.0) * natpow * std::cos(v_tprime * natpow / v_xi) /
				(grav_acc * sigma[i] * v_xi * v_xi * sintan);

			// partial derivative with respect to v_yi
			double dy = std::pow(v_tprime, 3.0) * (sqrtvt2vy2p1 * sintan / v_tprime
				- v_tprime * sintan / (v_yi * v_yi * sqrtvt2vy2p1)
				- 1 / (v_yi * sqrtvt2vy2p1)) /
				(grav_acc * sigma[i] * v_yi * sqrtvt2vy2p1 * sintan);

			// partial derivative with respect to v_tprime
			double dt = (2 * v_tprime * std::log(v_yi * sqrtvt2vy2p1 * sintan / v_tprime) / grav_acc +
				(std::pow(v_tprime, 3.0) *
					((v_yi * sqrtvt2vy2p1 *
						(natpow / v_xi - 2 * grav_acc * x_pos[i] * natpowm1 / (v_tprime * v_tprime * v_xi))) *
						std::cos(v_tprime * natpow / v_xi) + 1 / (v_yi * vt2vy2p1)) / v_tprime -
					v_yi * sqrtvt2vy2p1 * sintan / (v_tprime * v_tprime) +
					sintan / (v_yi * sqrtvt2vy2p1)) /
				(grav_acc * v_yi * sqrtvt2vy2p1 * sintan)) / sigma[i];

			gsl_matrix_set(J, i, 0, dx);
			gsl_matrix_set(J, i, 1, dy);
			gsl_matrix_set(J, i, 2, dt);
		}
		return GSL_SUCCESS;
	}

	static std::list<std::pair<Eigen::Vector4d, pcl::PointCloud<pcl::PointXYZ>::Ptr>> cluster_cloud_centroids;

public:
	point_cloud_tracking() {};
	~point_cloud_tracking() {};

	static inline void increase_max() {
		std::cout << SHUTTLE_POINTS_MAX << std::endl;
		++SHUTTLE_POINTS_MAX;
	}

	static inline void decrease_max() {
		std::cout << SHUTTLE_POINTS_MAX << std::endl;
		--SHUTTLE_POINTS_MAX;
	}

	static inline void increase_min() {
		std::cout << SHUTTLE_POINTS_MIN << std::endl;
		++SHUTTLE_POINTS_MIN;
	}

	static inline void decrease_min() {
		std::cout << SHUTTLE_POINTS_MIN << std::endl;
		--SHUTTLE_POINTS_MIN;
	}

	static inline bool icp_depth_image_processing(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<UINT16>& depthBuffer, ICoordinateMapper*& pCoordinateMapper, int depthWidth, int depthHeight, pcl::visualization::PCLVisualizer& viewer, Eigen::Affine3d& transformation)
	{
		if (!fast_image_processing_enabled)
			return false;

		for (auto i = prev_cloud_number; i >= 0; --i) {
			viewer.removePointCloud("Cloud" + std::to_string(i));
			viewer.removeShape("cube" + std::to_string(i));
		}
		prev_cloud_number = -1;

		for (int y = 0; y < depthHeight; y++){
			for (int x = 0; x < depthWidth; x++){
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

				if (![&point_vector, &cameraSpacePoint, &transformation]() {
					point_vector << cameraSpacePoint.X, cameraSpacePoint.Y, cameraSpacePoint.Z;
					point_vector = transformation * point_vector;

					// height filter
					// lower bound (filters anything below the net)
					// if (point_vector(2) < 1570.0) { return false; }

					// floor filter
					if (point_vector(2) < 10.0) {
						return false;
					}

					// net filter
					if (point_vector(1) < 6710.0 && point_vector(1) > 6690.0 && point_vector(2) < 1570.0)
					{
						return false;
					}

					// upper bound (filter ceiling - not needed for actual competition)
					//if (point_vector(2) > 2700.0) { return false; }

					// y-axis filter (filters walls - probably not needed for actual competition)
					//if (point_vector(1) > 9500.0) { return false; }

					// bound filtering
					if (point_vector(0) > 3050.0 || point_vector(0) < -3050.0) { return false; }
					if (point_vector(1) > 13400.0 || point_vector(1) < 0.0) { return false; }

					return true; 
					}())
				{
					continue;
				}

				// add points to centroids within a cubic bounding box
				for (auto& i : cluster_cloud_centroids)
				{
					if (std::abs(point_vector.x() - i.first.x()) < 400.0 && 
						std::abs(point_vector.z() - i.first.z()) < 800.0 && 
						(point_vector.y() - i.first.y() < 200.0 && 
						point_vector.y() - i.first.y() > -1700.0)) {
						i.second->push_back(pcl::PointXYZ(point_vector.x(), point_vector.y(), point_vector.z()));
					}
				}
				//cloud->push_back(pcl::PointXYZ(point_vector.x(), point_vector.y(), point_vector.z()));
			}
		}

		for (auto i = cluster_cloud_centroids.begin(); i != cluster_cloud_centroids.end();)
		{
			if (i->second->points.size() < 4) {
				i = cluster_cloud_centroids.erase(i);
				continue;
			}
			i->second->height = 1;
			i->second->width = static_cast<uint32_t>(i->second->points.size());
			i->second->is_dense = true;
			Eigen::Vector4d center = Eigen::Vector4d::Identity();
			pcl::compute3DCentroid(*i->second, center);
			i->second->clear();

			// check if y-displacement is positive, and remove tracking cluster if it is

			const double epsilon = 0.01;

			if (center.y() - i->first.y() > 0.0 || (std::abs(center.x() - 0.0) < epsilon && std::abs(center.y() - 0.0) < epsilon && std::abs(center.z() - 0.0) < epsilon && std::abs(center.w() - 0.0) < epsilon)) {
				i = cluster_cloud_centroids.erase(i);
				continue;
			}

			// OutputDebugString((_T("Center y difference: ") + std::to_wstring(center.y() - i->first.y()) + _T("\n")).c_str());

			// if it is not positive, push it into the equation fitter and update the center
			i->first = center;
			++i;
		}

		for (int i = prev_centroid_number; i > cluster_cloud_centroids.size(); --i)
		{
			viewer.removeShape("sphere" + std::to_string(i));
		}

		if (cluster_cloud_centroids.size() > 0) {
			int j = 0;
			for (auto i = cluster_cloud_centroids.begin(); i != cluster_cloud_centroids.end(); ++i) {
				if (!viewer.updateSphere(pcl::PointXYZ(i->first.x(), i->first.y(), i->first.z()), 100, 0.5, 0.5, 0.5, "sphere" + std::to_string(j + 1)))
				{
					viewer.addSphere(pcl::PointXYZ(i->first.x(), i->first.y(), i->first.z()), 100, 0.5, 0.5, 0.5, "sphere" + std::to_string(j + 1));
				}
				++j;
			}
		}

		prev_centroid_number = static_cast<int>(cluster_cloud_centroids.size());

		if (cluster_cloud_centroids.empty()) {
			// tracking has failed
			// OutputDebugString(_T("Tracking has failed\n"));
			fast_image_processing_enabled = false;
		}

		return true;
	}

	static inline void track_clusters(pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud)
	{
		generate_clusters(point_cloud);
		for (auto &i : clouds) {
			Eigen::Vector4d center;
			pcl::compute3DCentroid(*i, center);
			cluster_cloud_centroids.push_back(std::make_pair(center, pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>())));
		}
		fast_image_processing_enabled = true;
	}

	static inline void display_clusters(pcl::visualization::PCLVisualizer& viewer)
	{
		int j = 0;
		for (auto &i : clouds) {
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(i, std::get<0>(color_vector[j % 3]), std::get<1>(color_vector[j % 3]), std::get<2>(color_vector[j % 3]));
			
			viewer.removeShape("cube" + std::to_string(j));
			viewer.addCube(cluster_cloud_centroids.back().first.x() - 400.0, 
				cluster_cloud_centroids.back().first.x() + 400.0,
				cluster_cloud_centroids.back().first.y() - 1700.0, 
				cluster_cloud_centroids.back().first.y() + 200.0,
				cluster_cloud_centroids.back().first.z() - 600.0,
				cluster_cloud_centroids.back().first.z() + 600.0,
				std::get<0>(color_vector[j % 3]),
				std::get<1>(color_vector[j % 3]),
				std::get<2>(color_vector[j % 3]),
				"cube" + std::to_string(j));
				
			/*
			if (!viewer.updateSphere(pcl::PointXYZ(cluster_cloud_centroids.back().first.x(),
				cluster_cloud_centroids.back().first.y(),
				cluster_cloud_centroids.back().first.z()), 60, 
				std::get<0>(color_vector[j % 3]),
				std::get<1>(color_vector[j % 3]),
				std::get<2>(color_vector[j % 3]),
				"sphere" + std::to_string(j))) {
				viewer.addSphere(pcl::PointXYZ(cluster_cloud_centroids.back().first.x(),
					cluster_cloud_centroids.back().first.y(),
					cluster_cloud_centroids.back().first.z()), 60,
					std::get<0>(color_vector[j % 3]),
					std::get<1>(color_vector[j % 3]),
					std::get<2>(color_vector[j % 3]),
					"sphere" + std::to_string(j));
			}
			*/
			
			if (!viewer.updatePointCloud(i, single_color, "Cloud" + std::to_string(j))) {
				// Show Point Cloud on Cloud Viewer
				viewer.addPointCloud(i, single_color, "Cloud" + std::to_string(j));
			}
			 viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Cloud" + std::to_string(j));
			++j;
		}

		if (prev_cloud_number > j) {
			for (auto i = prev_cloud_number; i > j; --i) {
				viewer.removeShape("cube" + std::to_string(i));
				viewer.removePointCloud("Cloud" + std::to_string(i));
			}
		}
		prev_cloud_number = j;
	}
};

bool point_cloud_tracking::fast_image_processing_enabled = false;
int point_cloud_tracking::prev_cloud_number = -1;
int point_cloud_tracking::prev_centroid_number = 0;

const std::vector<std::tuple<unsigned char, unsigned char, unsigned char>> point_cloud_tracking::color_vector = {
	std::make_tuple(0, 255, 0), std::make_tuple(255, 0, 0), std::make_tuple(0, 0, 255)
};

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> point_cloud_tracking::clouds = std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>();
std::list<std::pair<Eigen::Vector4d, pcl::PointCloud<pcl::PointXYZ>::Ptr>> point_cloud_tracking::cluster_cloud_centroids = std::list<std::pair<Eigen::Vector4d, pcl::PointCloud<pcl::PointXYZ>::Ptr>>();
pcl::EuclideanClusterExtraction<pcl::PointXYZ> point_cloud_tracking::ec = [] {
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(SHUTTLE_TOLERANCE); // 5 cm
	ec.setMinClusterSize(SHUTTLE_POINTS_MIN);
	ec.setMaxClusterSize(SHUTTLE_POINTS_MAX);
	ec.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>()));
	return ec;
}();