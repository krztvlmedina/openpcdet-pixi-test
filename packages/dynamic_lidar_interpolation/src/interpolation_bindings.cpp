/**
 * Python bindings for point cloud interpolation algorithms
 *
 * This module provides Python bindings to the C++ interpolation algorithms
 * without requiring ROS2 dependencies.
 *
 * @author Dynamic LiDAR Interpolation Contributors
 * @license AGPL-3.0
 */

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

#include "dynamic_lidar_interpolation/pointcloud_interpolation.hpp"

namespace py = pybind11;

/**
 * Convert numpy array to PCL point cloud
 * Expected input shape: (N, 3) or (N, 4) where columns are [x, y, z] or [x, y, z, intensity]
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr numpy_to_pcl(
    py::array_t<float> points_array)
{
    auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

    auto buf = points_array.request();

    if (buf.ndim != 2) {
        throw std::runtime_error("Input array must be 2-dimensional (N, 3) or (N, 4)");
    }

    size_t num_points = buf.shape[0];
    size_t num_dims = buf.shape[1];

    if (num_dims < 3) {
        throw std::runtime_error("Input array must have at least 3 columns (x, y, z)");
    }

    cloud->points.resize(num_points);
    cloud->width = num_points;
    cloud->height = 1;
    cloud->is_dense = false;

    float* ptr = static_cast<float*>(buf.ptr);

    for (size_t i = 0; i < num_points; ++i) {
        cloud->points[i].x = ptr[i * num_dims + 0];
        cloud->points[i].y = ptr[i * num_dims + 1];
        cloud->points[i].z = ptr[i * num_dims + 2];
    }

    return cloud;
}

/**
 * Convert PCL point cloud to numpy array
 * Output shape: (N, 3) where columns are [x, y, z]
 */
py::array_t<float> pcl_to_numpy(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    size_t num_points = cloud->points.size();

    // Create numpy array with shape (N, 3)
    auto result = py::array_t<float>({num_points, size_t(3)});
    auto buf = result.request();
    float* ptr = static_cast<float*>(buf.ptr);

    for (size_t i = 0; i < num_points; ++i) {
        ptr[i * 3 + 0] = cloud->points[i].x;
        ptr[i * 3 + 1] = cloud->points[i].y;
        ptr[i * 3 + 2] = cloud->points[i].z;
    }

    return result;
}

/**
 * Python-friendly wrapper for the interpolation function
 */
py::array_t<float> interpolate_pointcloud(
    py::array_t<float> input_points,
    double angular_res_x,
    double angular_res_y,
    double max_angle_width,
    double max_angle_height,
    const std::string& interpolation_method,
    double scale_factor_x,
    double scale_factor_y,
    double min_range,
    double max_range,
    bool apply_variance_filter,
    double max_allowed_variance,
    const std::vector<float>& sensor_translation,
    double rotation_angle_x,
    double extrapolation_value,
    double min_ang_fov,
    double max_ang_fov)
{
    // Convert numpy to PCL
    auto input_cloud = numpy_to_pcl(input_points);

    // Create interpolator
    pointcloud_interpolation::PointCloudInterpolator interpolator;

    // Convert sensor translation to Eigen vector
    Eigen::Vector3f sensor_trans(
        sensor_translation.size() >= 1 ? sensor_translation[0] : 0.0f,
        sensor_translation.size() >= 2 ? sensor_translation[1] : 0.0f,
        sensor_translation.size() >= 3 ? sensor_translation[2] : 0.0f
    );

    // Convert rotation angle from degrees to radians
    double rotation_rad = rotation_angle_x * M_PI / 180.0;

    // Perform interpolation
    auto output_cloud = interpolator.interpolatePointCloud(
        input_cloud,
        angular_res_x,
        angular_res_y,
        max_angle_width,
        max_angle_height,
        interpolation_method,
        scale_factor_x,
        scale_factor_y,
        min_range,
        max_range,
        apply_variance_filter,
        max_allowed_variance,
        sensor_trans,
        rotation_rad,
        extrapolation_value,
        min_ang_fov,
        max_ang_fov
    );

    // Convert PCL to numpy
    return pcl_to_numpy(output_cloud);
}

PYBIND11_MODULE(interpolation_core, m) {
    m.doc() = "Point cloud interpolation algorithms for LiDAR data";

    m.def("interpolate",
          &interpolate_pointcloud,
          py::arg("input_points"),
          py::arg("angular_res_x") = 0.25,
          py::arg("angular_res_y") = 2.05,
          py::arg("max_angle_width") = 360.0,
          py::arg("max_angle_height") = 180.0,
          py::arg("interpolation_method") = "linear",
          py::arg("scale_factor_x") = 1.0,
          py::arg("scale_factor_y") = 2.0,
          py::arg("min_range") = 0.0,
          py::arg("max_range") = std::numeric_limits<double>::max(),
          py::arg("apply_variance_filter") = false,
          py::arg("max_allowed_variance") = 50.0,
          py::arg("sensor_translation") = std::vector<float>{0.0f, 0.0f, 0.0f},
          py::arg("rotation_angle_x") = 0.0,
          py::arg("extrapolation_value") = std::numeric_limits<double>::quiet_NaN(),
          py::arg("min_ang_fov") = 0.0,
          py::arg("max_ang_fov") = 360.0,
          R"pbdoc(
            Interpolate a point cloud using various interpolation methods.

            Parameters
            ----------
            input_points : numpy.ndarray
                Input point cloud as (N, 3) or (N, 4) array [x, y, z] or [x, y, z, intensity]
            angular_res_x : float
                Angular resolution in X direction (azimuth) in degrees (default: 0.25)
            angular_res_y : float
                Angular resolution in Y direction (elevation) in degrees (default: 2.05)
            max_angle_width : float
                Maximum angle width in degrees (default: 360.0)
            max_angle_height : float
                Maximum angle height in degrees (default: 180.0)
            interpolation_method : str
                Interpolation method: 'linear', 'nearest', 'bilateral', 'edgeAware', or 'spline'
                (default: 'linear')
            scale_factor_x : float
                Scale factor in X direction (default: 1.0)
            scale_factor_y : float
                Scale factor in Y direction (default: 2.0)
            min_range : float
                Minimum range filter in meters (default: 0.0)
            max_range : float
                Maximum range filter in meters (default: inf)
            apply_variance_filter : bool
                Apply variance filtering (default: False)
            max_allowed_variance : float
                Maximum allowed variance (default: 50.0)
            sensor_translation : list of float
                3D translation offset [x, y, z] (default: [0.0, 0.0, 0.0])
            rotation_angle_x : float
                Rotation around X axis in degrees (default: 0.0)
            extrapolation_value : float
                Value for extrapolation (default: NaN)
            min_ang_fov : float
                Minimum angle FOV in degrees (default: 0.0)
            max_ang_fov : float
                Maximum angle FOV in degrees (default: 360.0)

            Returns
            -------
            numpy.ndarray
                Interpolated point cloud as (M, 3) array [x, y, z]

            Examples
            --------
            >>> import numpy as np
            >>> import interpolation_core
            >>> points = np.random.rand(1000, 3).astype(np.float32)
            >>> interpolated = interpolation_core.interpolate(
            ...     points,
            ...     interpolation_method='linear',
            ...     scale_factor_y=2.0
            ... )
          )pbdoc");
}
