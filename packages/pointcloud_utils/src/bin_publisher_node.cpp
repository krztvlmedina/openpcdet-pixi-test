/**
 * @file bin_publisher_node.cpp
 * @brief ROS2 node for publishing .bin point cloud files as PointCloud2 messages
 *
 * This node reads .bin files from a directory and publishes them as sensor_msgs::PointCloud2
 * messages. It supports:
 * - Directory playback (sequential publishing of all .bin files)
 * - Configurable publish rate
 * - Timestamp synchronization based on file modification times
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <filesystem>
#include <vector>
#include <fstream>
#include <algorithm>
#include <chrono>

namespace fs = std::filesystem;

class BinPublisherNode : public rclcpp::Node
{
public:
  BinPublisherNode() : Node("bin_publisher_node"), current_file_index_(0)
  {
    // Declare parameters
    this->declare_parameter<std::string>("bin_directory", "");
    this->declare_parameter<double>("publish_rate", 10.0);
    this->declare_parameter<bool>("loop", true);
    this->declare_parameter<bool>("use_file_timestamps", false);
    this->declare_parameter<std::string>("frame_id", "velodyne");
    this->declare_parameter<std::string>("topic", "velodyne_points");
    this->declare_parameter<bool>("verbose", true);

    // Get parameters
    bin_directory_ = this->get_parameter("bin_directory").as_string();
    publish_rate_ = this->get_parameter("publish_rate").as_double();
    loop_ = this->get_parameter("loop").as_bool();
    use_file_timestamps_ = this->get_parameter("use_file_timestamps").as_bool();
    frame_id_ = this->get_parameter("frame_id").as_string();
    topic_ = this->get_parameter("topic").as_string();
    verbose_ = this->get_parameter("verbose").as_bool();

    // Validate bin_directory parameter
    if (bin_directory_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Parameter 'bin_directory' is required!");
      rclcpp::shutdown();
      return;
    }

    // Check if directory exists
    if (!fs::exists(bin_directory_) || !fs::is_directory(bin_directory_)) {
      RCLCPP_ERROR(this->get_logger(), "Directory does not exist: %s", bin_directory_.c_str());
      rclcpp::shutdown();
      return;
    }

    // Load all .bin files from directory
    loadBinFiles();

    if (bin_files_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No .bin files found in directory: %s", bin_directory_.c_str());
      rclcpp::shutdown();
      return;
    }

    // Create publisher
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_, 10);

    // Create timer based on publish rate
    auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
      std::bind(&BinPublisherNode::timerCallback, this)
    );

    RCLCPP_INFO(this->get_logger(), "Bin Publisher Node initialized");
    RCLCPP_INFO(this->get_logger(), "  Directory: %s", bin_directory_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Files found: %zu", bin_files_.size());
    RCLCPP_INFO(this->get_logger(), "  Publish rate: %.2f Hz", publish_rate_);
    RCLCPP_INFO(this->get_logger(), "  Loop: %s", loop_ ? "yes" : "no");
    RCLCPP_INFO(this->get_logger(), "  Use file timestamps: %s", use_file_timestamps_ ? "yes" : "no");
    RCLCPP_INFO(this->get_logger(), "  Frame ID: %s", frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Topic: %s", topic_.c_str());
  }

private:
  void loadBinFiles()
  {
    for (const auto& entry : fs::directory_iterator(bin_directory_)) {
      if (entry.is_regular_file() && entry.path().extension() == ".bin") {
        bin_files_.push_back(entry.path());
      }
    }

    // Sort files by name for sequential playback
    std::sort(bin_files_.begin(), bin_files_.end());

    if (verbose_) {
      RCLCPP_INFO(this->get_logger(), "Loaded %zu .bin files:", bin_files_.size());
      for (size_t i = 0; i < std::min(bin_files_.size(), size_t(5)); ++i) {
        RCLCPP_INFO(this->get_logger(), "  [%zu] %s", i, bin_files_[i].filename().string().c_str());
      }
      if (bin_files_.size() > 5) {
        RCLCPP_INFO(this->get_logger(), "  ... and %zu more files", bin_files_.size() - 5);
      }
    }
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr readBinFile(const fs::path& filepath)
  {
    std::ifstream input(filepath, std::ios::binary);
    if (!input) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", filepath.string().c_str());
      return nullptr;
    }

    // Get file size
    input.seekg(0, std::ios::end);
    size_t file_size = input.tellg();
    input.seekg(0, std::ios::beg);

    // Each point is 4 floats (x, y, z, intensity) = 16 bytes
    size_t num_points = file_size / 16;

    if (file_size % 16 != 0) {
      RCLCPP_WARN(this->get_logger(), "File size is not a multiple of 16 bytes: %s",
                  filepath.string().c_str());
      num_points = file_size / 16;  // Truncate incomplete points
    }

    // Read all data at once
    std::vector<float> buffer(num_points * 4);
    input.read(reinterpret_cast<char*>(buffer.data()), num_points * 16);
    input.close();

    // Create PCL point cloud
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    cloud->points.reserve(num_points);

    for (size_t i = 0; i < num_points; ++i) {
      pcl::PointXYZI point;
      point.x = buffer[i * 4 + 0];
      point.y = buffer[i * 4 + 1];
      point.z = buffer[i * 4 + 2];
      point.intensity = buffer[i * 4 + 3];
      cloud->points.push_back(point);
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    return cloud;
  }

  void timerCallback()
  {
    // Check if we have files to publish
    if (current_file_index_ >= bin_files_.size()) {
      if (loop_) {
        current_file_index_ = 0;  // Restart from beginning
        if (verbose_) {
          RCLCPP_INFO(this->get_logger(), "Looping back to first file");
        }
      } else {
        RCLCPP_INFO(this->get_logger(), "Finished publishing all files. Shutting down.");
        rclcpp::shutdown();
        return;
      }
    }

    // Get current file path
    const auto& filepath = bin_files_[current_file_index_];

    // Read point cloud from .bin file
    auto cloud = readBinFile(filepath);
    if (!cloud || cloud->points.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to read or empty cloud: %s",
                   filepath.filename().string().c_str());
      current_file_index_++;
      return;
    }

    // Convert to ROS message
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.frame_id = frame_id_;

    // Set timestamp
    if (use_file_timestamps_) {
      // Use file modification time
      auto ftime = fs::last_write_time(filepath);
      auto sctp = std::chrono::time_point_cast<std::chrono::system_clock::duration>(
        ftime - fs::file_time_type::clock::now() + std::chrono::system_clock::now()
      );
      auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        sctp.time_since_epoch()
      ).count();
      msg.header.stamp.sec = ns / 1000000000;
      msg.header.stamp.nanosec = ns % 1000000000;
    } else {
      // Use current time
      msg.header.stamp = this->now();
    }

    // Publish
    publisher_->publish(msg);

    if (verbose_) {
      RCLCPP_INFO(this->get_logger(),
                  "Published [%zu/%zu]: %s (%zu points)",
                  current_file_index_ + 1,
                  bin_files_.size(),
                  filepath.filename().string().c_str(),
                  cloud->points.size());
    }

    // Move to next file
    current_file_index_++;
  }

  // Node parameters
  std::string bin_directory_;
  double publish_rate_;
  bool loop_;
  bool use_file_timestamps_;
  std::string frame_id_;
  std::string topic_;
  bool verbose_;

  // Internal state
  std::vector<fs::path> bin_files_;
  size_t current_file_index_;

  // ROS2 objects
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BinPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
