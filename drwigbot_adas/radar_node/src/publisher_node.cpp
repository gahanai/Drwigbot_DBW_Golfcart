#include <chrono>
#include <memory>
#include <string>
#include <algorithm>
#include <cmath>
#include <fstream> // Include for file handling
#include <filesystem> // Include for folder creation

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <radar_node/drwig_parser.hpp>

#include "radar_node/msg/array_tracked_obj.hpp"
#include "radar_node/msg/array_detected_obj.hpp"
#include "radar_node/msg/array_cluster_obj.hpp"
#include "radar_node/msg/tracked_obj.hpp"
#include "radar_node/msg/cluster_obj.hpp"
#include "radar_node/msg/detected_obj.hpp"

using namespace std::chrono_literals;

class RadarPubNode : public rclcpp::Node
{
public:

  std::chrono::time_point<std::chrono::system_clock> last_detection_time;

  RadarPubNode(std::string name, std::string port, const uint32_t buad_rate)
      : Node(name), count_(0), parser(port, buad_rate)
  {

    // Create the directory for radar logs
    std::string folder_name = "radar_data_log";
    std::filesystem::create_directory(folder_name);  // Create directory if it doesn't exist

    // Get current time
    std::time_t current_time = std::time(nullptr);
    
    std::stringstream ss;
    ss << folder_name << "/radar_log_" << std::put_time(std::localtime(&current_time), "%Y%m%d_%H%M%S") << ".txt";
    log_file_.open(ss.str(), std::ios::out); // Create a new file for each run

    tracked_publisher_ = this->create_publisher<radar_node::msg::ArrayTrackedObj>(name + "tracked_obj", 10);
    cluster_publisher_ = this->create_publisher<radar_node::msg::ArrayClusterObj>(name + "cluster_obj", 10);
    detected_publisher_ = this->create_publisher<radar_node::msg::ArrayDetectedObj>(name + "detected_obj", 10);
    combined_tracked_publisher_ = this->create_publisher<radar_node::msg::ArrayTrackedObj>("tracked_obj", 10);

    // Log Successful shutdown
    RCLCPP_INFO(this->get_logger(), "Radar node Initialized Successfully");

    this->Init();
    this->last_detection_time = std::chrono::system_clock::now();

    auto timer_callback = [this]() -> void
    {
      parser.ParseFrame();
      RCLCPP_INFO(this->get_logger(), "Publishing: ");
    };
    timer_ = this->create_wall_timer(50ms, timer_callback);
  }

  ~RadarPubNode()
  {
    parser.Wait();
    log_file_.close(); // Close the log file
  }

  void logData(const std::string &type, const std::string &data)
  {
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    log_file_ << std::ctime(&time) << " - " << type << ": " << data << std::endl;
  }

  void Init()
  {
    parser.Init();
    using std::placeholders::_1;
    using std::placeholders::_2;

    this->drwig_cluster_object = std::bind(&RadarPubNode::handleClusterObjData, this, _1, _2);
    this->drwig_tracked_object = std::bind(&RadarPubNode::handleTrackedData, this, _1, _2);
    this->drwig_detected_object = std::bind(&RadarPubNode::handleDetectedData, this, _1, _2);
    parser.RegisterObjectHandler(this->drwig_cluster_object);
    parser.RegisterObjectHandler(this->drwig_tracked_object);
    parser.RegisterObjectHandler(this->drwig_detected_object);
    parser.ReadFrame();
  }

  void handleClusterObjData(const DrwigObjectHeader &header, const std::vector<DrwigClusterObject> &cluster_object)
  {
    RCLCPP_INFO(this->get_logger(), "Cluster object data received.");
    auto pub_msg = std::make_shared<radar_node::msg::ArrayClusterObj>();

    std::cout << "\nTotal Number of Cluster Objects: " << header.num_obj;
    double result = pow(2, header.format);

    std::ostringstream cluster_log;
    cluster_log << "Total Cluster Objects: " << header.num_obj;
    logData("Clustered Objects", cluster_log.str());

    // Vector to store clustered objects with their distances
    std::vector<std::pair<float, DrwigClusterObject>> objects_with_distance;

    // Calculate distances and store them
    for (const auto &cluster : cluster_object)
    {
      float x_center = static_cast<float>(cluster.x) / result;
      float y_center = static_cast<float>(cluster.y) / result;
      float distance = sqrt(x_center * x_center + y_center * y_center);
      objects_with_distance.emplace_back(distance, cluster);
    }

    // Sort objects by distance (ascending order)
    std::sort(objects_with_distance.begin(), objects_with_distance.end(),
              [](const auto &a, const auto &b)
              {
                return a.first < b.first; // Compare distances
              });

    bool brake_status_flag = false;

    for (const auto &pair : objects_with_distance)
    {
      const auto &cluster = pair.second; // Get the clustered object

      radar_node::msg::ClusterObj pub_obj;
      float x_center = static_cast<float>(cluster.x) / result;
      float y_center = static_cast<float>(cluster.y) / result;
      float cluster_width = static_cast<float>(cluster.width) / result;
      float cluster_length = static_cast<float>(cluster.length) / result;

      std::cout << "\n - x_center: " << x_center << ", y_center: " << y_center
                << " - cluster_width: " << cluster_width << ", cluster_length: " << cluster_length;

      std::ostringstream obj_log;
      obj_log << "x_center: " << (cluster.x / result) << ", y_center: " << (cluster.y / result)
              << ", width: " << (cluster.width / result) << ", length: " << (cluster.length / result);
      logData("Clustered Object", obj_log.str());

      pub_obj.x_center = x_center;
      pub_obj.y_center = y_center;
      pub_obj.cluster_width = cluster_width;
      pub_obj.cluster_length = cluster_length;

      // Check brake status using the unified function
      pub_obj.brake_status = shouldBrake(cluster, header, cluster_width, cluster_length, x_center, y_center);

      if (pub_obj.brake_status)
      {
        brake_status_flag = true;
      }

      pub_msg->cluster_obj.push_back(pub_obj);
    }

    // Set overall brake status based on clusters
    pub_msg->brake_status = brake_status_flag;

    logData("Brake Status", brake_status_flag ? "True" : "False");

    cluster_publisher_->publish(*pub_msg);
    publishCombinedBrake();
  }

  void handleDetectedData(const DrwigObjectHeader &header, const std::vector<DrwigDetectedObject> &detected_object)
  {
    RCLCPP_INFO(this->get_logger(), "Detected object data received.");
    auto pub_msg = std::make_shared<radar_node::msg::ArrayDetectedObj>();

    std::cout << "\nTotal Number of Detected Objects: " << header.num_obj;
    double result = pow(2, header.format);

    std::ostringstream detected_log;
    detected_log << "Total Detected Objects: " << header.num_obj;
    logData("Detected Objects", detected_log.str());

    static const std::chrono::milliseconds stable_duration(500); // Duration to maintain stable state
    static bool brake_status_flag = false;                       // Initialize brake status

    // Current time
    auto now = std::chrono::system_clock::now();
    bool new_detection = false; // Track if any new detections occur

    // Check if new objects are detected
    if (!detected_object.empty())
    {
      // Check if any detected object requires braking
      for (const auto &obj : detected_object)
      {
        // Check if the object is within ROI
        if (shouldBrake(obj, header, 0, 0, static_cast<float>(obj.x) / result, static_cast<float>(obj.y) / result))
        {
          this->last_detection_time = now; // Reset timer on valid detection
          brake_status_flag = true;  // Set brake status to true
          new_detection = true;            // Mark that a new object was detected
          RCLCPP_INFO(this->get_logger(), "Brake status set to true due to detected object within ROI.");
          break; // Exit the loop once a valid object is found
        }
      }
    }

    // If no new detections were made, check the elapsed time
    if (!new_detection)
    {
      // Check if 500 ms have passed since the last detection
      auto elapsed_time = now - this->last_detection_time;
      if (elapsed_time >= stable_duration)
      {
        brake_status_flag = false; // Set brake status to false if no new detections for 500 ms
        RCLCPP_INFO(this->get_logger(), "Brake status set to false after 500 ms of no detections within ROI.");
      }
    }

    // Vector to store detected objects with their distances
    std::vector<std::pair<float, DrwigDetectedObject>> objects_with_distance;

    // Calculate distances and store them
    for (const auto &obj : detected_object)
    {
      float x = static_cast<float>(obj.x) / result;
      float y = static_cast<float>(obj.y) / result;
      float z = static_cast<float>(obj.z) / result;
      float range = sqrt(x * x + y * y + z * z);

      // Store the object and its distance
      objects_with_distance.emplace_back(range, obj);
    }

    // Sort objects by distance (ascending order)
    std::sort(objects_with_distance.begin(), objects_with_distance.end(),
              [](const auto &a, const auto &b)
              {
                return a.first < b.first; // Compare distances
              });

    // Process detected objects
    for (const auto &pair : objects_with_distance)
    {
      const auto &obj = pair.second; // Get the detected object
      std::cout << "\n - x: " << (obj.x / result) << ", y: " << (obj.y / result)
                << ", z: " << (obj.z / result) << ", range: " << pair.first
                << ", doppler_velocity: " << obj.doppler_velovity
                << ", peak_value: " << obj.peak_value;

      std::ostringstream obj_log;
      obj_log << "x: " << (obj.x / result) << ", y: " << (obj.y / result)
              << ", z: " << (obj.z / result) << ", range: " << pair.first
              << ", doppler_velocity: " << obj.doppler_velovity
              << ", peak_value: " << obj.peak_value;
      logData("Detected Object", obj_log.str());

      radar_node::msg::DetectedObj pub_obj;
      pub_obj.x = static_cast<float>(obj.x) / result;
      pub_obj.y = static_cast<float>(obj.y) / result;
      pub_obj.z = static_cast<float>(obj.z) / result;
      pub_obj.doppler_velovity = obj.doppler_velovity;
      pub_obj.peak_value = obj.peak_value;

      // Check brake status for individual objects
      pub_obj.brake_status = shouldBrake(obj, header, 0, 0, pub_obj.x, pub_obj.y);
      pub_msg->detected_obj.push_back(pub_obj);
    }

    // Finalize the brake status for the message
    pub_msg->brake_status = brake_status_flag; // This will be true if any object requires braking
    detected_brake_status_ = brake_status_flag;
    detected_publisher_->publish(*pub_msg);
    publishCombinedBrake();
    logData("Brake Status", brake_status_flag ? "True" : "False");
  }

  void handleTrackedData(const DrwigObjectHeader &header, const std::vector<DrwigTrackedObject> &tracked_object)
  {
    RCLCPP_INFO(this->get_logger(), "Tracked object data received.");
    auto pub_msg = std::make_shared<radar_node::msg::ArrayTrackedObj>();

    std::cout << "\nTotal Number of Tracked Objects: " << header.num_obj;
    double result = pow(2, header.format);

    std::ostringstream tracked_log;
    tracked_log << "Total Tracked Objects: " << header.num_obj;
    logData("Tracked Objects", tracked_log.str());

    // Vector to store tracked objects with their distances
    std::vector<std::pair<float, DrwigTrackedObject>> objects_with_distance;

    // Calculate distances and store them
    for (const auto &obj : tracked_object)
    {
      float x_pos = static_cast<float>(obj.x) / result;
      float y_pos = static_cast<float>(obj.y) / result;
      float distance = sqrt(x_pos * x_pos + y_pos * y_pos);
      objects_with_distance.emplace_back(distance, obj);
    }

    // Sort objects by distance (ascending order)
    std::sort(objects_with_distance.begin(), objects_with_distance.end(),
              [](const auto &a, const auto &b)
              {
                return a.first < b.first; // Compare distances
              });

    bool brake_status_flag = false;

    for (const auto &pair : objects_with_distance)
    {
      const auto &obj = pair.second; // Get the tracked object

      radar_node::msg::TrackedObj pub_obj;
      float x_pos = static_cast<float>(obj.x) / result;
      float y_pos = static_cast<float>(obj.y) / result;
      float vx = static_cast<float>(obj.vx) / result;
      float vy = static_cast<float>(obj.vy) / result;
      float width = static_cast<float>(obj.width) / result;
      float length = static_cast<float>(obj.length) / result;
      // Calculate Doppler velocity
      float doppler_velocity = (vx * x_pos + vy * y_pos) / sqrt(x_pos * x_pos + y_pos * y_pos);

      std::cout << "\n - x_pos: " << x_pos << ", y_pos: " << y_pos
                << " - vx: " << vx << ", vy: " << vy
                << ", doppler_velocity: " << doppler_velocity
                << ", width: " << width << ", length: " << length;

      std::ostringstream obj_log;
      obj_log << "x_pos: " << (obj.x / result) << ", y_pos: " << (obj.y / result)
              << ", vx: " << (obj.vx / result) << ", vy: " << (obj.vy / result)
              << ", doppler_velocity: " << doppler_velocity
              << ", width: " << (obj.width / result) << ", length: " << (obj.length / result);
      logData("Tracked Object", obj_log.str());

      pub_obj.x_pos = x_pos;
      pub_obj.y_pos = y_pos;
      pub_obj.vx = vx;
      pub_obj.vy = vy;
      pub_obj.width = width;
      pub_obj.length = length;

      // Check brake status using the unified function
      pub_obj.brake_status = shouldBrake(obj, header, pub_obj.width, pub_obj.length, x_pos, y_pos);

      if (pub_obj.brake_status)
      {
        brake_status_flag = true;
      }

      pub_msg->tracked_obj.push_back(pub_obj);
    }

    if (brake_status_flag)
    {
      pub_msg->brake_status = true;
    }
    else
    {
      pub_msg->brake_status = false;
    }
    tracked_brake_status_ = brake_status_flag;
    tracked_publisher_->publish(*pub_msg);
    publishCombinedBrake();
    logData("Brake Status", brake_status_flag ? "True" : "False");
  }
  
  template <typename T>
  bool shouldBrake(const T &object, const DrwigObjectHeader &header, float width, float length, float x, float y)
  {
    const float roi_x_limit = 1.0;     // X position limit
    const float roi_range_limit = 5.0; // Range limit
    const float ttc_threshold = 3.0;   // Time to collision threshold
    const float immediate_stop_limit = 4.0;

    float range = sqrt(x * x + y * y);
    float doppler_velocity;

    // Determine doppler_velocity based on the type of object
    if constexpr (std::is_same_v<T, DrwigTrackedObject>)
    {
      doppler_velocity = (x * object.vx + y * object.vy) / range;
    }
    else if constexpr (std::is_same_v<T, DrwigDetectedObject>)
    {
      doppler_velocity = object.doppler_velovity;
    }
    else if constexpr (std::is_same_v<T, DrwigClusterObject>)
    {
      float x_center = static_cast<float>(object.x) / pow(2, header.format);
      float y_center = static_cast<float>(object.y) / pow(2, header.format);
    }
    else
    {
      return false; // Unsupported type
    }

    // Check if the object is within ROI
    if (fabs(x) <= roi_x_limit && fabs(y) <= roi_range_limit)
    {
      if (fabs(y) <= immediate_stop_limit)
      {
        return true; // Immediate stop condition
      }
      // Calculate Time to Collision (TTC)
      if (doppler_velocity < 0)
      {
        float ttc = range / fabs(doppler_velocity);
        return ttc < ttc_threshold; // Activate brakes if condition met

      }
    }
    return false; // No braking needed
    
    // if (fabs(x) <= roi_x_limit && fabs(y) <= roi_range_limit) {
    //   if constexpr (std::is_same_v<T, DrwigTrackedObject>) {
    //       // Moving object logic
    //       if (doppler_velocity < 0) {
    //           float ttc = range / fabs(doppler_velocity);
    //           return ttc < ttc_threshold; // Activate brakes if condition met
    //       }
    //   } else if constexpr (std::is_same_v<T, DrwigDetectedObject>) {
    //       // Stationary object logic
    //       return fabs(y) <= immediate_stop_limit; // Immediate stop condition
    //   } else if constexpr (std::is_same_v<T, DrwigClusterObject>) {
    //       // Cluster object logic: Calculate safe distance based on the object's dimensions
    //       float distance = sqrt(x * x + y * y); // Distance from the vehicle to the cluster object
    //       float safe_distance = (width + length) / 2.0 + 1.0; // Safe distance margin
    //       return distance < safe_distance; // Activate brakes if the cluster object is too close
    //   }
    // } 
    // // No braking needed
    // return false;
  
  }

  void publishCombinedBrake()
  {
    auto combined_msg = std::make_shared<radar_node::msg::ArrayTrackedObj>();

    // Set the overall brake status based on individual statuses
    combined_msg->brake_status = detected_brake_status_ || tracked_brake_status_ || clustered_brake_status_;

    // Publish the combined tracked object
    combined_tracked_publisher_->publish(*combined_msg);

    logData("Combined Brake Status", combined_msg->brake_status ? "True" : "False");
  }

private:
  int count_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<radar_node::msg::ArrayTrackedObj>::SharedPtr tracked_publisher_;
  rclcpp::Publisher<radar_node::msg::ArrayClusterObj>::SharedPtr cluster_publisher_;
  rclcpp::Publisher<radar_node::msg::ArrayDetectedObj>::SharedPtr detected_publisher_;
  rclcpp::Publisher<radar_node::msg::ArrayTrackedObj>::SharedPtr combined_tracked_publisher_;
  DrwigParser parser;
  DrwigClusterObjectHandler drwig_cluster_object;
  DrwigTrackedObjectHandler drwig_tracked_object;
  DrwigDetectObjectHandler drwig_detected_object;
  // static std::chrono_literals::TimePoint last_detection_time = std::chrono::system_clock::now();

private:
  bool detected_brake_status_ = false;
  bool tracked_brake_status_ = false;
  bool clustered_brake_status_ = false;
  std::ofstream log_file_; // File stream for logging
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  uint32_t buad_rate = 926100;
  std::string port1 = "/dev/ttyACM1";
  std::string port2 = "/dev/ttyACM3";
  std::string port3 = "/dev/ttyACM5";
  rclcpp::executors::MultiThreadedExecutor executor;
  auto radar1 = std::make_shared<RadarPubNode>("radar_node_0", port1, buad_rate);
  // auto radar2 = std::make_shared<RadarPubNode>("radar_node_1",port2, buad_rate);
  // auto radar3 = std::make_shared<RadarPubNode>("radar_node_2",port3, buad_rate);

  executor.add_node(radar1);
  // executor.add_node(radar2);
  // executor.add_node(radar3);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
