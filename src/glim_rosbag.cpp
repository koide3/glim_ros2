#include <glob.h>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <glim/util/config.hpp>
#include <glim_ros/glim_ros.hpp>

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: glim_rosbag input_rosbag_path" << std::endl;
    return 0;
  }

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto glim = std::make_shared<glim::GlimROS>(options);

  // List topics
  const std::string config_ros_path = ament_index_cpp::get_package_share_directory("glim_ros") + "/config/glim_ros.json";
  glim::Config config_ros(config_ros_path);

  const auto topics = config_ros.param<std::vector<std::string>>("glim_rosbag", "topics");
  if (!topics) {
    std::cerr << "error: topics must be specified" << std::endl;
    return 1;
  }

  rosbag2_storage::StorageFilter filter;
  std::cout << "topics:" << std::endl;
  for (const auto& topic : *topics) {
    std::cout << "- " << topic << std::endl;
    filter.topics.push_back(topic);
  }

  // List input rosbag filenames
  std::vector<std::string> bag_filenames;

  for (int i = 1; i < argc; i++) {
    std::vector<std::string> filenames;
    glob_t globbuf;
    int ret = glob(argv[i], 0, nullptr, &globbuf);
    for (int i = 0; i < globbuf.gl_pathc; i++) {
      filenames.push_back(globbuf.gl_pathv[i]);
    }
    globfree(&globbuf);

    bag_filenames.insert(bag_filenames.end(), filenames.begin(), filenames.end());
  }
  std::sort(bag_filenames.begin(), bag_filenames.end());

  std::cout << "bag_filenames:" << std::endl;
  for (const auto& bag_filename : bag_filenames) {
    std::cout << "- " << bag_filename << std::endl;
  }

  const double playback_speed = config_ros.param<double>("glim_rosbag", "playback_speed", 1.0);
  const auto real_t0 = std::chrono::high_resolution_clock::now();
  rcutils_time_point_value_t bag_t0 = 0;

  // Bag read function
  const auto read_bag = [&](const std::string& bag_filename) {
    std::cout << "opening " << bag_filename << std::endl;
    rosbag2_cpp::Reader reader;
    reader.open(bag_filename);
    reader.set_filter(filter);

    const auto topics_and_types = reader.get_all_topics_and_types();
    std::unordered_map<std::string, std::string> topic_type_map;
    for (const auto& topic : topics_and_types) {
      topic_type_map[topic.name] = topic.type;
    }

    rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serialization;
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> points_serialization;
    rclcpp::Serialization<sensor_msgs::msg::Image> image_serialization;
    rclcpp::Serialization<sensor_msgs::msg::CompressedImage> compressed_image_serialization;

    while (reader.has_next()) {
      if (!glim->ok()) {
        return false;
      }

      const auto msg = reader.read_next();
      const std::string topic_type = topic_type_map[msg->topic_name];
      const rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);

      if (bag_t0 == 0) {
        bag_t0 = msg->time_stamp;
      }

      const auto bag_elapsed = std::chrono::nanoseconds(msg->time_stamp - bag_t0);
      while (playback_speed > 0.0 && (std::chrono::high_resolution_clock::now() - real_t0) * playback_speed < bag_elapsed) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      if (topic_type == "sensor_msgs/msg/Imu") {
        auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
        imu_serialization.deserialize_message(&serialized_msg, imu_msg.get());
        glim->imu_callback(imu_msg);
      } else if (topic_type == "sensor_msgs/msg/PointCloud2") {
        auto points_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        points_serialization.deserialize_message(&serialized_msg, points_msg.get());
        glim->points_callback(points_msg);
      } else if (topic_type == "sensor_msgs/msg/Image") {
        auto image_msg = std::make_shared<sensor_msgs::msg::Image>();
        image_serialization.deserialize_message(&serialized_msg, image_msg.get());
        glim->image_callback(image_msg);
      } else if (topic_type == "sensor_msgs/msg/CompressedImage") {
        auto compressed_image_msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
        compressed_image_serialization.deserialize_message(&serialized_msg, compressed_image_msg.get());
      }

      glim->timer_callback();
    }

    return true;
  };

  // Read all rosbags
  for (const auto& bag_filename : bag_filenames) {
    if (!read_bag(bag_filename)) {
      break;
    }
  }

  while (glim->ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    glim->timer_callback();
  }

  glim->save("/tmp/dump");

  return 0;
}