#include <glob.h>
#include <chrono>
#include <iostream>
#include <spdlog/spdlog.h>
#include <boost/format.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_compression/sequential_compression_reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <glim/util/config.hpp>
#include <glim/util/extension_module_ros2.hpp>
#include <glim_ros/glim_ros.hpp>
#include <glim_ros/ros_compatibility.hpp>

class SpeedCounter {
public:
  SpeedCounter() : last_sim_time(0.0), last_real_time(std::chrono::high_resolution_clock::now()) {}

  void update(const double& stamp) {
    const auto now = std::chrono::high_resolution_clock::now();
    if (now - last_real_time < std::chrono::seconds(5)) {
      return;
    }

    if (last_sim_time > 0.0) {
      const auto real = now - last_real_time;
      const auto sim = stamp - last_sim_time;
      const double playback_speed = sim / (std::chrono::duration_cast<std::chrono::nanoseconds>(real).count() / 1e9);
      spdlog::info("playback speed: {:.3f}x", playback_speed);
    }

    last_sim_time = stamp;
    last_real_time = now;
  }

private:
  double last_sim_time;
  std::chrono::high_resolution_clock::time_point last_real_time;
};

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: glim_rosbag input_rosbag_path" << std::endl;
    return 0;
  }

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto glim = std::make_shared<glim::GlimROS>(options);

  // List topics
  glim::Config config_ros(glim::GlobalConfig::get_config_path("config_ros"));

  const std::string imu_topic = config_ros.param<std::string>("glim_ros", "imu_topic", "/imu");
  const std::string points_topic = config_ros.param<std::string>("glim_ros", "points_topic", "/points");
  const std::string image_topic = config_ros.param<std::string>("glim_ros", "image_topic", "/image");
  std::vector<std::string> topics = {imu_topic, points_topic, image_topic};

  rosbag2_storage::StorageFilter filter;
  spdlog::info("topics:");
  for (const auto& topic : topics) {
    spdlog::info("- {}", topic);
    filter.topics.push_back(topic);
  }

  //
  std::unordered_map<std::string, std::vector<glim::GenericTopicSubscription::Ptr>> subscription_map;
  for (const auto& sub : glim->extension_subscriptions()) {
    spdlog::info("- {} (ext)", sub->topic);
    filter.topics.push_back(sub->topic);
    subscription_map[sub->topic].push_back(sub);
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

  spdlog::info("bag_filenames:");
  for (const auto& bag_filename : bag_filenames) {
    spdlog::info("- {}", bag_filename);
  }

  // Playback range settings
  double delay = 0.0;
  glim->declare_parameter<double>("delay", delay);
  glim->get_parameter<double>("delay", delay);

  double start_offset = 0.0;
  glim->declare_parameter<double>("start_offset", start_offset);
  glim->get_parameter<double>("start_offset", start_offset);

  double playback_duration = 0.0;
  glim->declare_parameter<double>("playback_duration", playback_duration);
  glim->get_parameter<double>("playback_duration", playback_duration);

  double playback_until = 0.0;
  glim->declare_parameter<double>("playback_until", playback_until);
  glim->get_parameter<double>("playback_until", playback_until);

  // Playback speed settings
  const double playback_speed = config_ros.param<double>("glim_ros", "playback_speed", 100.0);
  std::chrono::high_resolution_clock::time_point real_t0;
  rcutils_time_point_value_t bag_t0 = 0;
  SpeedCounter speed_counter;

  double end_time = std::numeric_limits<double>::max();
  glim->declare_parameter<double>("end_time", end_time);
  glim->get_parameter<double>("end_time", end_time);

  if (delay > 0.0) {
    spdlog::info("delaying {} sec", delay);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(delay * 1000)));
  }

  // Bag read function
  const auto read_bag = [&](const std::string& bag_filename) {
    spdlog::info("opening {}", bag_filename);
    rosbag2_storage::StorageOptions options;
    options.uri = bag_filename;

    rosbag2_cpp::ConverterOptions converter_options;

    // rosbag2_cpp::Reader reader;
    std::unique_ptr<rosbag2_cpp::reader_interfaces::BaseReaderInterface> reader_;
    reader_ = std::make_unique<rosbag2_cpp::readers::SequentialReader>();
    reader_->open(options, converter_options);

    if (reader_->get_metadata().compression_format != "") {
      spdlog::info("compression detected (format={})", reader_->get_metadata().compression_format);
      spdlog::info("opening bag with SequentialCompressionReader");
      reader_ = std::make_unique<rosbag2_compression::SequentialCompressionReader>();
      reader_->open(options, converter_options);
    }

    auto& reader = *reader_;
    reader.set_filter(filter);

    const auto topics_and_types = reader.get_all_topics_and_types();
    std::unordered_map<std::string, std::string> topic_type_map;
    for (const auto& topic : topics_and_types) {
      topic_type_map[topic.name] = topic.type;
    }

    rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serialization;
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> points_serialization;
#ifdef BUILD_WITH_CV_BRIDGE
    rclcpp::Serialization<sensor_msgs::msg::Image> image_serialization;
    rclcpp::Serialization<sensor_msgs::msg::CompressedImage> compressed_image_serialization;
#endif

    while (reader.has_next()) {
      if (!rclcpp::ok()) {
        return false;
      }
      rclcpp::spin_some(glim);

      const auto msg = reader.read_next();
      const std::string topic_type = topic_type_map[msg->topic_name];
      const rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);

      if (real_t0.time_since_epoch().count() == 0) {
        real_t0 = std::chrono::high_resolution_clock::now();
      }

      const auto msg_time = get_msg_recv_timestamp(*msg);
      if (bag_t0 == 0) {
        bag_t0 = msg_time;
      }
      spdlog::debug("msg_time: {} ({} sec)", msg_time / 1e9, (msg_time - bag_t0) / 1e9);

      if (start_offset > 0.0) {
        spdlog::info("skipping msg for start_offset {}", start_offset);
        reader.seek(bag_t0 + start_offset * 1e9);

        start_offset = 0.0;
        bag_t0 = 0;
        real_t0 = std::chrono::high_resolution_clock::from_time_t(0);
        continue;
      }

      if (playback_until > 0.0 && msg_time / 1e9 > playback_until) {
        spdlog::info("reached playback_until ({} < {})", msg_time / 1e9, playback_until);
        return false;
      }

      if (playback_duration > 0.0 && (msg_time - bag_t0) / 1e9 > playback_duration) {
        spdlog::info("reached playback_duration ({} > {})", (msg_time - bag_t0) / 1e9, playback_duration);
        return false;
      }

      const auto bag_elapsed = std::chrono::nanoseconds(msg_time - bag_t0);
      while (playback_speed > 0.0 && (std::chrono::high_resolution_clock::now() - real_t0) * playback_speed < bag_elapsed) {
        const double real_elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - real_t0).count() / 1e9;
        spdlog::debug("throttling (real_elapsed={} bag_elapsed={} playback_speed={})", real_elapsed, bag_elapsed.count() / 1e9, playback_speed);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      if (msg->topic_name == imu_topic) {
        if (topic_type != "sensor_msgs/msg/Imu") {
          spdlog::error("topic_type mismatch: {} != sensor_msgs/msg/Imu (topic={})", topic_type, msg->topic_name);
          return false;
        }
        auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
        imu_serialization.deserialize_message(&serialized_msg, imu_msg.get());
        glim->imu_callback(imu_msg);
      } else if (msg->topic_name == points_topic) {
        if (topic_type != "sensor_msgs/msg/PointCloud2") {
          spdlog::error("topic_type mismatch: {} != sensor_msgs/msg/PointCloud2 (topic={})", topic_type, msg->topic_name);
          return false;
        }
        auto points_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        points_serialization.deserialize_message(&serialized_msg, points_msg.get());
        const size_t workload = glim->points_callback(points_msg);

        if (points_msg->header.stamp.sec + points_msg->header.stamp.nanosec * 1e-9 > end_time) {
          spdlog::info("end_time reached");
          return false;
        }

        if (workload > 5) {
          // Odometry estimation is behind
          const size_t sleep_msec = (workload - 4) * 5;
          spdlog::debug("throttling: {} msec (workload={})", sleep_msec, workload);
          std::this_thread::sleep_for(std::chrono::milliseconds(sleep_msec));
        }
      }
#ifdef BUILD_WITH_CV_BRIDGE
      else if (msg->topic_name == image_topic) {
        if (topic_type == "sensor_msgs/msg/Image") {
          auto image_msg = std::make_shared<sensor_msgs::msg::Image>();
          image_serialization.deserialize_message(&serialized_msg, image_msg.get());
          glim->image_callback(image_msg);
        } else if (topic_type == "sensor_msgs/msg/CompressedImage") {
          auto compressed_image_msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
          compressed_image_serialization.deserialize_message(&serialized_msg, compressed_image_msg.get());

          auto image_msg = std::make_shared<sensor_msgs::msg::Image>();
          cv_bridge::toCvCopy(*compressed_image_msg, "bgr8")->toImageMsg(*image_msg);
          glim->image_callback(image_msg);
        } else {
          spdlog::error("topic_type mismatch: {} != sensor_msgs/msg/(Image|CompressedImage) (topic={})", topic_type, msg->topic_name);
          return false;
        }
      }
#endif

      auto found = subscription_map.find(msg->topic_name);
      if (found != subscription_map.end()) {
        for (const auto& sub : found->second) {
          sub->insert_message_instance(serialized_msg, topic_type);
        }
      }

      glim->timer_callback();
      speed_counter.update(msg_time / 1e9);

      const auto t0 = std::chrono::high_resolution_clock::now();
      while (glim->needs_wait()) {
        rclcpp::spin_some(glim);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        spdlog::debug("throttling (waiting for odometry estimation)");
        if (std::chrono::high_resolution_clock::now() - t0 > std::chrono::seconds(1)) {
          spdlog::warn("throttling timeout (an extension module may be hanged)");
          break;
        }
      }
    }

    return true;
  };

  // Read all rosbags
  bool auto_quit = false;
  glim->declare_parameter<bool>("auto_quit", auto_quit);
  glim->get_parameter<bool>("auto_quit", auto_quit);

  std::string dump_path = "/tmp/dump";
  glim->declare_parameter<std::string>("dump_path", dump_path);
  glim->get_parameter<std::string>("dump_path", dump_path);

  for (const auto& bag_filename : bag_filenames) {
    if (!read_bag(bag_filename)) {
      auto_quit = true;
      break;
    }
  }

  if (!auto_quit) {
    rclcpp::spin(glim);
  }

  glim->wait(auto_quit);
  glim->save(dump_path);

  return 0;
}