#include <glim_ros/raw_points.hpp>

namespace glim_ros {

template <typename T>
Eigen::Vector4d get_vec4(const void* x, const void* y, const void* z) {
  return Eigen::Vector4d(*reinterpret_cast<const T*>(x), *reinterpret_cast<const T*>(y), *reinterpret_cast<const T*>(z), 1.0);
}

RawPoints::Ptr RawPoints::extract(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& points_msg) {
  int num_points = points_msg->width * points_msg->height;

  int x_type = 0;
  int y_type = 0;
  int z_type = 0;
  int time_type = 0;
  int intensity_type = 0;

  int x_offset = -1;
  int y_offset = -1;
  int z_offset = -1;
  int time_offset = -1;
  int intensity_offset = -1;

  std::unordered_map<std::string, std::pair<int*, int*>> fields;
  fields["x"] = std::make_pair(&x_type, &x_offset);
  fields["y"] = std::make_pair(&y_type, &y_offset);
  fields["z"] = std::make_pair(&z_type, &z_offset);
  fields["time"] = std::make_pair(&time_type, &time_offset);
  fields["intensity"] = std::make_pair(&intensity_type, &intensity_offset);

  for (const auto& field : points_msg->fields) {
    auto found = fields.find(field.name);
    if (found == fields.end()) {
      continue;
    }

    *found->second.first = field.datatype;
    *found->second.second = field.offset;
  }

  if (x_offset < 0 || y_offset < 0 || z_offset < 0) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "missing point coordinate fields");
    return nullptr;
  }

  const auto FLOAT32 = sensor_msgs::msg::PointField::FLOAT32;
  const auto FLOAT64 = sensor_msgs::msg::PointField::FLOAT64;
  if ((x_type != FLOAT32 && x_type != FLOAT64) || x_type != y_type || x_type != y_type) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "unsupported points type");
    return nullptr;
  }

  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> points;
  points.resize(num_points);
  for (int i = 0; i < num_points; i++) {
    const auto* x_ptr = &points_msg->data[points_msg->point_step * i + x_offset];
    const auto* y_ptr = &points_msg->data[points_msg->point_step * i + y_offset];
    const auto* z_ptr = &points_msg->data[points_msg->point_step * i + z_offset];

    if (x_type == FLOAT32) {
      points[i] = get_vec4<float>(x_ptr, y_ptr, z_ptr);
    } else {
      points[i] = get_vec4<double>(x_ptr, y_ptr, z_ptr);
    }
  }

  std::vector<double> times;
  if (time_offset >= 0) {
    times.resize(num_points);

    for (int i = 0; i < num_points; i++) {
      const auto* time_ptr = &points_msg->data[points_msg->point_step * i + time_offset];
      if(time_type == FLOAT32) {
        times[i] = *reinterpret_cast<const float*>(time_ptr);
      } else if (time_type == FLOAT64) {
        times[i] = *reinterpret_cast<const double*>(time_ptr);
      } else {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "unsupported time type");
        return nullptr;
      }
    }
  }

  std::vector<double> intensities;
  if(intensity_offset >= 0) {
    intensities.resize(num_points);

    for (int i = 0; i < num_points; i++) {
      const auto* intensity_ptr = &points_msg->data[points_msg->point_step * i + intensity_offset];
      if (intensity_type == FLOAT32) {
        intensities[i] = *reinterpret_cast<const float*>(intensity_ptr);
      } else if (intensity_type == FLOAT64) {
        intensities[i] = *reinterpret_cast<const double*>(intensity_ptr);
      } else {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "unsupported intensity type");
        return nullptr;
      }
    }
  }

  const double stamp = points_msg->header.stamp.sec + points_msg->header.stamp.nanosec / 1e9;
  return RawPoints::Ptr(new RawPoints{stamp, times, intensities, points});
}
}