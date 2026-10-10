#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <std_msgs/msg/bool.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

using Cloud = sensor_msgs::msg::PointCloud2;
using Field = sensor_msgs::msg::PointField;

template<class T> T read_value(const uint8_t *ptr) {
  T value; std::memcpy(&value, ptr, sizeof(T)); return value;
}

class Adapter : public rclcpp::Node {
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fault_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Subscription<Cloud>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  double accel_scale_, max_span_;
  int64_t last_cloud_ = -1, last_imu_ = -1;
  bool fault_ = false;
  size_t drops_ = 0, clouds_ = 0, imus_ = 0;

  void diagnostic(const std::string &reason, uint8_t level) {
    diagnostic_msgs::msg::DiagnosticArray msg;
    msg.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus s;
    s.name = "towergo/livox_adapter"; s.level = level;
    s.message = reason + "; clouds=" + std::to_string(clouds_) +
      "; imus=" + std::to_string(imus_) + "; dropped=" + std::to_string(drops_);
    msg.status.push_back(s); diag_pub_->publish(msg);
  }
  void fault(const std::string &reason) {
    fault_ = true;
    std_msgs::msg::Bool value; value.data = true; fault_pub_->publish(value);
    diagnostic(reason + "; new mapping session required", 2);
  }
  bool check_stamp(int64_t stamp, int64_t &last) {
    if (stamp <= 0 || (last >= 0 && stamp <= last)) {
      fault("Sensor timestamp reset/repeated"); return false;
    }
    last = stamp; return true;
  }
  void cloud(Cloud::ConstSharedPtr m) {
    if (fault_) return;
    const auto header_ns = rclcpp::Time(m->header.stamp).nanoseconds();
    if (!check_stamp(header_ns, last_cloud_)) return;
    try {
      if (m->header.frame_id != "livox_frame" || m->is_bigendian || m->width == 0 || m->height == 0)
        throw std::runtime_error("Unexpected frame/endianness/empty cloud");
      if (m->point_step == 0 || static_cast<uint64_t>(m->row_step) < static_cast<uint64_t>(m->width)*m->point_step ||
          m->data.size() < static_cast<uint64_t>(m->height)*m->row_step)
        throw std::runtime_error("Truncated/invalid PointCloud2 layout");
      std::map<std::string, size_t> off;
      const std::map<std::string, std::pair<int, size_t>> expected = {
        {"x", {Field::FLOAT32, 4}}, {"y", {Field::FLOAT32, 4}}, {"z", {Field::FLOAT32, 4}},
        {"intensity", {Field::FLOAT32, 4}}, {"timestamp", {Field::FLOAT64, 8}},
        {"tag", {Field::UINT8, 1}}, {"line", {Field::UINT8, 1}}};
      for (const auto &f : m->fields) {
        const auto e = expected.find(f.name);
        if (e == expected.end()) continue;
        if (off.count(f.name) || f.datatype != e->second.first || f.count != 1 ||
            static_cast<uint64_t>(f.offset) + e->second.second > m->point_step)
          throw std::runtime_error("Invalid/duplicate field: " + f.name);
        off[f.name] = f.offset;
      }
      if (off.size() != expected.size()) throw std::runtime_error("Required point time/field missing");
      livox_ros_driver2::msg::CustomMsg out;
      out.header = m->header; out.header.frame_id = "lidar_front";
      out.timebase = static_cast<uint64_t>(header_ns); out.lidar_id = 0;
      out.points.reserve(static_cast<size_t>(m->height)*m->width);
      int64_t previous_offset = -1;
      for (size_t row = 0; row < m->height; ++row) {
        for (size_t col = 0; col < m->width; ++col) {
          const uint8_t *p = m->data.data() + row*m->row_step + col*m->point_step;
          livox_ros_driver2::msg::CustomPoint q;
          q.x = read_value<float>(p+off.at("x")); q.y = read_value<float>(p+off.at("y"));
          q.z = read_value<float>(p+off.at("z"));
          const float intensity = read_value<float>(p+off.at("intensity"));
          const double ns = read_value<double>(p+off.at("timestamp"));
          if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) ||
              !std::isfinite(intensity) || !std::isfinite(ns) || intensity < 0 || intensity > 255)
            throw std::runtime_error("Non-finite point or invalid reflectivity");
          // float64 epoch nanoseconds have ~256 ns spacing today. Subtract in
          // long double and allow ONLY sub-microsecond quantization at t=0.
          const long double delta = static_cast<long double>(ns) - static_cast<long double>(header_ns);
          if (delta < -1000.L || delta > max_span_*1e9L)
            throw std::runtime_error("Point time is not absolute ns within this frame");
          const int64_t offset = std::max<int64_t>(0, std::llround(delta));
          if (offset < previous_offset) throw std::runtime_error("Point times go backwards");
          previous_offset = offset;
          q.offset_time = static_cast<uint32_t>(offset);
          q.reflectivity = static_cast<uint8_t>(std::lround(intensity));
          q.line = *(p+off.at("line")); q.tag = *(p+off.at("tag"));
          if (q.line >= 4) throw std::runtime_error("MID360 line index outside [0,3]");
          out.points.push_back(q);
        }
      }
      if (previous_offset <= 0) throw std::runtime_error("No per-point time span");
      out.point_num = static_cast<uint32_t>(out.points.size());
      cloud_pub_->publish(out); ++clouds_;
      if (clouds_ % 10 == 0) diagnostic("Preserving actual point times", 0);
    } catch (const std::exception &e) {
      ++drops_; diagnostic(e.what(), 2);
    }
  }
  void imu(sensor_msgs::msg::Imu::ConstSharedPtr m) {
    if (fault_) return;
    if (!check_stamp(rclcpp::Time(m->header.stamp).nanoseconds(), last_imu_)) return;
    const auto &a = m->linear_acceleration; const auto &g = m->angular_velocity;
    if (m->header.frame_id != "livox_frame" || !std::isfinite(a.x) || !std::isfinite(a.y) ||
        !std::isfinite(a.z) || !std::isfinite(g.x) || !std::isfinite(g.y) || !std::isfinite(g.z)) {
      ++drops_; diagnostic("Invalid IMU frame/value", 2); return;
    }
    auto out = *m; out.header.frame_id = "imu_front";
    out.linear_acceleration.x *= accel_scale_; out.linear_acceleration.y *= accel_scale_;
    out.linear_acceleration.z *= accel_scale_;
    if (out.linear_acceleration_covariance[0] != -1)
      for (auto &v : out.linear_acceleration_covariance) v *= accel_scale_*accel_scale_;
    imu_pub_->publish(out); ++imus_;
  }
public:
  Adapter() : Node("towergo_livox_adapter") {
    const auto unit = declare_parameter("imu_acceleration_unit", std::string("unverified"));
    if (unit != "g" && unit != "m_s2") throw std::runtime_error("Explicit IMU acceleration unit required");
    accel_scale_ = unit == "g" ? 9.80665 : 1.;
    max_span_ = declare_parameter("max_frame_span", .2);
    if (!(max_span_ > 0 && max_span_ <= 1.)) throw std::runtime_error("Invalid frame span");
    cloud_pub_ = create_publisher<livox_ros_driver2::msg::CustomMsg>("/towergo/lio/lidar", 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/towergo/lio/imu", 400);
    fault_pub_ = create_publisher<std_msgs::msg::Bool>("/towergo/mapping_fault", rclcpp::QoS(1).transient_local());
    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    cloud_sub_ = create_subscription<Cloud>("/livox/lidar_front", rclcpp::SensorDataQoS(),
      [this](Cloud::ConstSharedPtr m){cloud(m);});
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("/livox/imu_front", rclcpp::SensorDataQoS().keep_last(400),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr m){imu(m);});
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<Adapter>()); }
  catch (const std::exception &e) { std::fprintf(stderr, "%s\n", e.what()); rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
