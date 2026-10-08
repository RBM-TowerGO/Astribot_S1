#include "fusion.hpp"
#include <chrono>
#include <cmath>
#include <cstring>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>

using Cloud = sensor_msgs::msg::PointCloud2;
using Field = sensor_msgs::msg::PointField;
using Clock = std::chrono::steady_clock;
template <class T> T read(const uint8_t *p) {
  T v;
  std::memcpy(&v, p, sizeof(T));
  return v;
}
class Adapter : public rclcpp::Node {
  dual::Fusion fusion;
  bool failed = false, started = false;
  int64_t last_imu = -1;
  uint64_t windows = 0;
  double max_callback_ms = 0;
  Clock::time_point last_output = Clock::now();
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr pub;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fault_pub;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub;
  std::array<rclcpp::Subscription<Cloud>::SharedPtr, 2> subs;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::TimerBase::SharedPtr timer;
  std::string reason = "Waiting for both lidars";
  void diagnostic() {
    diagnostic_msgs::msg::DiagnosticArray msg;
    msg.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus st;
    st.name = "towergo/dual_livox";
    st.level = failed ? 2 : 0;
    st.message = reason;
    auto add = [&](const std::string &k, const auto &v) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = k;
      kv.value = std::to_string(v);
      st.values.push_back(kv);
    };
    for (int s = 0; s < 2; ++s) {
      std::string prefix = s ? "back_" : "front_";
      add(prefix + "input_points", fusion.input[s]);
      add(prefix + "fused_points", fusion.emitted[s]);
      add(prefix + "initial_discarded", fusion.discarded[s]);
      add(prefix + "gap_discarded", fusion.gap_discarded[s]);
      add(prefix + "expired_points", fusion.expired[s]);
      add(prefix + "buffered_points", fusion.queue[s].size());
    }
    add("windows", windows);
    add("skipped_windows", fusion.skipped_windows);
    add("max_callback_ms", max_callback_ms);
    msg.status.push_back(st);
    diag_pub->publish(msg);
  }
  void fault(const std::string &text) {
    if (failed)
      return;
    failed = true;
    reason = text;
    std_msgs::msg::Bool b;
    b.data = true;
    diagnostic();
    fault_pub->publish(b);
    RCLCPP_ERROR(get_logger(), "%s", text.c_str());
  }
  std::vector<dual::Point> parse(const Cloud &m, int64_t header) {
    if (m.header.frame_id != "livox_frame" || m.is_bigendian || !m.width ||
        !m.height || !m.point_step || uint64_t(m.width) * m.height > 100000 ||
        uint64_t(m.row_step) < uint64_t(m.width) * m.point_step ||
        m.data.size() < uint64_t(m.height) * m.row_step)
      throw std::runtime_error("Invalid cloud frame/layout");
    const std::map<std::string, std::pair<int, size_t>> expected = {
        {"x", {Field::FLOAT32, 4}},         {"y", {Field::FLOAT32, 4}},
        {"z", {Field::FLOAT32, 4}},         {"intensity", {Field::FLOAT32, 4}},
        {"timestamp", {Field::FLOAT64, 8}}, {"tag", {Field::UINT8, 1}},
        {"line", {Field::UINT8, 1}}};
    std::map<std::string, size_t> off;
    for (auto &f : m.fields) {
      auto it = expected.find(f.name);
      if (it == expected.end())
        continue;
      if (off.count(f.name) || f.count != 1 || f.datatype != it->second.first ||
          uint64_t(f.offset) + it->second.second > m.point_step)
        throw std::runtime_error("Invalid point field");
      off[f.name] = f.offset;
    }
    if (off.size() != expected.size())
      throw std::runtime_error("Missing point field");
    std::vector<dual::Point> result;
    result.reserve(m.width * m.height);
    for (size_t r = 0; r < m.height; ++r)
      for (size_t c = 0; c < m.width; ++c) {
        auto *p = m.data.data() + r * m.row_step + c * m.point_step;
        const float x = read<float>(p + off["x"]),
                    y = read<float>(p + off["y"]),
                    z = read<float>(p + off["z"]),
                    i = read<float>(p + off["intensity"]);
        const double ns = read<double>(p + off["timestamp"]);
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
            !std::isfinite(i) || !std::isfinite(ns) || i < 0 || i > 255)
          throw std::runtime_error("Invalid point value");
        const long double delta = static_cast<long double>(ns) - header;
        if (delta < -1000.L || delta > 200000000.L)
          throw std::runtime_error("Invalid absolute point timestamp");
        // Preserve epoch ns; only clamp sub-microsecond float64 quantization at
        // frame start.
        result.push_back({header + std::max<int64_t>(0, std::llround(delta)), x,
                          y, z, static_cast<uint8_t>(std::lround(i)),
                          p[off["tag"]], p[off["line"]]});
      }
    if (result.back().ns <= result.front().ns)
      throw std::runtime_error("No per-point time span");
    return result;
  }
  void cloud(Cloud::ConstSharedPtr m, int sensor) {
    if (failed)
      return;
    auto begin = Clock::now();
    try {
      if (!started) {
        started = true;
        last_output = begin;
      }
      auto output =
          fusion.push(sensor, rclcpp::Time(m->header.stamp).nanoseconds(),
                      parse(*m, rclcpp::Time(m->header.stamp).nanoseconds()));
      for (auto &w : output) {
        livox_ros_driver2::msg::CustomMsg msg;
        msg.header.stamp = rclcpp::Time(w.start);
        msg.header.frame_id = "lidar_front";
        msg.timebase = w.start;
        msg.lidar_id = 0;
        msg.points.reserve(w.points.size());
        for (auto &p : w.points) {
          livox_ros_driver2::msg::CustomPoint q;
          q.x = p.x;
          q.y = p.y;
          q.z = p.z;
          q.reflectivity = p.reflectivity;
          q.tag = p.tag;
          q.line = p.line;
          q.offset_time = p.ns - w.start;
          msg.points.push_back(q);
        }
        msg.point_num = msg.points.size();
        pub->publish(msg);
        ++windows;
        last_output = Clock::now();
        reason = "Dual lidar fusion active";
      }
      max_callback_ms = std::max(
          max_callback_ms,
          std::chrono::duration<double, std::milli>(Clock::now() - begin)
              .count());
      if (!output.empty())
        diagnostic();
    } catch (const std::exception &e) {
      fault(std::string(sensor ? "back: " : "front: ") + e.what());
    }
  }
  void imu(sensor_msgs::msg::Imu::ConstSharedPtr m) {
    if (failed)
      return;
    const auto ns = rclcpp::Time(m->header.stamp).nanoseconds();
    const auto &a = m->linear_acceleration;
    const auto &g = m->angular_velocity;
    if (ns <= 0 || ns <= last_imu || m->header.frame_id != "livox_frame" ||
        !std::isfinite(a.x) || !std::isfinite(a.y) || !std::isfinite(a.z) ||
        !std::isfinite(g.x) || !std::isfinite(g.y) || !std::isfinite(g.z)) {
      fault("Invalid/repeated front IMU");
      return;
    }
    last_imu = ns;
    auto out = *m;
    out.header.frame_id = "imu_front";
    out.linear_acceleration.x *= 9.80665;
    out.linear_acceleration.y *= 9.80665;
    out.linear_acceleration.z *= 9.80665;
    if (out.linear_acceleration_covariance[0] != -1)
      for (auto &v : out.linear_acceleration_covariance)
        v *= 9.80665 * 9.80665;
    imu_pub->publish(out);
  }

public:
  Adapter() : Node("towergo_dual_livox") {
    if (declare_parameter("coordinate_contract", std::string("unverified")) !=
        "both_raw_topics_already_in_front_lidar_frame")
      throw std::runtime_error(
          "Explicit verified coordinate contract required");
    if (declare_parameter("imu_acceleration_unit", std::string("unverified")) !=
        "g")
      throw std::runtime_error("Explicit IMU g units required");
    pub = create_publisher<livox_ros_driver2::msg::CustomMsg>(
        "/towergo/lio/lidar", 10);
    imu_pub = create_publisher<sensor_msgs::msg::Imu>("/towergo/lio/imu", 400);
    fault_pub = create_publisher<std_msgs::msg::Bool>(
        "/towergo/mapping_fault", rclcpp::QoS(1).transient_local());
    diag_pub = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", 10);
    auto lidar_q = rclcpp::SensorDataQoS().keep_last(10);
    auto imu_q = rclcpp::SensorDataQoS().keep_last(400);
    if (declare_parameter("reliable_input", false)) {
      lidar_q.reliable();
      imu_q.reliable();
    }
    const char *topics[] = {"/livox/lidar_front", "/livox/lidar_back"};
    for (int s = 0; s < 2; ++s)
      subs[s] = create_subscription<Cloud>(
          topics[s], lidar_q,
          [this, s](Cloud::ConstSharedPtr m) { cloud(m, s); });
    imu_sub = create_subscription<sensor_msgs::msg::Imu>(
        "/livox/imu_front", imu_q,
        [this](sensor_msgs::msg::Imu::ConstSharedPtr m) { imu(m); });
    timer = create_wall_timer(std::chrono::milliseconds(200), [this] {
      if (started && !failed &&
          std::chrono::duration<double>(Clock::now() - last_output).count() > 2)
        fault("No complete dual lidar window for 2 seconds");
      diagnostic();
    });
  }
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Adapter>());
  } catch (const std::exception &e) {
    std::fprintf(stderr, "%s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
