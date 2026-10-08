#pragma once
#include <algorithm>
#include <array>
#include <cstdint>
#include <deque>
#include <stdexcept>
#include <vector>

namespace dual {
struct Point {
  int64_t ns;
  float x, y, z;
  uint8_t reflectivity, tag, line;
};
struct Window {
  int64_t start;
  std::vector<Point> points;
  std::array<uint64_t, 2> counts{};
};
// Pure bounded event-time fusion, independent of ROS and wall-clock scheduling.
class Fusion {
public:
  static constexpr int64_t width = 100000000, max_span = 1000000000;
  static constexpr size_t max_points = 300000;
  std::array<uint64_t, 2> input{}, emitted{}, discarded{}, gap_discarded{},
      expired{};
  uint64_t skipped_windows = 0;
  std::array<std::deque<Point>, 2> queue;
  std::array<int64_t, 2> first{{-1, -1}}, last{{-1, -1}}, headers{{-1, -1}};
  int64_t next = -1;
  bool failed = false;
  [[noreturn]] void fail(const char *why) {
    failed = true;
    throw std::runtime_error(why);
  }
  std::vector<Window> push(int sensor, int64_t header,
                           std::vector<Point> points) {
    if (failed)
      throw std::runtime_error("Fusion fault is latched");
    if (sensor < 0 || sensor > 1 || header <= 0 || header <= headers[sensor])
      fail("Repeated/backwards frame time");
    if (points.empty() || points.size() > 100000)
      fail("Invalid input point count");
    auto previous = last[sensor];
    for (auto &p : points) {
      if (p.ns < previous)
        fail("Late/backwards point time");
      if (p.line > 3)
        fail("Invalid MID360 line");
      previous = p.ns;
    }
    headers[sensor] = header;
    if (first[sensor] < 0)
      first[sensor] = points.front().ns;
    last[sensor] = points.back().ns;
    input[sensor] += points.size();
    for (auto &p : points) {
      p.line += sensor * 4;
      queue[sensor].push_back(p);
    }
    // Retention is a rolling window, not a sensor-fault threshold. Startup
    // bursts and forward clock gaps can make a small queue span many seconds.
    int64_t cutoff = -1;
    for (int side = 0; side < 2; ++side) {
      auto &q = queue[side];
      if (q.empty())
        continue;
      if (q.back().ns - q.front().ns > max_span)
        cutoff = std::max(cutoff, q.back().ns - max_span);
      if (q.size() > max_points)
        cutoff = std::max(cutoff, q[q.size() - max_points].ns + 1);
    }
    if (cutoff >= 0) {
      const auto retain_from = ((cutoff + width - 1) / width) * width;
      if (next >= 0 && retain_from > next)
        skipped_windows += (retain_from - next) / width;
      next = std::max(next, retain_from);
      for (int side = 0; side < 2; ++side) {
        auto &q = queue[side];
        while (!q.empty() && q.front().ns < next) {
          q.pop_front();
          ++expired[side];
        }
      }
    }
    std::vector<Window> out;
    if (first[0] < 0 || first[1] < 0)
      return out;
    if (next < 0) {
      const auto start = std::max(first[0], first[1]);
      next = ((start + width - 1) / width) * width;
    }
    // A short startup packet may end before the first complete window.
    // Discard its leading samples on subsequent arrivals as well.
    for (int s = 0; s < 2; ++s)
      while (!queue[s].empty() && queue[s].front().ns < next) {
        queue[s].pop_front();
        ++discarded[s];
      }
    while (std::min(last[0], last[1]) >= next + width) {
      Window w;
      w.start = next;
      for (int s = 0; s < 2; ++s) {
        while (!queue[s].empty() && queue[s].front().ns < next + width) {
          w.points.push_back(queue[s].front());
          queue[s].pop_front();
          ++w.counts[s];
        }
      }
      next += width;
      if (!w.counts[0] || !w.counts[1]) {
        // Missing DDS/startup frames do not make the next complete window
        // unusable. Never publish a one-sided window or invent its points.
        ++skipped_windows;
        for (int s = 0; s < 2; ++s)
          gap_discarded[s] += w.counts[s];
        continue;
      }
      for (int s = 0; s < 2; ++s)
        emitted[s] += w.counts[s];
      std::stable_sort(
          w.points.begin(), w.points.end(),
          [](const Point &a, const Point &b) { return a.ns < b.ns; });
      out.push_back(std::move(w));
    }
    return out;
  }
};
} // namespace dual
