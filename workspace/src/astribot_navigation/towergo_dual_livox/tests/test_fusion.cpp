#include "fusion.hpp"
#include <cassert>
#include <functional>
#include <iostream>
using namespace dual;
std::vector<Point> points(int64_t a, int64_t b, int64_t step = 10000000) {
  std::vector<Point> v;
  for (auto t = a; t <= b; t += step)
    v.push_back({t, 1, 2, 3, 1, 0, 0});
  return v;
}
void throws(std::function<void()> f) {
  bool caught = false;
  try {
    f();
  } catch (const std::runtime_error &) {
    caught = true;
  }
  assert(caught);
}
int main() {
  {
    Fusion f;
    auto a = f.push(0, 100000000, points(100000000, 200000000));
    assert(a.empty());
    auto out = f.push(1, 100000000, points(100000000, 200000000));
    assert(out.size() == 1);
    auto &w = out[0];
    assert(w.start == 100000000 && w.counts[0] == 10 && w.counts[1] == 10);
    int64_t prev = 0;
    int back = 0;
    for (auto &p : w.points) {
      assert(p.ns >= prev && p.ns < w.start + Fusion::width);
      prev = p.ns;
      back += p.line == 4;
    }
    assert(back == 10);
    assert(f.queue[0].front().ns == 200000000);
    assert(f.push(1, 210000000, points(210000000, 300000000)).empty());
    out = f.push(0, 210000000, points(210000000, 300000000));
    assert(out.size() == 1 && out[0].counts[0] == 10 && out[0].counts[1] == 10);
    for (int s = 0; s < 2; ++s)
      assert(f.input[s] == f.emitted[s] + f.discarded[s] + f.queue[s].size());
  }
  {
    Fusion f;
    f.push(0, 101000000, points(101000000, 211000000));
    auto out = f.push(1, 102000000, points(102000000, 212000000));
    assert(out.empty());
    assert(f.next == 200000000);
    assert(f.discarded[0] == 10 && f.discarded[1] == 10);
    f.push(0, 221000000, points(221000000, 311000000));
    out = f.push(1, 222000000, points(222000000, 312000000));
    assert(out.size() == 1 && out[0].counts[0] == 10 && out[0].counts[1] == 10);
  }
  {
    Fusion f;
    f.push(0, 1, points(1, 100000001));
    throws([&] { f.push(0, 1, points(100000002, 200000002)); });
    throws([&] { f.push(1, 1, points(1, 100000001)); });
  }
  {
    Fusion f;
    throws([&] {
      f.push(0, 1, {{10, 0, 0, 0, 0, 0, 0}, {9, 0, 0, 0, 0, 0, 0}});
    });
  }
  {
    Fusion f;
    f.push(0, 100000000, points(100000000, 200000000));
    f.push(1, 100000000, points(100000000, 200000000));
    throws([&] { f.push(0, 200000001, points(190000000, 290000000)); });
  }
  {
    Fusion f;
    f.push(0, 1, points(1, 1100000001));
    assert(!f.failed && f.expired[0] > 0);
    assert(f.queue[0].back().ns - f.queue[0].front().ns <= Fusion::max_span);
  }
  {
    Fusion f;
    auto p = points(1, 100000001);
    p[0].line = 4;
    throws([&] { f.push(0, 1, p); });
  }
  {
    Fusion f;
    throws([&] { f.push(0, 1, std::vector<Point>(100001)); });
  }
  {
    // A dropped frame must not discard already completed output in this batch.
    Fusion f;
    f.push(0, 100000000, points(100000000, 200000000));
    f.push(1, 100000000, points(100000000, 200000000));
    f.push(0, 210000000, points(210000000, 500000000));
    auto out = f.push(1, 410000000, points(410000000, 500000000));
    assert(out.size() == 2 && out[0].start == 200000000 &&
           out[1].start == 400000000);
    assert(f.skipped_windows == 1 && f.gap_discarded[0] == 10 &&
           f.gap_discarded[1] == 0);
    for (int side = 0; side < 2; ++side)
      assert(f.input[side] == f.emitted[side] + f.discarded[side] +
                                  f.gap_discarded[side] + f.queue[side].size());
  }
  {
    Fusion f;
    f.push(0, 110000000, points(110000000, 120000000));
    f.push(1, 110000000, points(110000000, 120000000));
    assert(f.push(0, 130000000, points(130000000, 140000000)).empty());
    assert(f.push(1, 130000000, points(130000000, 140000000)).empty());
    f.push(0, 150000000, points(150000000, 300000000));
    auto out = f.push(1, 150000000, points(150000000, 300000000));
    assert(out.size() == 1 && out[0].start == 200000000 && !f.failed);
  }
  {
    Fusion f;
    f.push(0, 100000000, points(100000000, 200000000));
    f.push(1, 100000000, points(100000000, 200000000));
    for (int64_t t = 210000000; t < 1100000000; t += 100000000)
      assert(f.push(0, t, points(t, t + 90000000)).empty());
    f.push(0, 1110000000, points(1110000000, 1210000000));
    assert(!f.failed && f.expired[0] > 0);
  }
  {
    Fusion f;
    f.push(0, 1000000000, points(1000000000, 1100000000));
    f.push(1, 1000000000, points(1000000000, 1100000000));
    assert(f.push(0, 3000000000, points(3000000000, 3100000000)).empty());
    auto out = f.push(1, 3000000000, points(3000000000, 3100000000));
    assert(out.size() == 1 && out[0].start == 3000000000 && !f.failed);
    for (int side = 0; side < 2; ++side)
      assert(f.input[side] == f.emitted[side] + f.discarded[side] +
                                  f.gap_discarded[side] + f.expired[side] +
                                  f.queue[side].size());
  }
  {
    Fusion f;
    for (int frame = 0; frame < 10; ++frame) {
      const int64_t start = 1000000000 + frame * 100000;
      auto p = points(start, start + 99999, 1);
      f.push(0, start, p);
      assert(f.queue[0].size() <= Fusion::max_points && !f.failed);
    }
    assert(f.expired[0] > 0);
  }
  std::cout << "13 fusion scenarios passed\n";
}
