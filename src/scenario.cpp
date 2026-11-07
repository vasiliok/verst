/// @file
/// Vrst_scenario: parsing and playing scripted map motion.
/// Declarations and documentation are in scenario.h.
#include "scenario.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>

#include "gpx_track.h"

namespace nav {

namespace {

/// Splits a line into words; drops a `#` comment.
/// @param line One scenario line.
/// @return Words.
std::vector<std::string_view> words(std::string_view line) {
  if (const Vrst_size hash = line.find('#'); hash != std::string_view::npos) line = line.substr(0, hash);
  std::vector<std::string_view> out;
  Vrst_size i = 0;
  while (i < line.size()) {
    while (i < line.size() && std::isspace(static_cast<unsigned char>(line[i]))) ++i;
    const Vrst_size b = i;
    while (i < line.size() && !std::isspace(static_cast<unsigned char>(line[i]))) ++i;
    if (i > b) out.push_back(line.substr(b, i - b));
  }
  return out;
}

}  // namespace

Vrst_scenario Vrst_scenario::parse(std::string name, std::string_view text) {
  Vrst_scenario s;
  s.m_name = std::move(name);
  Vrst_size line_no = 0;
  while (!text.empty()) {
    const Vrst_size nl = text.find('\n');
    const std::string_view line = text.substr(0, nl);
    text = nl == std::string_view::npos ? std::string_view{} : text.substr(nl + 1);
    ++line_no;
    const std::vector<std::string_view> w = words(line);
    if (w.empty()) continue;
    const auto bad = [&](std::string_view why) {
      return std::runtime_error("scenario " + s.m_name + " line " + std::to_string(line_no) + ": " +
                                std::string(why) + ": " + std::string(line));
    };
    // Number argument i (1-based after the command).
    const auto arg = [&](Vrst_size i) {
      if (i >= w.size()) throw bad("missing value");
      const std::optional<Vrst_float64> v = parse_float64(w[i]);
      if (!v) throw bad("not a number");
      return *v;
    };
    const auto args = [&](Vrst_size n) {
      if (w.size() != n + 1) throw bad("expects " + std::to_string(n) + " value(s)");
    };
    Vrst_scenario_step st;
    st.text = std::string(line.substr(0, line.find('#')));
    const std::string_view cmd = w[0];
    if (cmd == "wait") {
      args(1);
      st.seconds = arg(1);
    } else if (cmd == "wait_tiles") {
      args(1);
      st.kind = Vrst_step_kind::WaitTiles;
      st.seconds = arg(1);
    } else if (cmd == "pan" || cmd == "fling") {
      args(3);
      st.kind = cmd == "pan" ? Vrst_step_kind::Pan : Vrst_step_kind::Fling;
      st.pan = {arg(1), arg(2)};
      st.seconds = arg(3);
    } else if (cmd == "zoom") {
      args(2);
      st.kind = Vrst_step_kind::Zoom;
      st.factor = arg(1);
      st.seconds = arg(2);
      if (st.factor <= 0) throw bad("factor must be > 0");
    } else if (cmd == "zoom_in" || cmd == "zoom_out") {
      args(0);
      st.kind = Vrst_step_kind::Zoom;
      st.factor = cmd == "zoom_in" ? 2.0 : 0.5;
    } else if (cmd == "rotate") {
      args(2);
      st.kind = Vrst_step_kind::Rotate;
      st.degrees = arg(1);
      st.seconds = arg(2);
    } else if (cmd == "goto") {
      args(3);
      st.kind = Vrst_step_kind::Goto;
      const Vrst_float64 lat = arg(1), lon = arg(2);
      if (std::abs(lat) > 90 || std::abs(lon) > 180) throw bad("lat/lon out of range");
      st.at = {static_cast<Vrst_int32>(std::lround(lat * 1e7)), static_cast<Vrst_int32>(std::lround(lon * 1e7))};
      st.mpp = arg(3);
      if (st.mpp <= 0) throw bad("m/pt must be > 0");
    } else {
      throw bad("unknown step");
    }
    if (st.seconds < 0) throw bad("seconds must be >= 0");
    s.m_steps.push_back(std::move(st));
  }
  if (s.m_steps.empty()) throw std::runtime_error("scenario " + s.m_name + " has no steps");
  return s;
}

void Vrst_scenario::start(Vrst_float64 now) {
  m_index = 0;
  m_step_start = now;
  m_done = 0;
  m_running = true;
}

Vrst_float64 Vrst_scenario::max_seconds() const {
  Vrst_float64 sum = 0;
  for (const Vrst_scenario_step& s : m_steps) sum += s.seconds;
  return sum;
}

Vrst_float64 Vrst_scenario::ease(const Vrst_scenario_step& s, Vrst_float64 x) {
  x = std::clamp(x, 0.0, 1.0);
  if (s.kind == Vrst_step_kind::Fling) return 1 - (1 - x) * (1 - x) * (1 - x);  // fast, then slowing down
  return x * x * (3 - 2 * x);                                                // smooth start and stop
}

void Vrst_scenario::apply(const Vrst_scenario_step& s, Vrst_float64 from, Vrst_float64 to, Vrst_view& v,
                          const Vrst_scenario_frame& f) {
  const Vrst_float64 part = to - from;
  if (part <= 0) return;
  const Vrst_float64 cx = f.fb_w / 2.0, cy = f.fb_h / 2.0;
  switch (s.kind) {
    case Vrst_step_kind::Pan:
    case Vrst_step_kind::Fling:
      pan_by(v, s.pan.x * part * f.px_per_point, s.pan.y * part * f.px_per_point);
      break;
    case Vrst_step_kind::Zoom:
      zoom_at(v, f.fb_w, f.fb_h, cx, cy, std::pow(s.factor, part), f.min_scale, f.max_scale);
      break;
    case Vrst_step_kind::Rotate:
      rotate_at(v, f.fb_w, f.fb_h, cx, cy, s.degrees * part * kPi / 180);
      break;
    case Vrst_step_kind::Wait:
    case Vrst_step_kind::WaitTiles:
    case Vrst_step_kind::Goto:
      break;
  }
}

Vrst_scenario_tick Vrst_scenario::advance(Vrst_float64 now, Vrst_view& v, const Vrst_scenario_frame& f) {
  Vrst_scenario_tick tick;
  while (m_running && m_index < m_steps.size()) {
    const Vrst_scenario_step& s = m_steps[m_index];
    const Vrst_float64 elapsed = now - m_step_start;
    bool done = elapsed >= s.seconds;
    if (s.kind == Vrst_step_kind::WaitTiles) done = done || f.tiles_ready;
    if (s.kind == Vrst_step_kind::Goto && m_done == 0) {
      tick.jump = s;
      tick.moved = true;
      m_done = 1;
    }
    const Vrst_float64 to = done ? 1.0 : ease(s, s.seconds > 0 ? elapsed / s.seconds : 1.0);
    if (to > m_done) {
      apply(s, m_done, to, v, f);
      tick.moved = tick.moved || s.kind == Vrst_step_kind::Pan || s.kind == Vrst_step_kind::Fling ||
                   s.kind == Vrst_step_kind::Zoom || s.kind == Vrst_step_kind::Rotate;
      m_done = to;
    }
    if (!done) break;
    // Next step starts where this one ended, so the timeline does not drift; a tile wait
    // ends when the tiles are ready, which is now.
    m_step_start = s.kind == Vrst_step_kind::WaitTiles ? now : m_step_start + s.seconds;
    m_done = 0;
    ++m_index;
    tick.step_changed = true;
    if (s.kind == Vrst_step_kind::Goto) break;  // the front end applies the jump before more motion
  }
  if (m_running && m_index >= m_steps.size()) {
    m_running = false;
    tick.finished = true;
  }
  return tick;
}

}  // namespace nav
