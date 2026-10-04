#pragma once

#include <cstdint>

#include "a320/a320_api.h"

namespace a320 {

// One step of a lesson. The step is done when done() is true (checked every update); a manual
// step is a check item the pilot confirms with NEXT.
struct GuideStep {
  const char* phase;
  const char* title;
  const char* action;
  const char* look;
  const char* why;
  const char* msfs;
  A320GuideTarget targets[A320_GUIDE_MAX_TARGETS];
  bool manual;
  bool (*done)(const A320State& s, const A320Controls& c);
};

struct GuideDef {
  const char* name;
  const char* summary;
  A320Scenario scenario;
  const GuideStep* steps;
  int stepCount;
  // What needs attention right now at this step, or nullptr.
  const char* (*alert)(const A320State& s, const A320Controls& c, int step);
};

int guideCount();
const GuideDef* guideDef(int guide);

class GuideRunner {
 public:
  void start(int guide);
  void stop() { guide_ = -1; }
  void next();
  void back();
  // Ticks off the current step, and any following ones already done, from the live cockpit.
  void update(const A320State& s, const A320Controls& c);
  A320GuideStatus status() const;
  const char* alert() const { return alert_; }

 private:
  int guide_ = -1;
  int step_ = 0;
  uint32_t done_ = 0;
  const char* alert_ = "";
};

}  // namespace a320
