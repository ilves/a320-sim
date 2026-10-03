#pragma once

namespace a320 {

// Fixed-step simulation clock decoupled from the render frame rate. Pausing stops sim
// time while rendering and the cockpit keep running.
class SimClock {
 public:
  explicit SimClock(double stepS = 1.0 / 120.0, double maxCatchUpS = 0.25);

  // Returns how many fixed steps to run for this frame.
  int advance(double realDtS);

  void setPaused(bool paused);
  bool paused() const { return paused_; }
  void setRate(double rate);
  double rate() const { return rate_; }
  double stepS() const { return stepS_; }
  double simTimeS() const { return simTimeS_; }
  void resetTime() { simTimeS_ = 0.0; accumulatorS_ = 0.0; }

 private:
  double stepS_;
  double maxCatchUpS_;
  double accumulatorS_ = 0.0;
  double simTimeS_ = 0.0;
  double rate_ = 1.0;
  bool paused_ = false;
};

}  // namespace a320
