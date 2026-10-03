#include "a320/SimClock.h"

namespace a320 {

SimClock::SimClock(double stepS, double maxCatchUpS) : stepS_(stepS), maxCatchUpS_(maxCatchUpS) {}

int SimClock::advance(double realDtS) {
  if (paused_ || realDtS <= 0.0) return 0;
  accumulatorS_ += realDtS * rate_;
  // After a hitch (breakpoint, window drag) drop the backlog instead of spiralling.
  const double cap = maxCatchUpS_ * rate_;
  if (accumulatorS_ > cap) accumulatorS_ = cap;
  int steps = 0;
  while (accumulatorS_ >= stepS_) {
    accumulatorS_ -= stepS_;
    ++steps;
  }
  simTimeS_ += steps * stepS_;
  return steps;
}

void SimClock::setPaused(bool paused) {
  paused_ = paused;
  // Resuming must not replay the frames spent paused.
  accumulatorS_ = 0.0;
}

void SimClock::setRate(double rate) {
  rate_ = rate > 0.0 ? rate : 1.0;
}

}  // namespace a320
