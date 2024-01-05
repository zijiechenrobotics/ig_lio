#include "ig_lio/timer.h"

/// print the run time
void Timer::PrintAll() {
  LOG(INFO) << ">>> ===== Printing run time =====";
  for (const auto& r : records_) {
    double time_temp = std::accumulate(r.second.time_usage_in_ms_.begin(),
                                       r.second.time_usage_in_ms_.end(),
                                       0.0) /
                       double(r.second.time_usage_in_ms_.size());
    LOG(INFO) << "> [ " << r.first << " ] average time usage: " << time_temp
              << " ms , called times: " << r.second.time_usage_in_ms_.size();
  }
}