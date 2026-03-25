//
// Created by guoqian on 23-10-1.
//
#include "tools/time_recorder.h"
#include "tools/tools.hpp"

namespace PathOptimizationNS {

void TimeRecorder::recordTime(const std::string &name) {
    time_stamps_.emplace_back(std::clock());
    names_.emplace_back(name);
}

void TimeRecorder::printTime() const {
    if (time_stamps_.size() <= 1) {
        std::cout << "time stamps size not enough!";
        return;
    }
    std::cout << "========Printing time for " << title_ << "========";
    for (size_t i = 0; i < time_stamps_.size() - 1; ++i) {
        std::cout << names_[i] << " cost " << time_ms(time_stamps_[i], time_stamps_[i + 1]) << " ms.";
    }
    if (time_stamps_.size() > 2) {
        std::cout << "Total time cost: " << time_ms(time_stamps_.front(), time_stamps_.back()) << " ms.";
    }
    std::cout << "========End printing time for " << title_ << "========";
}

void TimeRecorder::clear() {
    time_stamps_.clear();
    names_.clear();
}

}

