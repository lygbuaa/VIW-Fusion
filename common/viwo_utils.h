#ifndef VIWO_UTILS_H
#define VIWO_UTILS_H

#include <chrono>
#include <memory>
#include <ros/ros.h>


#define HANG_STOPWATCH() auto _ViwoUtilsPtr_ = ViwoUtils::HangStopWatch(__FUNCTION__);

class ViwoUtils
{
public:
    ViwoUtils() {}
    ~ViwoUtils() {}

    static inline __attribute__((always_inline)) uint64_t CurrentMicros() {
        return std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::time_point_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now()).time_since_epoch()).count();
    }

    /*print function enter/exit/time_interval*/
    static std::shared_ptr<uint64_t> HangStopWatch(const char* func_name){
        uint64_t* pts_us = new uint64_t;
        *pts_us = CurrentMicros();
        ROS_INFO("stop watch trigger by %s (epoch %ld us)", func_name, *pts_us);
        return std::shared_ptr<uint64_t>(pts_us, [func_name](uint64_t* ptr){
            uint64_t ts_us = CurrentMicros();
            // LOG(INFO) << "stop watch end: " << ts_us << " us.";
            ROS_INFO("stop watch end by %s (elapse = %ld us)", func_name, (ts_us - *ptr));
            delete ptr;
        });
    }

};

#endif