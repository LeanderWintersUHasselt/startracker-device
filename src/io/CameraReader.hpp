#pragma once
#include "common/Types.hpp"
#include "common/Config.hpp"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>

// Leest YUV420-frames van een rpicam-vid subprocess in een eigen thread.
// Bounded queue van 2 frames: houdt de verwerking altijd actueel.
//
// RAII: destructor sluit de pipe en stopt de thread netjes.

class CameraReader {
public:
    explicit CameraReader(CameraConfig cfg);
    ~CameraReader();

    // Blokkeert tot een frame beschikbaar is of de timeout verstrijkt.
    // Leeg Mat = timeout of cameraprobleem.
    cv::Mat nextFrame(std::chrono::milliseconds timeout
                      = std::chrono::milliseconds{3000});

    bool isRunning() const { return !stop_.load(std::memory_order_acquire); }
    bool isStopped() const { return  stop_.load(std::memory_order_acquire); }

private:
    void   readerLoop();
    cv::Mat readYuv420Frame();          // leest precies één frame van pipe_

    CameraConfig cfg_;
    FILE*        pipe_    = nullptr;
    int          pipe_fd_ = -1;         // fd van pipe_ voor interrupt in destructor

    std::thread             thread_;
    std::mutex              mutex_;
    std::condition_variable cv_;
    std::deque<cv::Mat>     queue_;
    std::atomic<bool>       stop_{false};

    static constexpr int kMaxQueue = 1;
};
