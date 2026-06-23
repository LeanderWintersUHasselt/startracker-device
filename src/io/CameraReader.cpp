#include "io/CameraReader.hpp"
#include <cstdio>
#include <sstream>
#include <stdexcept>
#include <unistd.h>

// ── rpicam-vid commandoopbouw ─────────────────────────────────────────────────

static std::string buildCmd(const CameraConfig& c) {
    std::ostringstream cmd;
    cmd << "rpicam-vid"
        << " --camera "    << c.camera
        << " --width "     << c.width
        << " --height "    << c.height
        << " --framerate " << c.fps
        << " --codec yuv420"
        << " --buffer-count 1"
        << " --nopreview"
        << " --timeout 0"
        << " -o -";
    if (c.shutter > 0)
        cmd << " --shutter " << c.shutter;
    if (c.gain > 0.f)
        cmd << " --gain " << c.gain;
    if (!std::isnan(c.lensPosition) && c.lensPosition > 0.f)
        cmd << " --autofocus-mode manual --lens-position " << c.lensPosition;
    if (c.awbGainR > 0.f && c.awbGainB > 0.f)
        cmd << " --awbgains " << c.awbGainR << "," << c.awbGainB;
    cmd << " 2>/dev/null";
    return cmd.str();
}

// ── Constructie / destructie ──────────────────────────────────────────────────

CameraReader::CameraReader(CameraConfig cfg) : cfg_(std::move(cfg)) {
    std::string cmd = buildCmd(cfg_);
    pipe_ = popen(cmd.c_str(), "r");
    if (!pipe_)
        throw std::runtime_error("CameraReader: popen mislukt: " + cmd);
    pipe_fd_ = fileno(pipe_);

    thread_ = std::thread(&CameraReader::readerLoop, this);
}

CameraReader::~CameraReader() {
    stop_.store(true, std::memory_order_release);
    cv_.notify_all();

    // Close the fd to interrupt any blocking read() inside the reader thread.
    // After close, the next read() returns EBADF (or the in-progress one returns
    // a short count / EIO), causing readYuv420Frame() to return an empty Mat,
    // readerLoop() to exit, and thread_.join() to return quickly.
    // We accept that pclose() is skipped: rpicam-vid receives SIGPIPE when it
    // next writes to stdout and exits on its own.
    if (pipe_fd_ >= 0) {
        ::close(pipe_fd_);
        pipe_fd_ = -1;
    }

    if (thread_.joinable()) thread_.join();
    pipe_ = nullptr;  // already closed via fd above
}

// ── Publieke API ──────────────────────────────────────────────────────────────

cv::Mat CameraReader::nextFrame(std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lk(mutex_);
    bool ok = cv_.wait_for(lk, timeout, [this]{ return !queue_.empty() || stop_.load(); });
    if (!ok || queue_.empty()) return {};
    cv::Mat frame = std::move(queue_.front());
    queue_.pop_front();
    return frame;
}

// ── Reader-thread ─────────────────────────────────────────────────────────────

void CameraReader::readerLoop() {
    while (!stop_.load()) {
        cv::Mat frame = readYuv420Frame();
        if (frame.empty()) break;   // pipe gesloten of fout

        std::unique_lock<std::mutex> lk(mutex_);
        if (static_cast<int>(queue_.size()) >= kMaxQueue)
            queue_.pop_front();     // gooi het oudste frame weg
        queue_.push_back(std::move(frame));
        lk.unlock();
        cv_.notify_one();
    }
    stop_.store(true);
    cv_.notify_all();
}

// ── YUV420 planar → BGR ───────────────────────────────────────────────────────

cv::Mat CameraReader::readYuv420Frame() {
    // YUV420 planar: Y-vlak (W×H) + U-vlak (W/2 × H/2) + V-vlak (W/2 × H/2)
    int W = cfg_.width, H = cfg_.height;
    int frameBytes = W * H * 3 / 2;

    static thread_local std::vector<uint8_t> buf;
    buf.resize(frameBytes);

    size_t read = fread(buf.data(), 1, frameBytes, pipe_);
    if (static_cast<int>(read) != frameBytes) return {};

    // Wikkel in een OpenCV Mat en converteer naar BGR
    cv::Mat yuv(H + H / 2, W, CV_8UC1, buf.data());
    cv::Mat bgr;
    cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR_I420);
    return bgr;
}
