/*
 * capture_frames — grabs N raw frames from the camera and saves them as PNG.
 * Run while startracker service is stopped (camera must be free).
 *
 * Usage: capture_frames [--n <count>] [--out <dir>] [--config <path>]
 *   default: 10 frames → /tmp/frames/
 */

#include "common/Config.hpp"
#include "io/CameraReader.hpp"
#include "util/ConfigLoader.hpp"

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <string>
#include <thread>

#include <opencv2/imgcodecs.hpp>

namespace fs = std::filesystem;

int main(int argc, char** argv) {
    int n_frames = 10;
    std::string out_dir = "/tmp/frames";
    std::string cfg_path;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if      (a == "--n"      && i+1 < argc) n_frames = std::stoi(argv[++i]);
        else if (a == "--out"    && i+1 < argc) out_dir  = argv[++i];
        else if (a == "--config" && i+1 < argc) cfg_path = argv[++i];
    }

    AppConfig cfg;
    if (cfg_path.empty()) {
        std::string bin_dir = fs::path(argv[0]).parent_path().string();
        cfg_path = bin_dir + "/config.json";
    }
    if (util::loadConfig(cfg_path, cfg))
        std::printf("Config: %s\n", cfg_path.c_str());

    fs::create_directories(out_dir);

    CameraReader cam(cfg.camera);
    std::printf("Camera opened, warming up...\n");
    std::this_thread::sleep_for(std::chrono::milliseconds{1000});

    // Discard first 5 frames (camera auto-exposure settling)
    for (int i = 0; i < 5; ++i) {
        auto f = cam.nextFrame(std::chrono::milliseconds{3000});
        if (f.empty()) { std::fprintf(stderr, "Camera timeout during warmup\n"); return 1; }
    }

    std::printf("Capturing %d frames to %s/\n", n_frames, out_dir.c_str());
    int saved = 0;
    for (int i = 0; i < n_frames; ++i) {
        auto frame = cam.nextFrame(std::chrono::milliseconds{3000});
        if (frame.empty()) { std::fprintf(stderr, "Camera timeout at frame %d\n", i); break; }
        char name[256];
        std::snprintf(name, sizeof(name), "%s/frame_%03d.png", out_dir.c_str(), i);
        cv::imwrite(name, frame);
        std::printf("  Saved %s\n", name);
        ++saved;
    }
    std::printf("Done: %d/%d frames saved.\n", saved, n_frames);
    return 0;
}
