/*
 * debug_detect — live detectie-visualisatie
 *
 * Toetsen via SSH-terminal (stdin):
 *   a / A   area_min   -5 / +5
 *   x / X   area_max  -25 /+25
 *   r / R   circ_min  -0.05/+0.05
 *   t / T   threshold -10 /+10  (0 = Otsu auto)
 *   d / D   downsample -1 / +1
 *   b / B   blur_kernel -2 / +2
 *   s       snapshot → /tmp/snap.jpg
 *   f       timestamped snapshot → /tmp/debug_raw_<ts>.png + /tmp/debug_grid_<ts>.png
 *   c       sla params op in config.json
 *   q       stop
 */

#include "common/Config.hpp"
#include <filesystem>
#include "detector/StarDetector.hpp"
#include "detector/StarDetectorLight.hpp"
#include "io/CameraReader.hpp"
#include "util/ConfigLoader.hpp"
#include "util/Intrinsics.hpp"

#include <chrono>
#include <cstdio>
#include <ctime>
#include <opencv2/highgui.hpp>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>

using clk = std::chrono::steady_clock;

// ── Niet-blokkerende stdin-lezer (zelfde als main.cpp) ────────────────────────
struct RawTerminal {
    RawTerminal() {
        tcgetattr(STDIN_FILENO, &old_);
        termios raw = old_;
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VMIN]  = 0;   // niet-blokkerend
        raw.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    }
    ~RawTerminal() { tcsetattr(STDIN_FILENO, TCSADRAIN, &old_); }
    char read() const {
        char c = '\0';
        ::read(STDIN_FILENO, &c, 1);
        return c;
    }
    termios old_;
};

int main(int argc, char** argv) {
    namespace fs = std::filesystem;
    AppConfig cfg;
    std::string cfgPath;
    {
        std::string binDir = fs::path(argv[0]).parent_path().string();
        cfgPath = binDir + "/config.json";
        if (!fs::exists(cfgPath)) cfgPath = "config.json";
    }
    if (util::loadConfig(cfgPath, cfg))
        std::printf("Config: %s\n", cfgPath.c_str());

    bool useLight = false;  // standaard: zware detector
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if      (a == "--light")      useLight = true;
        else if (a == "--heavy")      useLight = false;
        else if (a == "--intrinsics" && i+1 < argc) cfg.intrinsics.activePath = argv[++i];
        else if (a == "--config"    && i+1 < argc) { cfgPath = argv[++i]; util::loadConfig(cfgPath, cfg); }
    }
    if (cfg.intrinsics.activePath.empty()) {
        std::fprintf(stderr,
            "[FOUT] Geen intrinsics. Zet \"intrinsics\" in config.json "
            "of gebruik --intrinsics <pad>\n");
        return 1;
    }

    Intrinsics intr;
    try { intr = util::loadIntrinsics(cfg.intrinsics.activePath); }
    catch (const std::exception& e) { std::fprintf(stderr, "%s\n", e.what()); return 1; }

    CameraReader cam(cfg.camera);
    std::this_thread::sleep_for(std::chrono::milliseconds{800});

    StarDetector      heavy(cfg.detector);
    StarDetectorLight light(cfg.detectorLight);

    const std::string WIN = std::string("debug_detect [") + (useLight ? "LIGHT" : "HEAVY") + "]";
    cv::namedWindow(WIN, cv::WINDOW_NORMAL);
    cv::resizeWindow(WIN, 1920, 540);

    std::printf("Modus: %s  (herstart met --light of --heavy om te wisselen)\n",
                useLight ? "LIGHT" : "HEAVY");
    std::printf("Toetsen (Light): a/A=area_min  x/X=area_max  r/R=circ  t/T=thr  d/D=ds  b/B=blur\n");
    std::printf("Toetsen (Heavy): v/V=val_min  p/P=peak_floor  g/G=bg_kernel\n");
    std::printf("s=snapshot  f=timestamped snapshot  c=opslaan  q=stop\n\n");

    RawTerminal term;  // stdin in raw mode
    double fpsSmooth = 0.0;
    auto tLast = clk::now();

    while (true) {
        // ── Toets van stdin (SSH-terminal) ────────────────────────────────────
        char key = term.read();
        if (key == 'q' || key == 'Q') break;

        bool changed = true;
        if (useLight) {
            auto& c = light.config();
            switch (key) {
                case 'a': c.areaMin    = std::max(1,    c.areaMin -  5);          break;
                case 'A': c.areaMin   += 5;                                        break;
                case 'x': c.areaMax    = std::max(c.areaMin+1, c.areaMax - 25);   break;
                case 'X': c.areaMax   += 25;                                       break;
                case 'r': c.circMin    = std::max(0.f,  c.circMin - 0.05f);       break;
                case 'R': c.circMin    = std::min(1.f,  c.circMin + 0.05f);       break;
                case 't': c.threshold  = std::max(0,    c.threshold - 10);        break;
                case 'T': c.threshold  = std::min(254,  c.threshold + 10);        break;
                case 'd': c.downsample = std::max(1,    c.downsample - 1);        break;
                case 'D': c.downsample = std::min(8,    c.downsample + 1);        break;
                case 'b': c.blurKernel = std::max(0,   (c.blurKernel-2)|1);       break;
                case 'B': c.blurKernel = (c.blurKernel+2)|1;                      break;
                case 's': changed = false;                                         break;
                case 'f': changed = false;                                         break;
                case 'c':
                    cfg.detectorLight = c;
                    util::saveConfig(cfgPath, cfg);
                    std::printf("\rOpgeslagen: %s\n", cfgPath.c_str());
                    changed = false; break;
                default: changed = false; break;
            }
        } else {
            auto& c = heavy.config();
            switch (key) {
                case 'v': c.valMin    = std::max(0,    c.valMin    - 10);         break;
                case 'V': c.valMin    = std::min(255,  c.valMin    + 10);         break;
                case 'p': c.peakFloor = std::max(0,    c.peakFloor -  5);         break;
                case 'P': c.peakFloor = std::min(255,  c.peakFloor +  5);         break;
                case 'g': c.bgKernel  = std::max(3,   ((c.bgKernel-2)|1));        break;
                case 'G': c.bgKernel  = (c.bgKernel+2)|1;                         break;
                case 'a': c.areaMin   = std::max(1,    c.areaMin   - 10);         break;
                case 'A': c.areaMin  += 10;                                        break;
                case 'x': c.areaMax   = std::max(c.areaMin+1, c.areaMax - 50);    break;
                case 'X': c.areaMax  += 50;                                        break;
                case 'r': c.circMin   = std::max(0.f,  c.circMin   - 0.05f);      break;
                case 'R': c.circMin   = std::min(1.f,  c.circMin   + 0.05f);      break;
                case 's': changed = false;                                          break;
                case 'f': changed = false;                                          break;
                case 'c':
                    cfg.detector = c;
                    util::saveConfig(cfgPath, cfg);
                    std::printf("\rOpgeslagen: %s\n", cfgPath.c_str());
                    changed = false; break;
                default: changed = false; break;
            }
        }

        // ── Frame ophalen + detectie ──────────────────────────────────────────
        cv::Mat bgr = cam.nextFrame(std::chrono::milliseconds{3000});
        if (bgr.empty()) { std::printf("\r[FOUT] camera timeout\n"); break; }

        auto tNow = clk::now();
        double dt = std::chrono::duration<double, std::milli>(tNow - tLast).count();
        tLast = tNow;
        fpsSmooth = fpsSmooth == 0 ? 1000/dt : 0.85*fpsSmooth + 0.15*1000/dt;

        auto t0 = clk::now();
        std::vector<cv::Point2f> dets;
        cv::Mat grid;

        if (useLight) {
            dets = light.detect(bgr, intr);
            grid = light.makeDebugGrid(bgr, dets, 960, 540);
        } else {
            dets = heavy.detect(bgr, intr);
            grid = heavy.makeDebugGrid(bgr, dets, 960, 540);
        }
        double ms = std::chrono::duration<double, std::milli>(clk::now()-t0).count();

        char info[256];
        if (useLight) {
            auto& c = light.config();
            // Otsu-waarde voor referentie
            cv::Mat gs; cv::cvtColor(bgr, gs, cv::COLOR_BGR2GRAY);
            if (c.downsample > 1) cv::resize(gs, gs, {bgr.cols/c.downsample, bgr.rows/c.downsample}, 0, 0, cv::INTER_AREA);
            cv::Mat tmp; double otsuVal = cv::threshold(gs, tmp, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
            std::snprintf(info, sizeof(info),
                "[LIGHT] fps=%.0f  %.1fms  n=%d | thr=%s(otsu=%.0f)  ds=%d  area=[%d..%d]  circ=%.2f",
                fpsSmooth, ms, (int)dets.size(),
                c.threshold==0 ? "AUTO" : std::to_string(c.threshold).c_str(),
                otsuVal, c.downsample, c.areaMin, c.areaMax, c.circMin);
        } else {
            auto& c = heavy.config();
            std::snprintf(info, sizeof(info),
                "[HEAVY] fps=%.0f  %.1fms  n=%d | val_min=%d  peak=%d  bg_k=%d  area=[%d..%d]  circ=%.2f",
                fpsSmooth, ms, (int)dets.size(),
                c.valMin, c.peakFloor, c.bgKernel, c.areaMin, c.areaMax, c.circMin);
        }

        cv::putText(grid, info, {8, grid.rows - 10},
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, {255,255,0}, 2);

        std::printf("\r%s   ", info);
        std::fflush(stdout);

        cv::imshow(WIN, grid);
        cv::waitKey(1);

        if (key == 's') {
            cv::imwrite("/tmp/snap.jpg", grid);
            std::printf("\nSnapshot: /tmp/snap.jpg\n");
        }
        if (key == 'f') {
            std::time_t now_t = std::time(nullptr);
            char ts[20];
            std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", std::localtime(&now_t));
            std::string raw_path  = std::string("/tmp/debug_raw_")  + ts + ".png";
            std::string grid_path = std::string("/tmp/debug_grid_") + ts + ".png";
            cv::imwrite(raw_path,  bgr);
            cv::imwrite(grid_path, grid);
            std::printf("\nSaved: %s\n       %s\n", raw_path.c_str(), grid_path.c_str());
        }
    }

    cv::destroyAllWindows();
    std::printf("\nGestopt.\n");
    return 0;
}
