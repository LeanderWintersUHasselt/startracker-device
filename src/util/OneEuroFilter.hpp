#pragma once
#include <cmath>

// One Euro Filter — adaptive low-pass filter for noisy real-time signals.
// Reference: Casiez et al., "1€ Filter: A Simple Speed-based Low-pass Filter
// for Noisy Input in Interactive Systems", CHI 2012.
//
// Reduces high-frequency noise (vibrations) while keeping low lag during
// fast intentional motion. Two parameters:
//   min_cutoff  — base cutoff Hz; lower = more smoothing when still
//   beta        — speed coefficient; higher = less lag during fast motion
//
// Set min_cutoff = 0 to bypass filtering entirely (passthrough).
// For angular signals (yaw/pitch/roll) set angular = true to handle wraparound.

class OneEuroFilter {
public:
    OneEuroFilter() = default;

    double filter(double x, double dt_s, double min_cutoff, double beta,
                  bool angular = false) {
        if (min_cutoff <= 0.0) return x;
        if (!init_ || dt_s <= 0.0) {
            x_filt_  = x;
            dx_filt_ = 0.0;
            init_    = true;
            return x;
        }

        // Derivative in the signal's tangent space (handles angle wrap)
        double dx_raw = angular
            ? wrapPi(x - x_filt_) / dt_s
            : (x - x_filt_)       / dt_s;

        // Smooth derivative with fixed 1 Hz cutoff
        double ad    = alpha(1.0, dt_s);
        dx_filt_     = ad * dx_raw + (1.0 - ad) * dx_filt_;

        // Adaptive cutoff based on speed
        double cutoff = min_cutoff + beta * std::abs(dx_filt_);
        double a      = alpha(cutoff, dt_s);

        // For angles, step in tangent space to avoid jumps
        if (angular)
            x_filt_ = wrapPi(x_filt_ + a * wrapPi(x - x_filt_));
        else
            x_filt_ = a * x + (1.0 - a) * x_filt_;

        return x_filt_;
    }

    void reset() { init_ = false; }

private:
    static double alpha(double cutoff_hz, double dt_s) {
        double tau = 1.0 / (2.0 * M_PI * cutoff_hz);
        return 1.0 / (1.0 + tau / dt_s);
    }

    static double wrapPi(double a) {
        while (a >  M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    bool   init_    = false;
    double x_filt_  = 0.0;
    double dx_filt_ = 0.0;
};
