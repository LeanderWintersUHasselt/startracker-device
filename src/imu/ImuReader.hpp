#pragma once
#include <array>
#include <atomic>
#include <cstdint>
#include <string>
#include <thread>

namespace imu {

/// Raw quaternion output from the BNO085 rotation vector report.
struct Quaternion {
    float i = 0.f;
    float j = 0.f;
    float k = 0.f;
    float real = 1.f;  ///< w component
};

/// Yaw/pitch/roll in degrees extracted from a quaternion.
/// Roll  = rotation about X axis.
/// Pitch = rotation about Y axis.
/// Yaw   = rotation about Z axis (heading).
struct EulerAngles {
    float roll_deg  = 0.f;
    float pitch_deg = 0.f;
    float yaw_deg   = 0.f;
};

/// Converts a unit quaternion to Euler angles (ZYX convention).
EulerAngles quaternionToEuler(const Quaternion& q);

/// Combined raw IMU measurement (accelerometer + gyroscope).
/// accel_valid/gyro_valid are false until the first report of each type arrives.
struct ImuMeasBuf {
    float ax = 0, ay = 0, az = 0;  ///< accelerometer [m/s²], IMU frame
    float gx = 0, gy = 0, gz = 0;  ///< gyroscope [rad/s], IMU frame
    uint64_t stamp_us = 0;          ///< monotonic µs (steady_clock)
    bool accel_valid = false;
    bool gyro_valid  = false;
};

/// Reads the BNO085 IMU via Linux i2c-dev (/dev/i2c-1, address 0x4A).
/// Spawns an internal thread that reads quaternion data at ~100 Hz.
/// Thread-safe: latest quaternion/euler are stored atomically-updated doubles
/// and safe to read from Core 3 while the reader thread runs.
///
/// SHTP channel assignments (BNO085 datasheet):
///   Channel 0: SHTP command channel
///   Channel 1: SHTP executable channel
///   Channel 2: Sensor Hub Control (feature commands)
///   Channel 3: Sensor Hub Input: sensor reports
///   Channel 4: Wake sensor reports
///   Channel 5: Gyro-Rotation Vector reports
///
/// NOTE: This class uses Linux-specific i2c-dev headers and ioctl.
/// On non-Linux platforms, start() returns false immediately.
///
class ImuReader {
public:
    /// @param device  I2C device path, e.g. "/dev/i2c-1"
    /// @param addr    I2C 7-bit address, default 0x4A
    ImuReader(const std::string& device = "/dev/i2c-1", uint8_t addr = 0x4A);
    ~ImuReader();

    // Non-copyable
    ImuReader(const ImuReader&) = delete;
    ImuReader& operator=(const ImuReader&) = delete;

    /// Open the I2C device, enable the rotation vector report, start reader thread.
    /// Returns true on success, false if the device cannot be opened or SHTP init fails.
    /// On non-Linux platforms always returns false.
    bool start();

    /// Stop the reader thread and close the I2C file descriptor.
    void stop();

    /// True if start() succeeded and the thread is running.
    bool isRunning() const { return running_.load(std::memory_order_relaxed); }

    /// Latest raw quaternion (updated by reader thread at ~100 Hz).
    Quaternion quaternion() const;

    /// Latest Euler angles derived from the quaternion.
    EulerAngles euler() const;

    /// Latest raw IMU measurement (updated by reader thread at ~100 Hz).
    /// Returns the last written ImuMeasBuf; accel_valid/gyro_valid indicate
    /// whether the respective reports have arrived since start().
    ImuMeasBuf imuMeasurement() const;

    /// Number of successful quaternion reads since start() (for diagnostics).
    uint64_t readCount() const { return read_count_.load(std::memory_order_relaxed); }

    /// Number of SHTP read errors since start() (for diagnostics).
    uint64_t errorCount() const { return error_count_.load(std::memory_order_relaxed); }

private:
    std::string device_;
    uint8_t     addr_;
    int         fd_{ -1 };

    std::thread         thread_;
    std::atomic<bool>   running_{ false };
    std::atomic<bool>   stop_flag_{ false };

    // Latest values — written by reader thread, read by Core 3.
    // Stored as 4 floats (i, j, k, real) for lock-free access.
    // We use a seqlock-lite pattern: two copies + atomic counter.
    struct QuatBuf {
        float i, j, k, real;
    };
    std::atomic<uint32_t> quat_seq_{ 0 };
    QuatBuf               quat_a_{};
    QuatBuf               quat_b_{};

    std::atomic<uint64_t> read_count_{ 0 };
    std::atomic<uint64_t> error_count_{ 0 };

    // Seqlock for combined accel+gyro measurement
    std::atomic<uint32_t> meas_seq_{ 0 };
    ImuMeasBuf             meas_a_{};
    ImuMeasBuf             meas_b_{};

    // ── SHTP / I2C internals ──────────────────────────────────────────────────

    /// Read up to max_len bytes from the I2C bus into buf.
    bool i2c_read(uint8_t* buf, int len);

    /// Write len bytes from buf to the I2C bus.
    bool i2c_write(const uint8_t* buf, int len);

    /// Read one complete SHTP packet. Returns payload length (0 on error/empty).
    /// Fills channel and payload (up to payload_max bytes).
    int shtp_read_packet(uint8_t* channel_out, uint8_t* payload, int payload_max);

    /// Send a SHTP Set Feature Command to enable a sensor report.
    /// @param report_id    BNO085 sensor report ID (e.g. 0x05 for rotation vector)
    /// @param interval_us  Report interval in microseconds (10000 = 100 Hz)
    bool shtp_enable_report(uint8_t report_id, uint32_t interval_us);

    /// Send a raw SHTP packet and advance the transmit sequence for that channel.
    bool shtp_send_packet(uint8_t channel, const uint8_t* payload, int payload_len);

    /// Reset the BNO085 SHTP application into a known unconfigured state.
    bool shtp_soft_reset();

    /// Parse a rotation vector report payload (report ID 0x05).
    /// Returns true and fills quat if the report ID matches.
    bool parse_rotation_vector(const uint8_t* payload, int len, QuatBuf& quat);

    bool parse_accel(const uint8_t* payload, int len, ImuMeasBuf& meas);
    bool parse_gyro (const uint8_t* payload, int len, ImuMeasBuf& meas);
    void writeImuMeas(const ImuMeasBuf& m);

    /// Reader thread main loop.
    void readerLoop();

    /// Write quaternion using seqlock pattern.
    void writeQuat(const QuatBuf& q);

    std::array<uint8_t, 6> tx_seq_{};
};

} // namespace imu
