#include "imu/ImuReader.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>

#ifdef __linux__
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#endif

namespace imu {

// ── quaternionToEuler ─────────────────────────────────────────────────────────

EulerAngles quaternionToEuler(const Quaternion& q) {
    // ZYX Euler angles (yaw-pitch-roll, aerospace convention)
    // See: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

    float w = q.real, x = q.i, y = q.j, z = q.k;

    // Roll (x-axis rotation)
    float sinr_cosp = 2.f * (w * x + y * z);
    float cosr_cosp = 1.f - 2.f * (x * x + y * y);
    float roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    float sinp = 2.f * (w * y - z * x);
    float pitch;
    if (std::abs(sinp) >= 1.f)
        pitch = std::copysign(static_cast<float>(M_PI / 2.0), sinp); // clamp ±90°
    else
        pitch = std::asin(sinp);

    // Yaw (z-axis rotation)
    float siny_cosp = 2.f * (w * z + x * y);
    float cosy_cosp = 1.f - 2.f * (y * y + z * z);
    float yaw = std::atan2(siny_cosp, cosy_cosp);

    constexpr float RAD2DEG = 180.f / static_cast<float>(M_PI);
    return EulerAngles{
        roll  * RAD2DEG,
        pitch * RAD2DEG,
        yaw   * RAD2DEG
    };
}

// ── ImuReader ─────────────────────────────────────────────────────────────────

ImuReader::ImuReader(const std::string& device, uint8_t addr)
    : device_(device), addr_(addr) {}

ImuReader::~ImuReader() {
    stop();
}

// ── I2C low-level ─────────────────────────────────────────────────────────────

bool ImuReader::i2c_read(uint8_t* buf, int len) {
#ifdef __linux__
    int n = ::read(fd_, buf, len);
    return n == len;
#else
    (void)buf; (void)len;
    return false;
#endif
}

bool ImuReader::i2c_write(const uint8_t* buf, int len) {
#ifdef __linux__
    int n = ::write(fd_, buf, len);
    return n == len;
#else
    (void)buf; (void)len;
    return false;
#endif
}

// ── SHTP packet I/O ───────────────────────────────────────────────────────────

// SHTP header layout:
//   byte 0: length LSB (includes 4-byte header)
//   byte 1: length MSB (bit 15 = continuation flag, ignore for simple reads)
//   byte 2: channel
//   byte 3: sequence number (ignored for our use)

int ImuReader::shtp_read_packet(uint8_t* channel_out, uint8_t* payload, int payload_max) {
    // Step 1: read the 4-byte SHTP header
    uint8_t hdr[4] = {};
    if (!i2c_read(hdr, 4)) return 0;

    // Total packet length including header (mask out continuation bit)
    int total_len = static_cast<int>((hdr[1] & 0x7F) << 8 | hdr[0]);
    if (total_len < 4 || total_len > 512) {
        // Malformed or unexpectedly huge packet.
        return 0;
    }

    *channel_out = hdr[2];
    int payload_len = total_len - 4;
    if (payload_len == 0) return 0;

    // Step 2: read the payload bytes
    // The BNO085 over I2C delivers header + payload in a single I2C read;
    // we already consumed the header, so re-read the full packet.
    // In practice, re-issuing a full read of `total_len` bytes is the
    // correct approach for the BNO085 i2c-dev interface.
    uint8_t full[512] = {};
    if (!i2c_read(full, total_len)) return 0;

    if (payload_len > payload_max) {
        return 0;
    }

    // full[0..3] = header again (discard), full[4..] = payload
    std::memcpy(payload, full + 4, static_cast<size_t>(payload_len));
    return payload_len;
}

// ── shtp_enable_report ────────────────────────────────────────────────────────
//
// BNO085 Set Feature Command (SHTP channel 2, report ID 0xFD):
//   Byte  0: report ID = 0xFD
//   Byte  1: feature report ID (e.g. 0x05 = rotation vector)
//   Byte  2: flags = 0
//   Byte  3-4: change sensitivity (0 = disabled)
//   Byte  5-8: report interval (microseconds, LE uint32)
//   Byte  9-12: batch interval (0)
//   Byte 13-16: sensor-specific config (0)
// Total = 17 bytes payload + 4 bytes SHTP header = 21 bytes.

bool ImuReader::shtp_enable_report(uint8_t report_id, uint32_t interval_us) {
    uint8_t payload[17] = {};
    payload[0] = 0xFD;       // report ID: Set Feature Command
    payload[1] = report_id;  // feature to enable (0x05 = rotation vector)
    payload[2] = 0;          // flags
    payload[3] = 0;          // change sensitivity LSB
    payload[4] = 0;          // change sensitivity MSB
    // Report interval in microseconds, LE uint32
    payload[5] = static_cast<uint8_t>(interval_us & 0xFF);
    payload[6] = static_cast<uint8_t>((interval_us >>  8) & 0xFF);
    payload[7] = static_cast<uint8_t>((interval_us >> 16) & 0xFF);
    payload[8] = static_cast<uint8_t>((interval_us >> 24) & 0xFF);
    // Batch interval and sensor-specific config: all zero (cmd already zeroed)

    return shtp_send_packet(2, payload, sizeof(payload));
}

bool ImuReader::shtp_send_packet(uint8_t channel, const uint8_t* payload, int payload_len) {
    if (channel >= tx_seq_.size() || payload_len < 0 || payload_len > 252) return false;

    const int total_len = payload_len + 4;
    uint8_t packet[256] = {};
    packet[0] = static_cast<uint8_t>(total_len & 0xFF);
    packet[1] = static_cast<uint8_t>((total_len >> 8) & 0x7F);
    packet[2] = channel;
    packet[3] = tx_seq_[channel];
    if (payload_len > 0) {
        std::memcpy(packet + 4, payload, static_cast<size_t>(payload_len));
    }

    if (!i2c_write(packet, total_len)) return false;
    tx_seq_[channel] = static_cast<uint8_t>(tx_seq_[channel] + 1);
    return true;
}

bool ImuReader::shtp_soft_reset() {
    uint8_t reset_payload[1] = {1};
    if (!shtp_send_packet(1, reset_payload, sizeof(reset_payload))) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    if (!shtp_send_packet(1, reset_payload, sizeof(reset_payload))) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return true;
}

// ── parse_rotation_vector ─────────────────────────────────────────────────────
//
// BNO085 Rotation Vector report (report ID 0x05):
//   Byte  0: report ID = 0x05
//   Byte  1: sequence number
//   Byte  2: status
//   Byte  3: delay LSB
//   Byte  4: delay MSB
//   Byte  5-6: i (int16 LE, Q14 fixed-point)
//   Byte  7-8: j (int16 LE, Q14 fixed-point)
//   Byte  9-10: k (int16 LE, Q14 fixed-point)
//   Byte 11-12: real (int16 LE, Q14 fixed-point)
//   Byte 13-14: accuracy estimate (int16 LE, Q12)
// Total payload: 15 bytes minimum.

bool ImuReader::parse_rotation_vector(const uint8_t* payload, int len, QuatBuf& quat) {
    if (len < 12) return false;
    if (payload[0] != 0x05) return false;

    constexpr float Q14_SCALE = 1.0f / 16384.0f;  // 2^14

    int16_t raw_i    = static_cast<int16_t>(payload[4]  | (payload[5]  << 8));
    int16_t raw_j    = static_cast<int16_t>(payload[6]  | (payload[7]  << 8));
    int16_t raw_k    = static_cast<int16_t>(payload[8]  | (payload[9]  << 8));
    int16_t raw_real = static_cast<int16_t>(payload[10] | (payload[11] << 8));

    quat.i    = static_cast<float>(raw_i)    * Q14_SCALE;
    quat.j    = static_cast<float>(raw_j)    * Q14_SCALE;
    quat.k    = static_cast<float>(raw_k)    * Q14_SCALE;
    quat.real = static_cast<float>(raw_real) * Q14_SCALE;
    return true;
}

// ── parse_accel ───────────────────────────────────────────────────────────────
//
// BNO085 Accelerometer report (report ID 0x01):
//   Byte  0: report ID = 0x01
//   Byte  1: sequence number
//   Byte  2: status
//   Byte  3-4: delay (100 µs units)
//   Byte  5-6: x (int16 LE, Q8 = 1/256 m/s²)
//   Byte  7-8: y
//   Byte  9-10: z
// Minimum payload: 11 bytes.

bool ImuReader::parse_accel(const uint8_t* payload, int len, ImuMeasBuf& meas) {
    if (len < 10) return false;
    if (payload[0] != 0x01) return false;

    constexpr float Q8_SCALE = 1.0f / 256.0f;

    int16_t rx = static_cast<int16_t>(payload[4] | (payload[5] << 8));
    int16_t ry = static_cast<int16_t>(payload[6] | (payload[7] << 8));
    int16_t rz = static_cast<int16_t>(payload[8] | (payload[9] << 8));

    meas.ax = static_cast<float>(rx) * Q8_SCALE;
    meas.ay = static_cast<float>(ry) * Q8_SCALE;
    meas.az = static_cast<float>(rz) * Q8_SCALE;
    meas.accel_valid = true;
    return true;
}

// ── parse_gyro ────────────────────────────────────────────────────────────────
//
// BNO085 Gyroscope Calibrated report (report ID 0x02):
//   Same layout as accelerometer, Q9 = 1/512 rad/s.

bool ImuReader::parse_gyro(const uint8_t* payload, int len, ImuMeasBuf& meas) {
    if (len < 10) return false;
    if (payload[0] != 0x02) return false;

    constexpr float Q9_SCALE = 1.0f / 512.0f;

    int16_t rx = static_cast<int16_t>(payload[4] | (payload[5] << 8));
    int16_t ry = static_cast<int16_t>(payload[6] | (payload[7] << 8));
    int16_t rz = static_cast<int16_t>(payload[8] | (payload[9] << 8));

    meas.gx = static_cast<float>(rx) * Q9_SCALE;
    meas.gy = static_cast<float>(ry) * Q9_SCALE;
    meas.gz = static_cast<float>(rz) * Q9_SCALE;
    meas.gyro_valid = true;
    return true;
}

// ── writeImuMeas / imuMeasurement ─────────────────────────────────────────────

void ImuReader::writeImuMeas(const ImuMeasBuf& m) {
    uint32_t seq = meas_seq_.load(std::memory_order_relaxed);
    meas_seq_.store(seq | 1u, std::memory_order_release);
    if (seq & 2u) meas_a_ = m;
    else          meas_b_ = m;
    meas_seq_.store(seq + 2u, std::memory_order_release);
}

ImuMeasBuf ImuReader::imuMeasurement() const {
    uint32_t seq1, seq2;
    ImuMeasBuf buf;
    do {
        seq1 = meas_seq_.load(std::memory_order_acquire);
        if (seq1 & 1u) continue;
        buf  = (seq1 & 2u) ? meas_b_ : meas_a_;
        seq2 = meas_seq_.load(std::memory_order_acquire);
    } while (seq1 != seq2);
    return buf;
}

// ── Seqlock write ─────────────────────────────────────────────────────────────

void ImuReader::writeQuat(const QuatBuf& q) {
    uint32_t seq = quat_seq_.load(std::memory_order_relaxed);
    quat_seq_.store(seq | 1u, std::memory_order_release);
    if (seq & 2u)
        quat_a_ = q;
    else
        quat_b_ = q;
    quat_seq_.store(seq + 2u, std::memory_order_release);
}

// ── Public quaternion/euler accessors ─────────────────────────────────────────

Quaternion ImuReader::quaternion() const {
    // Read the seqlock consistently
    uint32_t seq1, seq2;
    QuatBuf buf;
    do {
        seq1 = quat_seq_.load(std::memory_order_acquire);
        if (seq1 & 1u) continue;  // write in progress
        buf  = (seq1 & 2u) ? quat_b_ : quat_a_;
        seq2 = quat_seq_.load(std::memory_order_acquire);
    } while (seq1 != seq2);
    return Quaternion{ buf.i, buf.j, buf.k, buf.real };
}

EulerAngles ImuReader::euler() const {
    return quaternionToEuler(quaternion());
}

// ── Reader thread ─────────────────────────────────────────────────────────────

void ImuReader::readerLoop() {
    uint8_t channel = 0;
    uint8_t payload[64] = {};
    ImuMeasBuf pending{};

    while (!stop_flag_.load(std::memory_order_relaxed)) {
        int len = shtp_read_packet(&channel, payload, sizeof(payload));
        if (len <= 0) {
            error_count_.fetch_add(1, std::memory_order_relaxed);
            std::this_thread::sleep_for(std::chrono::microseconds(500));
            continue;
        }

        if (channel != 3 && channel != 4) continue;

        for (int off = 0; off < len;) {
            const uint8_t report_id = payload[off];
            if (report_id == 0xFB || report_id == 0xFA) {
                off += 5;  // base timestamp / timestamp rebase
                continue;
            }

            QuatBuf quat;
            if (report_id == 0x05 && off + 14 <= len &&
                parse_rotation_vector(payload + off, len - off, quat)) {
                writeQuat(quat);
                read_count_.fetch_add(1, std::memory_order_relaxed);
                off += 14;
            } else if (report_id == 0x01 && off + 10 <= len &&
                       parse_accel(payload + off, len - off, pending)) {
                pending.stamp_us = static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now().time_since_epoch()).count());
                writeImuMeas(pending);
                off += 10;
            } else if (report_id == 0x02 && off + 10 <= len &&
                       parse_gyro(payload + off, len - off, pending)) {
                pending.stamp_us = static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now().time_since_epoch()).count());
                writeImuMeas(pending);
                off += 10;
            } else {
                ++off;
            }
        }
    }
}

// ── start / stop ──────────────────────────────────────────────────────────────

bool ImuReader::start() {
#ifndef __linux__
    // I2C via linux/i2c-dev.h is not available on this platform.
    std::fprintf(stderr, "[ImuReader] WARNING: Linux i2c-dev not available on this platform\n");
    return false;
#else
    // Open I2C device
    fd_ = ::open(device_.c_str(), O_RDWR);
    if (fd_ < 0) return false;

    // Set slave address
    if (::ioctl(fd_, I2C_SLAVE, static_cast<int>(addr_)) < 0) {
        ::close(fd_); fd_ = -1;
        return false;
    }

    // Allow BNO085 to finish startup (it may send an advertisement packet)
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    tx_seq_.fill(0);

    if (!shtp_soft_reset()) {
        ::close(fd_); fd_ = -1;
        return false;
    }

    // Enable rotation vector report at 100 Hz (10000 µs interval)
    if (!shtp_enable_report(0x05, 10000)) {
        ::close(fd_); fd_ = -1;
        return false;
    }
    // Enable accelerometer and calibrated gyroscope at 100 Hz
    if (!shtp_enable_report(0x01, 10000)) {
        ::close(fd_); fd_ = -1;
        return false;
    }
    if (!shtp_enable_report(0x02, 10000)) {
        ::close(fd_); fd_ = -1;
        return false;
    }

    stop_flag_.store(false, std::memory_order_relaxed);
    running_.store(true, std::memory_order_relaxed);
    thread_ = std::thread(&ImuReader::readerLoop, this);
    return true;
#endif
}

void ImuReader::stop() {
    stop_flag_.store(true, std::memory_order_relaxed);
    if (thread_.joinable()) thread_.join();
    running_.store(false, std::memory_order_relaxed);
#ifdef __linux__
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
#endif
}

} // namespace imu
