#pragma once
#include <array>
#include <cstdint>
#include <netinet/in.h>
#include <string>

namespace ipc {

/// Encodes a FreeD D1 packet (29 bytes, big-endian) and sends it via UDP.
///
/// Axis mapping:  pan = yaw,  tilt = pitch,  roll = roll
/// Angle units:   degrees × 32768  (signed 24-bit BE)
/// Position units: metres → mm × 64  (signed 24-bit BE)
class FreeD {
public:
    /// @param ip    Target IP address string (e.g. "192.168.1.100")
    /// @param port  Target UDP port (e.g. 40000)
    /// @param camera_id  FreeD camera ID byte (default 1)
    FreeD(const std::string& ip, uint16_t port, uint8_t camera_id = 0x01);
    ~FreeD();

    // Non-copyable
    FreeD(const FreeD&) = delete;
    FreeD& operator=(const FreeD&) = delete;

    /// Encode a D1 packet into buf[0..28].
    /// All angle parameters in degrees, position parameters in metres.
    static std::array<uint8_t, 29> encode(
        uint8_t camera_id,
        double pan_deg,   ///< yaw
        double tilt_deg,  ///< pitch
        double roll_deg,  ///< roll
        double x_m,
        double y_m,
        double z_m);

    /// Send the encoded packet via UDP.  Returns true on success.
    bool send(double pan_deg, double tilt_deg, double roll_deg,
              double x_m, double y_m, double z_m);

    /// Reopen the socket to a new target (applied immediately, no pipeline restart).
    void retarget(const std::string& ip, uint16_t port);

private:
    int                sock_fd_   = -1;
    uint8_t            camera_id_ = 0x01;
    struct sockaddr_in dest_addr_{};  // cached — rebuilt only in retarget()

    void openSocket();
    void closeSocket();

    /// Write a signed 24-bit big-endian integer into dst[0..2].
    static void write24be(uint8_t* dst, int32_t val);

    // FreeD checksum
    static uint8_t checksum(const std::array<uint8_t, 29>& pkt);
};

} // namespace ipc
