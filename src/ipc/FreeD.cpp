#include "ipc/FreeD.hpp"

#include <algorithm>
#include <arpa/inet.h>
#include <cmath>
#include <cstring>
#include <netinet/in.h>
#include <stdexcept>
#include <sys/socket.h>
#include <unistd.h>

namespace ipc {

// ── Helpers ───────────────────────────────────────────────────────────────────

void FreeD::write24be(uint8_t* dst, int32_t val) {
    // Clamp to signed 24-bit range [-8388608, 8388607] before byte truncation.
    // Without this, out-of-range values wrap silently and corrupt the FreeD packet.
    static constexpr int32_t S24_MIN = -8388608;
    static constexpr int32_t S24_MAX =  8388607;
    val = std::max(S24_MIN, std::min(S24_MAX, val));
    dst[0] = static_cast<uint8_t>((val >> 16) & 0xFF);
    dst[1] = static_cast<uint8_t>((val >>  8) & 0xFF);
    dst[2] = static_cast<uint8_t>( val        & 0xFF);
}

uint8_t FreeD::checksum(const std::array<uint8_t, 29>& pkt) {
    uint32_t cksum = 0x40;
    for (int i = 0; i < 28; ++i) {
        cksum -= pkt[i];
    }
    return static_cast<uint8_t>(cksum & 0xFF);
}

// ── encode (static, no socket needed) ────────────────────────────────────────

std::array<uint8_t, 29> FreeD::encode(
    uint8_t camera_id,
    double pan_deg, double tilt_deg, double roll_deg,
    double x_m, double y_m, double z_m)
{
    std::array<uint8_t, 29> pkt{};

    // Byte 0: packet type
    pkt[0] = 0xD1;
    // Byte 1: camera id
    pkt[1] = camera_id;

    // Angles: degrees × 32768, signed 24-bit BE
    write24be(&pkt[2],  static_cast<int32_t>(std::round(pan_deg  * 32768.0)));
    write24be(&pkt[5],  static_cast<int32_t>(std::round(tilt_deg * 32768.0)));
    write24be(&pkt[8],  static_cast<int32_t>(std::round(roll_deg * 32768.0)));

    // Position: metres → mm × 64, signed 24-bit BE
    write24be(&pkt[11], static_cast<int32_t>(std::round(x_m * 1000.0 * 64.0)));
    write24be(&pkt[14], static_cast<int32_t>(std::round(y_m * 1000.0 * 64.0)));
    write24be(&pkt[17], static_cast<int32_t>(std::round(z_m * 1000.0 * 64.0)));

    // Bytes 20-21: zoom = 0 (big-endian uint16)
    pkt[20] = 0x00; pkt[21] = 0x00;
    // Bytes 22-23: focus = 0
    pkt[22] = 0x00; pkt[23] = 0x00;
    // Bytes 24-27: spare / padding = 0x00
    // Byte 28: FreeD checksum
    pkt[28] = checksum(pkt);

    return pkt;
}

// ── Constructor / Destructor ──────────────────────────────────────────────────

FreeD::FreeD(const std::string& ip, uint16_t port, uint8_t camera_id)
    : camera_id_(camera_id)
{
    openSocket();
    retarget(ip, port);  // populate dest_addr_ once at construction
}

FreeD::~FreeD() {
    closeSocket();
}

void FreeD::openSocket() {
    closeSocket();
    sock_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd_ < 0)
        throw std::runtime_error("FreeD: failed to create UDP socket");
}

void FreeD::closeSocket() {
    if (sock_fd_ >= 0) {
        ::close(sock_fd_);
        sock_fd_ = -1;
    }
}

void FreeD::retarget(const std::string& ip, uint16_t port) {
    std::memset(&dest_addr_, 0, sizeof(dest_addr_));
    dest_addr_.sin_family      = AF_INET;
    dest_addr_.sin_port        = htons(port);
    dest_addr_.sin_addr.s_addr = ::inet_addr(ip.c_str());
    // inet_addr() is called ONLY here, not in send()
}

// ── send ──────────────────────────────────────────────────────────────────────

bool FreeD::send(double pan_deg, double tilt_deg, double roll_deg,
                 double x_m, double y_m, double z_m)
{
    auto pkt = encode(camera_id_, pan_deg, tilt_deg, roll_deg, x_m, y_m, z_m);

    // Use pre-built dest_addr_ — no per-packet address resolution
    ssize_t sent = ::sendto(sock_fd_,
                            pkt.data(), pkt.size(), 0,
                            reinterpret_cast<const struct sockaddr*>(&dest_addr_),
                            sizeof(dest_addr_));
    return sent == static_cast<ssize_t>(pkt.size());
}

} // namespace ipc
