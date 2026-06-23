#include "SocketServer.hpp"
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

namespace fs = std::filesystem;

SocketServer::SocketServer(const char* sock_path)
    : sock_path_(sock_path) {}

SocketServer::~SocketServer() { cleanup(); }

void SocketServer::onCommand(CommandHandler h) { handler_ = std::move(h); }

bool SocketServer::bind() {
    // Create socket directory (e.g. /run/startracker/) if it does not exist.
    fs::path dir = fs::path(sock_path_).parent_path();
    if (!dir.empty()) {
        std::error_code ec;
        fs::create_directories(dir, ec);
        // Non-fatal if it already exists.
    }

    // Remove stale socket file from a previous run.
    ::unlink(sock_path_.c_str());

    listen_fd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (listen_fd_ < 0) {
        std::perror("SocketServer: socket");
        return false;
    }

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    ::strncpy(addr.sun_path, sock_path_.c_str(), sizeof(addr.sun_path) - 1);

    if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        std::perror("SocketServer: bind");
        ::close(listen_fd_); listen_fd_ = -1;
        return false;
    }
    if (::listen(listen_fd_, 1) != 0) {
        std::perror("SocketServer: listen");
        ::close(listen_fd_); listen_fd_ = -1;
        return false;
    }
    return true;
}

void SocketServer::runOnce(int timeout_ms) {
    // Wait for a client connection (with optional timeout via select).
    if (timeout_ms > 0) {
        fd_set fds; FD_ZERO(&fds); FD_SET(listen_fd_, &fds);
        timeval tv{ timeout_ms / 1000, (timeout_ms % 1000) * 1000 };
        int r = ::select(listen_fd_ + 1, &fds, nullptr, nullptr, &tv);
        if (r <= 0) return;  // timeout or error — no client
    }

    client_fd_ = ::accept(listen_fd_, nullptr, nullptr);
    if (client_fd_ < 0) { std::perror("SocketServer: accept"); return; }

    // Line-buffered read loop.
    // Protocol: newline-delimited JSON, max 4096 bytes per line.
    // On framing error (line too long): reset connection.
    std::string buf;
    buf.reserve(512);
    char tmp[256];

    while (true) {
        ssize_t n = ::read(client_fd_, tmp, sizeof(tmp));
        if (n <= 0) break;  // disconnect or error

        buf.append(tmp, static_cast<size_t>(n));

        // Process all complete lines in buf.
        while (true) {
            auto nl = buf.find('\n');
            if (nl == std::string::npos) break;

            std::string line = buf.substr(0, nl);
            buf.erase(0, nl + 1);

            if (line.size() > 4096) {
                // Framing error: reset connection.
                std::fprintf(stderr, "SocketServer: line too long (%zu bytes), "
                             "resetting connection\n", line.size());
                goto disconnect;
            }

            std::string cmd = extractCmd(line);
            if (!cmd.empty() && handler_) {
                handler_(cmd, line, [this](const std::string& reply) {
                    this->send(reply);
                });
            }
        }

        // Partial line: buf still holds the incomplete bytes. Continue reading.
        // Guard against unbounded growth (framing error detection).
        if (buf.size() > 4096) {
            std::fprintf(stderr, "SocketServer: buffer overflow (no newline in "
                         "%zu bytes), resetting connection\n", buf.size());
            break;
        }
    }

disconnect:
    ::close(client_fd_);
    client_fd_ = -1;
}

void SocketServer::send(const std::string& line) {
    if (client_fd_ < 0) return;
    std::string out = line;
    if (out.empty() || out.back() != '\n') out += '\n';
    // Best-effort: ignore partial writes (line fits in one TCP segment on loopback).
    ::write(client_fd_, out.data(), out.size());
}

void SocketServer::cleanup() {
    if (client_fd_ >= 0) { ::close(client_fd_); client_fd_ = -1; }
    if (listen_fd_ >= 0) { ::close(listen_fd_); listen_fd_ = -1; }
    ::unlink(sock_path_.c_str());
}

// ── Minimal JSON field extractors ────────────────────────────────────────────
// Strategy: find the key as a quoted string, then find the colon, then extract
// the next quoted string or number. Sufficient for the small fixed protocol.

std::string SocketServer::extractCmd(const std::string& line) {
    return extractStr(line, "cmd");
}

std::string SocketServer::extractStr(const std::string& line,
                                      const std::string& key) {
    // Find: "key"
    std::string needle = "\"" + key + "\"";
    auto pos = line.find(needle);
    if (pos == std::string::npos) return {};
    pos += needle.size();

    // Skip whitespace and colon
    while (pos < line.size() && (line[pos] == ' ' || line[pos] == ':')) ++pos;
    if (pos >= line.size() || line[pos] != '"') return {};
    ++pos;  // skip opening quote

    std::string result;
    while (pos < line.size() && line[pos] != '"') {
        if (line[pos] == '\\' && pos + 1 < line.size()) {
            ++pos;  // skip escape char (basic handling)
        }
        result += line[pos++];
    }
    return result;
}

double SocketServer::extractDbl(const std::string& line,
                                 const std::string& key, double def) {
    std::string needle = "\"" + key + "\"";
    auto pos = line.find(needle);
    if (pos == std::string::npos) return def;
    pos += needle.size();
    while (pos < line.size() && (line[pos] == ' ' || line[pos] == ':')) ++pos;
    if (pos >= line.size()) return def;
    try { return std::stod(line.substr(pos)); }
    catch (...) { return def; }
}
