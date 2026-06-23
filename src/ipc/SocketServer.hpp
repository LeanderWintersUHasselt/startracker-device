#pragma once
#include <functional>
#include <string>

// SocketServer — Unix domain socket server for the startracker daemon.
//
// Accepts exactly one client at a time (one frontend). Reads newline-delimited
// JSON lines (max 4096 bytes each); lines exceeding that limit are treated as
// framing errors and the connection is reset.
//
// The onCommand callback receives:
//   cmd      — the value of the "cmd" JSON key (extracted by simple string scan)
//   raw_line — the full JSON line (NUL-terminated, no trailing newline)
//   send     — callable to write a reply line to the client (appends '\n' if absent)
//
// JSON parsing strategy: nlohmann/json is not available in this project.
// We extract the "cmd" key with a minimal scanner (find "\"cmd\"" then the next
// quoted string value). For commands with parameters (set_settings, set_kalman,
// build_map, calibrate_scale, list_files), the full raw_line is passed to the
// callback so the command handler can parse it with its own minimal scanner.
class SocketServer {
public:
    using Sender = std::function<void(const std::string&)>;
    using CommandHandler = std::function<void(const std::string& cmd,
                                               const std::string& raw_line,
                                               Sender send)>;

    explicit SocketServer(const char* sock_path);
    ~SocketServer();

    // Register the command dispatch callback. Call before bind().
    void onCommand(CommandHandler h);

    // Create and bind the socket. Returns false on error.
    // Also calls mkdir -p on the socket directory.
    bool bind();

    // Block until one client connects, processes commands until disconnect,
    // then returns. timeout_ms: accept() timeout in milliseconds (0 = infinite).
    // Intended to be called in a loop from the main thread.
    void runOnce(int timeout_ms = 0);

    // Close socket fd and unlink socket file.
    void cleanup();

    // Send a line to the currently connected client (thread-safe).
    // Appends '\n' if the string does not already end with one.
    // No-op if no client is connected.
    void send(const std::string& line);

private:
    // Extract the value of the "cmd" key from a JSON line.
    // Returns empty string if not found.
    static std::string extractCmd(const std::string& line);

    // Extract a string field value from a JSON line.
    // e.g. extractStr(line, "file") returns the string value of "file".
    static std::string extractStr(const std::string& line, const std::string& key);

    // Extract a double field value from a JSON line. Returns def if not found.
    static double extractDbl(const std::string& line,
                              const std::string& key, double def = 0.0);

    std::string      sock_path_;
    CommandHandler   handler_;
    int              listen_fd_{-1};
    int              client_fd_{-1};
};
