// Integration test: start SocketServer in a thread, connect as a client,
// send {"cmd":"get_status"}\n, assert the callback fires with cmd=="get_status".
// Run with: g++ -std=c++17 -I. -Isrc
//               src/ipc/test_socket.cpp src/ipc/SocketServer.cpp
//               -lpthread -o /tmp/test_socket && /tmp/test_socket
#include "ipc/SocketServer.hpp"
#include <atomic>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <sys/socket.h>
#include <sys/un.h>
#include <thread>
#include <unistd.h>

static constexpr const char* TEST_SOCK = "/tmp/test_startracker.sock";

int main() {
    std::atomic<bool> got_cmd{false};
    std::string received_cmd;

    SocketServer srv(TEST_SOCK);
    srv.onCommand([&](const std::string& cmd, const std::string& raw_line,
                      SocketServer::Sender send) {
        received_cmd = cmd;
        got_cmd = true;
        send("{\"event\":\"status\",\"calibration_complete\":false,"
             "\"tracking_active\":false}\n");
    });

    assert(srv.bind() && "bind failed");

    // Start server in background thread
    std::thread t([&]{ srv.runOnce(/*timeout_ms=*/500); });

    // Client: connect and send a command
    ::usleep(50'000);  // let server enter accept()
    int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    assert(fd >= 0);
    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    ::strncpy(addr.sun_path, TEST_SOCK, sizeof(addr.sun_path)-1);
    assert(::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0);

    const char* msg = "{\"cmd\":\"get_status\"}\n";
    ::write(fd, msg, ::strlen(msg));
    ::usleep(100'000);
    ::close(fd);

    t.join();
    srv.cleanup();

    assert(got_cmd.load() && "command callback never fired");
    assert(received_cmd == "get_status" && "wrong cmd value extracted");
    printf("SocketServer: all assertions passed.\n");
}
