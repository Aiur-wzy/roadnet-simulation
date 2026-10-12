#ifndef CAMS_NATIVE_REPLAY_GUARD_H
#define CAMS_NATIVE_REPLAY_GUARD_H

#include <fstream>
#include <stdexcept>
#include <string>
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>
#endif

// Held by the actual simulator, not just its parent Python process. A killed
// coordinator cannot start a concurrent replay on resume. The token rejects a
// delayed child from an older coordinator after the lock was safely rotated.
class NativeReplayGuard {
public:
    NativeReplayGuard(const std::string& path, const std::string& token) {
        if (path.empty() && token.empty()) return;
        if (path.empty() || token.empty()) throw std::runtime_error("Native replay guard/token must be paired");
#ifdef _WIN32
        handle_ = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr,
                              OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
        if (handle_ == INVALID_HANDLE_VALUE) throw std::runtime_error("Native replay guard busy or inaccessible");
#else
        handle_ = open(path.c_str(), O_CREAT | O_RDWR, 0600);
        if (handle_ < 0 || flock(handle_, LOCK_EX | LOCK_NB) != 0) {
            close_handle();
            throw std::runtime_error("Native replay guard busy or inaccessible");
        }
#endif
        std::ifstream input(path + ".token", std::ios::binary);
        std::string expected;
        std::getline(input, expected);
        if (!input || expected != token) {
            close_handle();
            throw std::runtime_error("Native replay token stale or missing");
        }
    }
    ~NativeReplayGuard() { close_handle(); }
    NativeReplayGuard(const NativeReplayGuard&) = delete;
    NativeReplayGuard& operator=(const NativeReplayGuard&) = delete;
private:
#ifdef _WIN32
    HANDLE handle_ = INVALID_HANDLE_VALUE;
    void close_handle() { if (handle_ != INVALID_HANDLE_VALUE) CloseHandle(handle_); handle_ = INVALID_HANDLE_VALUE; }
#else
    int handle_ = -1;
    void close_handle() { if (handle_ >= 0) close(handle_); handle_ = -1; }
#endif
};
#endif
