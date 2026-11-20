#pragma once

#include <chrono>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace mmsim::platform {

/// A child process with piped stdin/stdout/stderr (Win32 CreateProcess; the OS details stay in
/// process.cpp). stdout and stderr are read on background threads and split into lines
/// ("\r\n" or "\n"). The child is placed in a job object so it never outlives the simulator.
class ChildProcess {
  public:
    ChildProcess();
    ~ChildProcess();  // kills the child if still running
    ChildProcess(const ChildProcess&) = delete;
    ChildProcess& operator=(const ChildProcess&) = delete;

    /// Starts `exe` with `args` in `cwd` (empty = the exe's folder). false + error on failure.
    bool start(const std::filesystem::path& exe, const std::vector<std::string>& args,
               const std::filesystem::path& cwd, std::string& error);

    /// Writes raw bytes to the child's stdin. false if the pipe is closed.
    bool write(std::string_view data);
    /// Next stdout line; waits up to `timeout`. nullopt on timeout or once stdout reached EOF
    /// and every line was taken (eof() tells which).
    std::optional<std::string> read_line(std::chrono::milliseconds timeout);
    bool eof() const;  // stdout closed and drained
    /// stderr lines received so far (removed from the buffer).
    std::vector<std::string> take_stderr_lines();

    bool running() const;
    /// Waits for the child to exit; false on timeout.
    bool wait(std::chrono::milliseconds timeout);
    std::optional<int> exit_code() const;  // once exited
    void kill();                           // terminate (no-op if not running)

  private:
    struct Impl;
    std::unique_ptr<Impl> p_;
};

}  // namespace mmsim::platform
