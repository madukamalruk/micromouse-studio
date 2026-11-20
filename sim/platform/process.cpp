// Win32 child process with piped stdio. One of the few files allowed to include <windows.h>.
#include "sim/platform/process.hpp"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>

namespace mmsim::platform {

namespace {

std::wstring widen(const std::string& s) {
    if (s.empty()) return {};
    const int n = MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), nullptr, 0);
    std::wstring w(static_cast<std::size_t>(n), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), w.data(), n);
    return w;
}

/// Quotes one argument the way CommandLineToArgvW / the MSVC CRT parse it.
std::wstring quote_arg(const std::wstring& a) {
    if (!a.empty() && a.find_first_of(L" \t\n\v\"") == std::wstring::npos) return a;
    std::wstring out = L"\"";
    for (auto it = a.begin();; ++it) {
        std::size_t backslashes = 0;
        while (it != a.end() && *it == L'\\') {
            ++it;
            ++backslashes;
        }
        if (it == a.end()) {
            out.append(backslashes * 2, L'\\');
            break;
        }
        if (*it == L'"') {
            out.append(backslashes * 2 + 1, L'\\');
        } else {
            out.append(backslashes, L'\\');
        }
        out.push_back(*it);
    }
    out.push_back(L'"');
    return out;
}

std::string last_error_text(const char* what) {
    const DWORD code = GetLastError();
    char buf[256] = {};
    FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, nullptr, code, 0,
                   buf, sizeof buf, nullptr);
    std::string msg = std::string(what) + " failed (" + std::to_string(code) + "): " + buf;
    while (!msg.empty() && (msg.back() == '\n' || msg.back() == '\r' || msg.back() == ' '))
        msg.pop_back();
    return msg;
}

/// Lines read from one pipe on a background thread.
struct LineQueue {
    std::mutex m;
    std::condition_variable cv;
    std::deque<std::string> lines;
    std::string partial;
    bool eof = false;
    static constexpr std::size_t kMaxLine = 1 << 16;  // a runaway line is cut here
    static constexpr std::size_t kMaxLines = 100000;  // and the oldest lines are dropped

    void push_bytes(const char* p, std::size_t n) {
        std::lock_guard<std::mutex> lk(m);
        for (std::size_t i = 0; i < n; ++i) {
            const char c = p[i];
            if (c == '\n') {
                flush_locked();
            } else if (partial.size() < kMaxLine) {
                partial.push_back(c);
            }
        }
        cv.notify_all();
    }
    void flush_locked() {
        if (!partial.empty() && partial.back() == '\r') partial.pop_back();
        lines.push_back(std::move(partial));
        partial.clear();
        if (lines.size() > kMaxLines) lines.pop_front();
    }
    void finish() {
        std::lock_guard<std::mutex> lk(m);
        if (!partial.empty()) flush_locked();
        eof = true;
        cv.notify_all();
    }
};

void reader_loop(HANDLE h, LineQueue* q) {
    char buf[4096];
    for (;;) {
        DWORD got = 0;
        if (!ReadFile(h, buf, sizeof buf, &got, nullptr) || got == 0) break;
        q->push_bytes(buf, got);
    }
    q->finish();
}

}  // namespace

struct ChildProcess::Impl {
    HANDLE process = nullptr;
    HANDLE job = nullptr;
    HANDLE in_w = nullptr;   // our end of the child's stdin
    HANDLE out_r = nullptr;  // our end of the child's stdout
    HANDLE err_r = nullptr;  // our end of the child's stderr
    LineQueue out, err;
    std::thread out_thread, err_thread;
    std::optional<int> exit_code;

    void close(HANDLE& h) {
        if (h) CloseHandle(h);
        h = nullptr;
    }
    void poll_exit() {
        if (!process || exit_code) return;
        DWORD code = 0;
        if (WaitForSingleObject(process, 0) == WAIT_OBJECT_0 && GetExitCodeProcess(process, &code))
            exit_code = static_cast<int>(code);
    }
    void shutdown() {
        if (process) {
            poll_exit();
            if (!exit_code) {
                TerminateProcess(process, 1);
                WaitForSingleObject(process, 5000);
                DWORD code = 1;
                GetExitCodeProcess(process, &code);
                exit_code = static_cast<int>(code);
            }
        }
        close(in_w);
        // The child is gone, so its pipe write ends are closed and the readers see EOF.
        if (out_thread.joinable()) out_thread.join();
        if (err_thread.joinable()) err_thread.join();
        close(out_r);
        close(err_r);
        close(process);
        close(job);
    }
};

ChildProcess::ChildProcess() : p_(std::make_unique<Impl>()) {}
ChildProcess::~ChildProcess() {
    p_->shutdown();
}

bool ChildProcess::start(const std::filesystem::path& exe, const std::vector<std::string>& args,
                         const std::filesystem::path& cwd, std::string& error) {
    p_->shutdown();
    p_ = std::make_unique<Impl>();
    std::error_code ec;
    if (!std::filesystem::is_regular_file(exe, ec)) {
        error = "solver executable not found: " + exe.string();
        return false;
    }

    SECURITY_ATTRIBUTES sa{sizeof(SECURITY_ATTRIBUTES), nullptr, TRUE};
    HANDLE in_r = nullptr, out_w = nullptr, err_w = nullptr;
    auto fail = [&](const char* what) {
        error = last_error_text(what);
        for (HANDLE h : {in_r, out_w, err_w})
            if (h) CloseHandle(h);
        p_->shutdown();
        return false;
    };
    if (!CreatePipe(&in_r, &p_->in_w, &sa, 0)) return fail("CreatePipe(stdin)");
    if (!CreatePipe(&p_->out_r, &out_w, &sa, 0)) return fail("CreatePipe(stdout)");
    if (!CreatePipe(&p_->err_r, &err_w, &sa, 0)) return fail("CreatePipe(stderr)");
    // Our ends must not leak into the child (or it would hold its own pipes open).
    for (HANDLE h : {p_->in_w, p_->out_r, p_->err_r})
        SetHandleInformation(h, HANDLE_FLAG_INHERIT, 0);

    // Job object: the child dies with the simulator, even on a crash.
    p_->job = CreateJobObjectW(nullptr, nullptr);
    if (p_->job) {
        JOBOBJECT_EXTENDED_LIMIT_INFORMATION li{};
        li.BasicLimitInformation.LimitFlags = JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE;
        SetInformationJobObject(p_->job, JobObjectExtendedLimitInformation, &li, sizeof li);
    }

    std::wstring cmd = quote_arg(exe.wstring());
    for (const auto& a : args) cmd += L" " + quote_arg(widen(a));
    std::filesystem::path dir = cwd.empty() ? exe.parent_path() : cwd;

    STARTUPINFOW si{};
    si.cb = sizeof si;
    si.dwFlags = STARTF_USESTDHANDLES;
    si.hStdInput = in_r;
    si.hStdOutput = out_w;
    si.hStdError = err_w;
    PROCESS_INFORMATION pi{};
    if (!CreateProcessW(nullptr, cmd.data(), nullptr, nullptr, TRUE,
                        CREATE_NO_WINDOW | CREATE_SUSPENDED, nullptr,
                        dir.empty() ? nullptr : dir.c_str(), &si, &pi))
        return fail("CreateProcess");
    if (p_->job) AssignProcessToJobObject(p_->job, pi.hProcess);
    ResumeThread(pi.hThread);
    CloseHandle(pi.hThread);
    p_->process = pi.hProcess;
    // The child owns these now.
    CloseHandle(in_r);
    CloseHandle(out_w);
    CloseHandle(err_w);

    p_->out_thread = std::thread(reader_loop, p_->out_r, &p_->out);
    p_->err_thread = std::thread(reader_loop, p_->err_r, &p_->err);
    error.clear();
    return true;
}

bool ChildProcess::write(std::string_view data) {
    if (!p_->in_w) return false;
    while (!data.empty()) {
        DWORD wrote = 0;
        if (!WriteFile(p_->in_w, data.data(), static_cast<DWORD>(data.size()), &wrote, nullptr)) {
            p_->close(p_->in_w);
            return false;
        }
        data.remove_prefix(wrote);
    }
    return true;
}

std::optional<std::string> ChildProcess::read_line(std::chrono::milliseconds timeout) {
    LineQueue& q = p_->out;
    std::unique_lock<std::mutex> lk(q.m);
    // No wait_for() when nothing needs waiting: on MinGW even a 0 ms wait sleeps a timer slice.
    const auto ready = [&] { return !q.lines.empty() || q.eof; };
    if (!ready() && timeout.count() > 0) q.cv.wait_for(lk, timeout, ready);
    if (q.lines.empty()) return std::nullopt;
    std::string l = std::move(q.lines.front());
    q.lines.pop_front();
    return l;
}

bool ChildProcess::eof() const {
    std::lock_guard<std::mutex> lk(p_->out.m);
    return p_->out.eof && p_->out.lines.empty();
}

std::vector<std::string> ChildProcess::take_stderr_lines() {
    std::lock_guard<std::mutex> lk(p_->err.m);
    std::vector<std::string> v(std::make_move_iterator(p_->err.lines.begin()),
                               std::make_move_iterator(p_->err.lines.end()));
    p_->err.lines.clear();
    return v;
}

bool ChildProcess::running() const {
    p_->poll_exit();
    return p_->process && !p_->exit_code;
}

bool ChildProcess::wait(std::chrono::milliseconds timeout) {
    if (!p_->process) return true;
    if (WaitForSingleObject(p_->process, static_cast<DWORD>(timeout.count())) != WAIT_OBJECT_0)
        return false;
    p_->poll_exit();
    // Let the readers drain the last bytes.
    if (p_->out_thread.joinable()) p_->out_thread.join();
    if (p_->err_thread.joinable()) p_->err_thread.join();
    return true;
}

std::optional<int> ChildProcess::exit_code() const {
    p_->poll_exit();
    return p_->exit_code;
}

void ChildProcess::kill() {
    if (!p_->process) return;
    p_->poll_exit();
    if (!p_->exit_code) {
        TerminateProcess(p_->process, 1);
        WaitForSingleObject(p_->process, 5000);
    }
    p_->poll_exit();
    p_->close(p_->in_w);
    if (p_->out_thread.joinable()) p_->out_thread.join();
    if (p_->err_thread.joinable()) p_->err_thread.join();
}

}  // namespace mmsim::platform
