#include <atomic>
#include <cstdio>
#include <exception>
#include <future>
#include <mutex>
#include <string>
#include <thread>

#include "sim/host/session.hpp"

namespace mmsim::host {

namespace {
std::atomic<int> g_leaked{0};
std::mutex g_error_mutex;
std::string g_last_error;

void set_last_error(std::string msg) {
    std::lock_guard<std::mutex> lk(g_error_mutex);
    g_last_error = std::move(msg);
}

/// State shared between the caller and the worker so a late finisher can tell it was abandoned.
struct Shared {
    std::promise<void> done;
    std::atomic<int> state{0};  // 0 running, 1 finished, 2 abandoned by the caller
};
}  // namespace

int guard_leaked_threads() {
    return g_leaked.load();
}

std::string guard_last_error() {
    std::lock_guard<std::mutex> lk(g_error_mutex);
    return g_last_error;
}

GuardResult run_guarded(std::shared_ptr<Session> session, std::function<void(Session&)> work,
                        std::chrono::milliseconds wall_timeout) {
    auto shared = std::make_shared<Shared>();
    auto finished = shared->done.get_future();
    auto faulted = std::make_shared<std::atomic<bool>>(false);
    std::thread worker([session, shared, faulted, work = std::move(work)] {
        try {
            work(*session);
        } catch (const std::exception& e) {
            set_last_error(e.what());
            faulted->store(true);
        } catch (...) {
            set_last_error("unknown exception in the simulation step");
            faulted->store(true);
        }
        // Always signal, so a throwing worker never leaves the caller waiting for the timeout.
        shared->done.set_value();
        if (shared->state.exchange(1) == 2)
            g_leaked.fetch_sub(1);  // finished after being abandoned
    });
    const bool in_time = finished.wait_for(wall_timeout) == std::future_status::ready;
    if (in_time || shared->state.exchange(2) == 1) {
        worker.join();
        return faulted->load() ? GuardResult::Faulted : GuardResult::Completed;
    }
    // The firmware is spinning on the worker's fiber; there is no safe way to stop it.
    // Leak the thread (and, through the captured shared_ptr, the session and the work).
    g_leaked.fetch_add(1);
    std::fprintf(stderr,
                 "[guard] firmware did not return within %lld ms: worker abandoned "
                 "(%d leaked thread(s) still spinning)\n",
                 static_cast<long long>(wall_timeout.count()), g_leaked.load());
    worker.detach();
    return GuardResult::Hung;
}

GuardResult run_guarded(std::shared_ptr<Session> session, std::uint32_t ms,
                        std::chrono::milliseconds wall_timeout) {
    return run_guarded(std::move(session), [ms](Session& s) { s.run_ms(ms); }, wall_timeout);
}

}  // namespace mmsim::host
