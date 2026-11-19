#include <future>
#include <thread>

#include "sim/host/session.hpp"

namespace mmsim::host {

GuardResult run_guarded(std::shared_ptr<Session> session, std::function<void(Session&)> work,
                        std::chrono::milliseconds wall_timeout) {
    auto done = std::make_shared<std::promise<void>>();
    auto finished = done->get_future();
    std::thread worker([session, done, work = std::move(work)] {
        work(*session);
        done->set_value();
    });
    if (finished.wait_for(wall_timeout) == std::future_status::ready) {
        worker.join();
        return GuardResult::Completed;
    }
    // The firmware is spinning on the worker's fiber; there is no safe way to stop it.
    // Leak the thread (and, through the captured shared_ptr, the session and the work).
    worker.detach();
    return GuardResult::Hung;
}

GuardResult run_guarded(std::shared_ptr<Session> session, std::uint32_t ms,
                        std::chrono::milliseconds wall_timeout) {
    return run_guarded(std::move(session), [ms](Session& s) { s.run_ms(ms); }, wall_timeout);
}

}  // namespace mmsim::host
