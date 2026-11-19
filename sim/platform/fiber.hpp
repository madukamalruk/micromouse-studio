#pragma once

#include <cstddef>

namespace mmsim::platform {

/// Cooperative coroutine on its own stack (Win32 fiber). Used to run firmware fw_main()
/// in lockstep: the firmware yields only inside blocking HAL calls (docs/decisions.md D6).
/// Not thread-safe; may be resumed from different threads, but never concurrently.
class Fiber {
  public:
    using Entry = void (*)(void* arg);

    Fiber(Entry entry, void* arg, std::size_t stack_reserve = 1u << 20);
    ~Fiber();  // deletes the fiber even while suspended mid-function (no unwinding)
    Fiber(const Fiber&) = delete;
    Fiber& operator=(const Fiber&) = delete;

    bool valid() const { return handle_ != nullptr; }
    bool finished() const { return finished_; }

    /// Runs the fiber until it yields or its entry function returns.
    void resume();
    /// Called from inside the running fiber: returns control to whoever resumed it.
    static void yield();
    /// The fiber currently executing on this thread, or nullptr.
    static Fiber* current();

  private:
    static void __stdcall trampoline(void* self);

    void* handle_ = nullptr;
    void* caller_ = nullptr;
    Fiber* prev_ = nullptr;
    Entry entry_;
    void* arg_;
    bool finished_ = false;
};

/// Makes the calling thread able to switch fibers. If it converted the thread, the
/// destructor converts it back. Safe to nest; cheap if the thread already is a fiber.
class ThreadFiberScope {
  public:
    ThreadFiberScope();
    ~ThreadFiberScope();
    ThreadFiberScope(const ThreadFiberScope&) = delete;
    ThreadFiberScope& operator=(const ThreadFiberScope&) = delete;

  private:
    bool converted_ = false;
};

}  // namespace mmsim::platform
