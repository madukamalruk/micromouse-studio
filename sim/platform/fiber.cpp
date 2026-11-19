#include "sim/platform/fiber.hpp"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

namespace mmsim::platform {

namespace {
thread_local Fiber* t_current = nullptr;
}

Fiber::Fiber(Entry entry, void* arg, std::size_t stack_reserve) : entry_(entry), arg_(arg) {
    handle_ = CreateFiberEx(64 * 1024, stack_reserve, FIBER_FLAG_FLOAT_SWITCH,
                            reinterpret_cast<LPFIBER_START_ROUTINE>(&Fiber::trampoline), this);
}

Fiber::~Fiber() {
    if (handle_) DeleteFiber(handle_);
}

void __stdcall Fiber::trampoline(void* self) {
    auto* f = static_cast<Fiber*>(self);
    f->entry_(f->arg_);
    f->finished_ = true;
    for (;;) SwitchToFiber(f->caller_);  // a fiber routine must never return
}

void Fiber::resume() {
    if (!handle_ || finished_) return;
    thread_local ThreadFiberScope t_scope;  // converted once per thread, undone at thread exit
    (void)t_scope;
    caller_ = GetCurrentFiber();
    prev_ = t_current;
    t_current = this;
    SwitchToFiber(handle_);
    t_current = prev_;
}

void Fiber::yield() {
    Fiber* f = t_current;
    if (f) SwitchToFiber(f->caller_);
}

Fiber* Fiber::current() {
    return t_current;
}

ThreadFiberScope::ThreadFiberScope() {
    if (!IsThreadAFiber())
        converted_ = ConvertThreadToFiberEx(nullptr, FIBER_FLAG_FLOAT_SWITCH) != nullptr;
}

ThreadFiberScope::~ThreadFiberScope() {
    if (converted_) ConvertFiberToThread();
}

}  // namespace mmsim::platform
