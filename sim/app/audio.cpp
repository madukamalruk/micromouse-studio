#include "sim/app/audio.hpp"

#include <SDL.h>

#include <cmath>
#include <vector>

namespace mmsim::app {

namespace {
constexpr int kRate = 44100;

void tone(std::vector<float>& out, double f0, double f1, double dur_s, double amp) {
    const int n = static_cast<int>(dur_s * kRate);
    double phase = 0.0;
    for (int i = 0; i < n; ++i) {
        const double t = static_cast<double>(i) / n;
        const double f = f0 + (f1 - f0) * t;
        phase += 2.0 * 3.14159265358979323846 * f / kRate;
        const double env =
            std::min(1.0, t * 40.0) * std::pow(1.0 - t, 2.0);  // fast attack, ease-out
        out.push_back(static_cast<float>(amp * env * std::sin(phase)));
    }
}
}  // namespace

bool Audio::init(bool enabled) {
    if (!enabled) return false;
    if (SDL_InitSubSystem(SDL_INIT_AUDIO) != 0) return false;
    SDL_AudioSpec want{}, have{};
    want.freq = kRate;
    want.format = AUDIO_F32SYS;
    want.channels = 1;
    want.samples = 1024;
    dev_ = SDL_OpenAudioDevice(nullptr, 0, &want, &have, 0);
    if (dev_ == 0) return false;
    SDL_PauseAudioDevice(dev_, 0);
    return true;
}

void Audio::shutdown() {
    if (dev_ != 0) SDL_CloseAudioDevice(dev_);
    dev_ = 0;
}

void Audio::play(Cue c) {
    if (dev_ == 0 || muted_) return;
    if (SDL_GetQueuedAudioSize(dev_) > kRate * sizeof(float)) return;  // never pile up
    std::vector<float> s;
    switch (c) {
        case Cue::Goal:
            tone(s, 660, 660, 0.09, 0.22);
            tone(s, 880, 880, 0.09, 0.22);
            tone(s, 1320, 1320, 0.22, 0.20);
            break;
        case Cue::Done:
            tone(s, 880, 1320, 0.25, 0.20);
            break;
        case Cue::Crash:
            tone(s, 180, 90, 0.18, 0.30);
            break;
        case Cue::Tick:  // race timer: a soft, short click in the last ten seconds
            tone(s, 1000, 1000, 0.035, 0.10);
            break;
        case Cue::TimeUp:
            tone(s, 440, 440, 0.20, 0.25);
            tone(s, 330, 330, 0.35, 0.25);
            break;
    }
    SDL_QueueAudio(dev_, s.data(), static_cast<Uint32>(s.size() * sizeof(float)));
}

}  // namespace mmsim::app
