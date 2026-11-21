#pragma once
// Muteable audio cues (Slice 9.10): a rising chime when the goal is reached, a low buzz on a
// wall hit. Synthesised tones queued on an SDL audio device; never opened in scripted runs.

namespace mmsim::app {

enum class Cue { Goal, Crash, Done, Tick, TimeUp };

class Audio {
  public:
    /// enabled = false (scripted/headless): no device is opened, play() does nothing.
    bool init(bool enabled);
    void shutdown();
    void play(Cue c);
    void set_muted(bool m) { muted_ = m; }
    bool muted() const { return muted_; }
    bool available() const { return dev_ != 0; }

  private:
    unsigned dev_ = 0;
    bool muted_ = false;
};

}  // namespace mmsim::app
