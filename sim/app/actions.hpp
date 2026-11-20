#pragma once
// One action table behind every input path (Slice 9.10): keyboard shortcuts, the backtick
// console, buttons and the "?" help modal all dispatch the same named actions, so a
// shortcut and its console command can never drift apart. Pure (no ImGui), unit-tested.

#include <functional>
#include <string>
#include <string_view>
#include <vector>

namespace mmsim::app {

struct Action {
    std::string name;   // console command, e.g. "speed"
    std::string keys;   // shortcut shown in help, e.g. "Space" ("" = console only)
    std::string group;  // help section: "Run", "View", "Workspaces", ...
    std::string help;   // one line
    std::string usage;  // argument hint, e.g. "<1|2|5|fast>"
    /// Returns a short result line for the console / toast ("" = nothing to say).
    std::function<std::string(const std::vector<std::string>& args)> run;
};

class ActionRegistry {
  public:
    void add(Action a);
    const Action* find(std::string_view name) const;
    const std::vector<Action>& actions() const { return actions_; }

    /// Runs one command line ("speed 5", "ws studio"). Unknown or empty -> message, no
    /// action. Lines are trimmed; quotes group words ("maze \"uk 1987\"").
    std::string dispatch(std::string_view line);
    /// Runs a named action with no arguments (shortcut path).
    std::string trigger(std::string_view name) { return dispatch(name); }

    /// Command names starting with `prefix`, sorted.
    std::vector<std::string> complete(std::string_view prefix) const;
    /// Longest common prefix of complete(prefix) (Tab completion).
    std::string complete_line(std::string_view line) const;

    /// Console history (oldest first; duplicates of the last entry are not stored).
    const std::vector<std::string>& history() const { return history_; }
    void remember(std::string_view line);
    /// Walk history: dir -1 = older, +1 = newer. Returns "" past the newest entry.
    std::string history_step(int dir);

    static std::vector<std::string> tokenize(std::string_view line);

  private:
    std::vector<Action> actions_;
    std::vector<std::string> history_;
    int cursor_ = -1;  // -1 = not browsing
};

}  // namespace mmsim::app
