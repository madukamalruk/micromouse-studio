#include "sim/app/actions.hpp"

#include <algorithm>
#include <cctype>

namespace mmsim::app {

void ActionRegistry::add(Action a) {
    for (auto& e : actions_)
        if (e.name == a.name) {
            e = std::move(a);
            return;
        }
    actions_.push_back(std::move(a));
}

const Action* ActionRegistry::find(std::string_view name) const {
    for (const auto& a : actions_)
        if (a.name == name) return &a;
    return nullptr;
}

std::vector<std::string> ActionRegistry::tokenize(std::string_view line) {
    std::vector<std::string> out;
    std::string cur;
    bool quoted = false, have = false;
    for (char c : line) {
        if (c == '"') {
            quoted = !quoted;
            have = true;
        } else if (!quoted && std::isspace(static_cast<unsigned char>(c))) {
            if (have) out.push_back(cur);
            cur.clear();
            have = false;
        } else {
            cur += c;
            have = true;
        }
    }
    if (have) out.push_back(cur);
    return out;
}

std::string ActionRegistry::dispatch(std::string_view line) {
    auto tok = tokenize(line);
    if (tok.empty()) return "";
    std::string name = tok.front();
    std::transform(name.begin(), name.end(), name.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    tok.erase(tok.begin());
    const Action* a = find(name);
    if (!a) return "Unknown command '" + name + "'. Type help for the list.";
    if (!a->run) return "";
    return a->run(tok);
}

std::vector<std::string> ActionRegistry::complete(std::string_view prefix) const {
    std::vector<std::string> out;
    for (const auto& a : actions_)
        if (a.name.compare(0, prefix.size(), prefix) == 0) out.push_back(a.name);
    std::sort(out.begin(), out.end());
    return out;
}

std::string ActionRegistry::complete_line(std::string_view line) const {
    if (line.find(' ') != std::string_view::npos) return std::string(line);
    const auto m = complete(line);
    if (m.empty()) return std::string(line);
    std::string p = m.front();
    for (const auto& s : m) {
        std::size_t i = 0;
        while (i < p.size() && i < s.size() && p[i] == s[i]) ++i;
        p.resize(i);
    }
    if (m.size() == 1) p += ' ';
    return p.size() >= line.size() ? p : std::string(line);
}

void ActionRegistry::remember(std::string_view line) {
    cursor_ = -1;
    if (line.empty()) return;
    if (!history_.empty() && history_.back() == line) return;
    history_.emplace_back(line);
    if (history_.size() > 200) history_.erase(history_.begin());
}

std::string ActionRegistry::history_step(int dir) {
    if (history_.empty()) return "";
    const int n = static_cast<int>(history_.size());
    if (cursor_ < 0) {
        if (dir > 0) return "";
        cursor_ = n - 1;
    } else {
        cursor_ += dir;
        if (cursor_ >= n) {
            cursor_ = -1;
            return "";
        }
        cursor_ = std::max(cursor_, 0);
    }
    return history_[static_cast<std::size_t>(cursor_)];
}

}  // namespace mmsim::app
