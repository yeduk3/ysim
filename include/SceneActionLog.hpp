#pragma once

// Runtime-only scene action log. Records user-driven scene edits
// (object add/remove, point pin / reference-point set, scene
// save/load) so the user can review what they did this session.
//
// Deliberately NOT serialized into the scene file: this is a
// volatile in-memory ring buffer that lives only for the process
// lifetime. A fresh launch starts with an empty log; loadScene does
// not restore it (the load itself is logged as an action instead).
//
// Single-threaded by contract: every logging site and the ImGui
// draw run on the main GUI/sim thread, so no locking is needed —
// same threading model as FrameProfiler / the inspector windows.

#include <chrono>
#include <cstdio>
#include <ctime>
#include <deque>
#include <string>

namespace scene_log {

enum class Category {
    Object,      // 오브젝트 추가/제거
    Constraint,  // 점 고정 / 참조점 설정
    SceneIO      // 씬 저장 / 불러오기
};

struct Entry {
    std::string time;     // wall-clock HH:MM:SS when the action happened
    std::string message;  // human-readable, Korean (matches the UI language)
    Category    category = Category::Object;
    bool        ok = true; // false → failure; drawn in red
};

class SceneActionLog {
public:
    static SceneActionLog& instance() {
        static SceneActionLog inst;
        return inst;
    }

    void log(Category cat, std::string message, bool ok = true) {
        Entry e;
        e.time     = nowHms();
        e.message  = std::move(message);
        e.category = cat;
        e.ok       = ok;
        entries_.push_back(std::move(e));
        // Bounded ring buffer — a long session must not grow without
        // limit. Oldest entries fall off the front.
        if (entries_.size() > kMaxEntries) entries_.pop_front();
        scrollPending_ = true;
    }

    void clear() { entries_.clear(); }

    const std::deque<Entry>& entries() const { return entries_; }

    // Consumed by the window to auto-scroll to the newest line exactly
    // once after each append (so a user scrolled up to read history is
    // not yanked back down every frame, only when something new lands).
    bool takeScrollPending() {
        bool s = scrollPending_;
        scrollPending_ = false;
        return s;
    }

private:
    static std::string nowHms() {
        auto now = std::chrono::system_clock::now();
        std::time_t t = std::chrono::system_clock::to_time_t(now);
        std::tm lt{};
#if defined(_WIN32)
        localtime_s(&lt, &t);
#else
        localtime_r(&t, &lt);
#endif
        char buf[16];
        std::strftime(buf, sizeof(buf), "%H:%M:%S", &lt);
        return buf;
    }

    static constexpr std::size_t kMaxEntries = 1000;
    std::deque<Entry> entries_;
    bool scrollPending_ = false;
};

// Convenience shims so call sites read as one short line.
inline void logObject(const std::string& m, bool ok = true) {
    SceneActionLog::instance().log(Category::Object, m, ok);
}
inline void logConstraint(const std::string& m, bool ok = true) {
    SceneActionLog::instance().log(Category::Constraint, m, ok);
}
inline void logSceneIO(const std::string& m, bool ok = true) {
    SceneActionLog::instance().log(Category::SceneIO, m, ok);
}

struct SceneActionLogWindowState {
    // Opt-in via the "보기" menu, same convention as the profiler
    // window. drawSceneActionLogWindow early-returns when false, so
    // an unconditional call site costs nothing.
    bool open = false;
    bool autoScroll = true;
};

void drawSceneActionLogWindow(SceneActionLogWindowState& state);

} // namespace scene_log
