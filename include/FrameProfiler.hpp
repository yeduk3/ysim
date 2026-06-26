#pragma once

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace profiler {

struct SectionStats {
    double latest_ms = 0.0;
    double average_ms = 0.0;
    double max_ms = 0.0;
};

struct FrameSnapshot {
    uint64_t sequence = 0;
    double wall_time_seconds = 0.0;
    double frame_ms = 0.0;
    double fps = 0.0;
    std::vector<double> section_ms;
    // Per-frame totals summed across substeps. Simulator::update reads the
    // packed collision counters after each substep's narrow phase and feeds
    // them through FrameProfiler::addCollisionCounts. CSV exports the totals;
    // divide by your subStep count to get per-substep means.
    uint64_t broad_collisions = 0;
    uint64_t narrow_collisions = 0;
};

class FrameProfilerHistory {
public:
    explicit FrameProfilerHistory(std::size_t max_frames = 240)
        : max_frames_(max_frames) {}

    std::size_t ensureSection(const std::string& name) {
        auto it = section_lookup_.find(name);
        if (it != section_lookup_.end()) return it->second;

        std::size_t index = section_names_.size();
        section_names_.push_back(name);
        section_lookup_[name] = index;
        return index;
    }

    std::size_t sectionCount() const { return section_names_.size(); }
    const std::vector<std::string>& sectionNames() const { return section_names_; }
    const std::deque<FrameSnapshot>& frames() const { return frames_; }
    bool empty() const { return frames_.empty(); }

    int sectionIndex(const std::string& name) const {
        auto it = section_lookup_.find(name);
        return (it == section_lookup_.end()) ? -1 : static_cast<int>(it->second);
    }

    const FrameSnapshot* latestFrame() const {
        return frames_.empty() ? nullptr : &frames_.back();
    }

    // Drop all accumulated frames (the CSV export rows). Keeps section
    // names/lookup so columns stay stable across re-records. Bound to the
    // '0' reset so each manual measurement starts from an empty log.
    void clearFrames() { frames_.clear(); }

    void push(FrameSnapshot frame) {
        if (frame.section_ms.size() < section_names_.size())
            frame.section_ms.resize(section_names_.size(), 0.0);

        frames_.push_back(std::move(frame));
        while (frames_.size() > max_frames_) frames_.pop_front();
    }

    std::vector<float> makeRecentSeries(int section_index, double history_seconds) const {
        std::vector<float> values;
        if (frames_.empty()) return values;

        double cutoff = frames_.back().wall_time_seconds - history_seconds;
        for (const auto& frame : frames_) {
            if (frame.wall_time_seconds < cutoff) continue;
            if (section_index < 0) values.push_back(static_cast<float>(frame.frame_ms));
            else if (static_cast<std::size_t>(section_index) < frame.section_ms.size())
                values.push_back(static_cast<float>(frame.section_ms[section_index]));
            else
                values.push_back(0.0f);
        }
        return values;
    }

    SectionStats computeRecentStats(std::size_t section_index, double history_seconds) const {
        SectionStats stats;
        if (frames_.empty()) return stats;

        double cutoff = frames_.back().wall_time_seconds - history_seconds;
        double sum = 0.0;
        std::size_t count = 0;

        for (auto it = frames_.rbegin(); it != frames_.rend(); ++it) {
            if (it->wall_time_seconds < cutoff) break;
            double value = section_index < it->section_ms.size() ? it->section_ms[section_index] : 0.0;
            if (count == 0) stats.latest_ms = value;
            stats.max_ms = std::max(stats.max_ms, value);
            sum += value;
            ++count;
        }

        if (count > 0) stats.average_ms = sum / static_cast<double>(count);
        return stats;
    }

    bool exportCsv(const std::string& path) const {
        std::filesystem::path output_path(path);
        if (output_path.has_parent_path()) {
            std::error_code ec;
            std::filesystem::create_directories(output_path.parent_path(), ec);
        }

        std::ofstream file(path);
        if (!file.is_open()) return false;

        file << "frame_sequence,wall_time_seconds,frame_ms,fps,broad_collisions,narrow_collisions";
        for (const auto& name : section_names_) file << ',' << sanitizeHeader(name);
        file << '\n';

        for (const auto& frame : frames_) {
            file << frame.sequence << ','
                 << frame.wall_time_seconds << ','
                 << frame.frame_ms << ','
                 << frame.fps << ','
                 << frame.broad_collisions << ','
                 << frame.narrow_collisions;

            for (std::size_t i = 0; i < section_names_.size(); ++i) {
                double value = i < frame.section_ms.size() ? frame.section_ms[i] : 0.0;
                file << ',' << value;
            }
            file << '\n';
        }
        return true;
    }

private:
    static std::string sanitizeHeader(const std::string& name) {
        std::string out = name;
        for (char& c : out) {
            if (!(std::isalnum(static_cast<unsigned char>(c)) || c == '_')) c = '_';
        }
        return out;
    }

    std::size_t max_frames_ = 240;
    std::vector<std::string> section_names_;
    std::unordered_map<std::string, std::size_t> section_lookup_;
    std::deque<FrameSnapshot> frames_;
};

class FrameProfiler {
public:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    struct ScopedTimer {
        FrameProfiler* profiler = nullptr;
        std::size_t section_index = 0;
        TimePoint start_time;

        ScopedTimer() = default;
        ScopedTimer(FrameProfiler* profiler, std::size_t section_index)
            : profiler(profiler), section_index(section_index), start_time(Clock::now()) {}

        ScopedTimer(const ScopedTimer&) = delete;
        ScopedTimer& operator=(const ScopedTimer&) = delete;

        ScopedTimer(ScopedTimer&& other) noexcept
            : profiler(other.profiler), section_index(other.section_index), start_time(other.start_time) {
            other.profiler = nullptr;
        }

        ScopedTimer& operator=(ScopedTimer&& other) noexcept {
            if (this == &other) return *this;
            finalize();
            profiler = other.profiler;
            section_index = other.section_index;
            start_time = other.start_time;
            other.profiler = nullptr;
            return *this;
        }

        ~ScopedTimer() { finalize(); }

        void finalize() {
            if (!profiler) return;
            auto end_time = Clock::now();
            double ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
            profiler->addSample(section_index, ms);
            profiler = nullptr;
        }
    };

    explicit FrameProfiler(std::size_t max_frames = 240)
        : history_(max_frames) {}

    void beginFrame(uint64_t sequence, double wall_time_seconds) {
        current_sequence_ = sequence;
        current_wall_time_seconds_ = wall_time_seconds;
        current_frame_start_ = Clock::now();
        current_section_ms_.assign(history_.sectionCount(), 0.0);
        current_broad_collisions_ = 0;
        current_narrow_collisions_ = 0;
        frame_open_ = true;
    }

    void addCollisionCounts(uint64_t broad, uint64_t narrow) {
        if (!frame_open_) return;
        current_broad_collisions_ += broad;
        current_narrow_collisions_ += narrow;
    }

    ScopedTimer scoped(const std::string& name) {
        return ScopedTimer(this, history_.ensureSection(name));
    }

    void addSample(const std::string& name, double ms) {
        addSample(history_.ensureSection(name), ms);
    }

    void endFrame() {
        if (!frame_open_) return;

        auto frame_end = Clock::now();
        double frame_ms = std::chrono::duration<double, std::milli>(frame_end - current_frame_start_).count();

        FrameSnapshot snapshot;
        snapshot.sequence = current_sequence_;
        snapshot.wall_time_seconds = current_wall_time_seconds_;
        snapshot.frame_ms = frame_ms;
        snapshot.fps = frame_ms > 0.0 ? 1000.0 / frame_ms : 0.0;
        snapshot.section_ms = current_section_ms_;
        snapshot.broad_collisions = current_broad_collisions_;
        snapshot.narrow_collisions = current_narrow_collisions_;

        history_.push(std::move(snapshot));
        frame_open_ = false;
    }

    FrameProfilerHistory& history() { return history_; }
    const FrameProfilerHistory& history() const { return history_; }

private:
    void addSample(std::size_t section_index, double ms) {
        if (!frame_open_) return;
        if (current_section_ms_.size() <= section_index)
            current_section_ms_.resize(section_index + 1, 0.0);
        current_section_ms_[section_index] += ms;
    }

    FrameProfilerHistory history_;
    std::vector<double> current_section_ms_;
    TimePoint current_frame_start_;
    uint64_t current_sequence_ = 0;
    double current_wall_time_seconds_ = 0.0;
    uint64_t current_broad_collisions_ = 0;
    uint64_t current_narrow_collisions_ = 0;
    bool frame_open_ = false;
};

// RAII guard that bundles the begin/end pair as a single object gated on a
// single predicate. Production and the test harness both construct it with
// the same `collect = !simulator.pause` value so they drive the same
// gate; a future regression in the predicate (e.g., flipping the
// condition) breaks the harness immediately. close() is callable
// explicitly when endFrame() must precede other work (e.g., the window
// title read after the render loop's profile block); the destructor calls
// close() as a safety net. closed_ initial value is !collect so close()
// is a no-op when the gate isn't collecting — endFrame() is never called
// without a paired beginFrame().
class ProfilerFrameGate {
public:
    ProfilerFrameGate(FrameProfiler& profiler, bool collect,
                      uint64_t sequence, double wall_time_seconds)
        : profiler_(profiler), collect_(collect), closed_(!collect) {
        if (collect_) profiler_.beginFrame(sequence, wall_time_seconds);
    }
    ProfilerFrameGate(const ProfilerFrameGate&) = delete;
    ProfilerFrameGate& operator=(const ProfilerFrameGate&) = delete;
    ~ProfilerFrameGate() { close(); }

    void close() {
        if (closed_) return;
        profiler_.endFrame();
        closed_ = true;
    }
    bool collecting() const { return collect_; }

private:
    FrameProfiler& profiler_;
    bool collect_;
    bool closed_;
};

} // namespace profiler
