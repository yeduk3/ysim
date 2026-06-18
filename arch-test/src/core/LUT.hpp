#pragma once
#include <string>
#include <unordered_map>
#include <typeindex>

// LUT — a tiny, header-only, GL-free runtime data bus (DECISIONS O1).
//
// A typed name->handle dictionary. The Simulator (or any producer) bind()s
// raw pointers into it under stable string keys; consumers (renderer, gui,
// FrameSnapshot, headless Runner) get<T>() by key EACH FRAME — they cache
// the KEY, never the pointer (invariant a: a rebuild re-points the entry in
// place, so a cached pointer would dangle but a cached key stays valid).
//
// Type safety: each entry stores a std::type_index captured at bind time.
// get<T>() returns null on a type mismatch (never reinterpret-casts blindly).
//
// "render-out handles" and "params" are NOT separate containers — they are
// the same dict, distinguished only by a naming convention ("pos","frame"
// for render-out; "gravity","wind" for params). Lazy by design.
//
// The UpdatePolicy is STORED per entry but NOT enforced this pass (it is a
// hint for a future gui/scheduler: when may a consumer mutate this handle).
struct LUT {
    enum struct UpdatePolicy {
        Live,          // safe to read/write any time, mid-frame included
        RebuildPaused, // only mutate while the sim is paused (topology-ish)
        RestartFrame0, // change takes effect only on a fresh frame-0 restart
    };

    struct Entry {
        void* ptr = nullptr;
        std::type_index type = std::type_index(typeid(void));
        UpdatePolicy policy = UpdatePolicy::Live;
    };

    std::unordered_map<std::string, Entry> table;

    // Register / re-point a handle. Re-binding the same key overwrites the
    // entry in place (invariant a): consumers caching the key see the new
    // pointer on their next get<T>(); a rebuild never dangles a cached key.
    template <typename T>
    void bind(const std::string& name, T* ptr,
              UpdatePolicy policy = UpdatePolicy::Live) {
        table[name] = Entry{ static_cast<void*>(ptr),
                             std::type_index(typeid(T)), policy };
    }

    // Re-lookup by key. Returns T* on a present, type-matching entry; null
    // on a missing key OR a type mismatch (never an unchecked cast).
    template <typename T>
    T* get(const std::string& name) const {
        auto it = table.find(name);
        if (it == table.end()) return nullptr;
        if (it->second.type != std::type_index(typeid(T))) return nullptr;
        return static_cast<T*>(it->second.ptr);
    }

    bool has(const std::string& name) const { return table.count(name) != 0; }
    void unbind(const std::string& name) { table.erase(name); }

    // Policy peek (un-enforced this pass; for a future gui/scheduler).
    UpdatePolicy policyOf(const std::string& name) const {
        auto it = table.find(name);
        return it == table.end() ? UpdatePolicy::Live : it->second.policy;
    }
};
