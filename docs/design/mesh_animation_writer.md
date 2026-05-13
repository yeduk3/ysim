# Design — `MeshAnimationWriter` template-based contract

> Status: Design (pre-implementation). Author: human-Planner conversation 2026-05-13. First implementing slice: C-1 (FlatBuffers backend). Future implementing slice: C-3 (Alembic backend for Unreal Geometry Cache compatibility).
>
> Resolves Q5 (Alembic schema specifics). Closes BDD-013 in stages.

## Why this is a contract, not a virtual interface

ysim is a C++17 simulation engine that template-parameterizes the hot path (`MeshState<BE, PR>`, `Scene<BE, PR>`, `Simulator<BE, PR, System>`). The user's directive on 2026-05-13: **maximize compile-time efficiency** — no virtual dispatch on the inner per-frame loop. The mesh-animation writer follows the same pattern as the existing template family: it is a **duck-typed contract** that any backend type can satisfy, consumed via templates so the compiler monomorphizes the call graph.

Concepts (C++20) are not available; C++17 enforces the contract via "method not found" template-instantiation errors at the call site. This is sufficient because backends are added rarely and reviewed by a human.

## The contract

A type `W` is a valid `MeshAnimationWriter` if it provides:

```cpp
struct MeshAnimationWriterContract {  // Documentation-only struct; not compiled.
    bool open(const std::string& path);

    // One-time topology + frame-range declaration.
    bool writeTopology(const ysim::mesh_cache::TopologyHeader& topo);

    // Per-material-slot face index assignments. Empty vector = no FaceSets
    // (single default material slot). Backend is free to no-op this on
    // formats that don't support material slots.
    bool writeFaceSets(const std::vector<ysim::mesh_cache::FaceSetAssignment>& assignments);

    // Per-frame data. Frame indices must be monotonic from
    // TopologyHeader::start_frame.
    bool writeFrame(int32_t frame_index, const std::vector<tinym::vec3>& positions);
    bool writeFrameNormals(int32_t frame_index, const std::vector<tinym::vec3>& normals);
    bool writeFrameVelocities(int32_t frame_index, const std::vector<tinym::vec3>& velocities);

    bool close();
    const char* backendName() const;
};
```

## Shared POD types

These live in `include/MeshCacheTypes.hpp` and are included by every backend AND by the consumer:

```cpp
namespace ysim::mesh_cache {

struct TopologyHeader {
    std::string track_name;
    std::vector<uint32_t> triangle_indices;   // 3 * triangleCount
    std::vector<int32_t> face_counts;          // all 3s for tri mesh
    float fps;
    int32_t start_frame;
    int32_t end_frame;
    std::vector<tinym::vec3> rest_normals;     // optional; per-vertex
    std::vector<tinym::vec3> rest_uvs;         // optional; per-face-vertex (length = 3 * triCount); .x = u, .y = v, .z unused
    std::vector<std::string> material_slot_names;
};

struct FaceSetAssignment {
    std::string material_slot_name;
    std::vector<int32_t> face_indices;
};

}  // namespace ysim::mesh_cache
```

## Consumer pattern (caller-side)

The export entry point is a function template:

```cpp
template <typename Writer>
bool exportMeshAnimation(Writer& writer,
                         const ysim::mesh_cache::TopologyHeader& topo,
                         const std::vector<FaceSetAssignment>& face_sets,
                         /* per-frame source — abstracted */) {
    if (!writer.open(/* path passed elsewhere */)) return false;
    if (!writer.writeTopology(topo)) return false;
    if (!writer.writeFaceSets(face_sets)) return false;
    for (int f = topo.start_frame; f <= topo.end_frame; ++f) {
        /* extract positions / normals / velocities for frame f from the
           simulator */
        if (!writer.writeFrame(f, positions_f)) return false;
        if (!normals_f.empty() && !writer.writeFrameNormals(f, normals_f)) return false;
        if (!velocities_f.empty() && !writer.writeFrameVelocities(f, velocities_f)) return false;
    }
    return writer.close();
}
```

The compiler monomorphizes one copy per backend type used in the program. There is no virtual call inside the per-frame loop.

## Runtime backend selection

The user-facing CLI flag (`--mesh-cache-format=flatbuffers|alembic`) picks the backend type at startup. Selection uses `std::variant<FlatBuffersMeshAnimationWriter, AlembicMeshAnimationWriter, ...>` + `std::visit`:

```cpp
using MeshAnimationWriterVariant = std::variant<
    FlatBuffersMeshAnimationWriter
    // AlembicMeshAnimationWriter appended after Stage C-3.
    >;

MeshAnimationWriterVariant pickWriter(const std::string& backendName) {
    if (backendName == "flatbuffers") return FlatBuffersMeshAnimationWriter{};
    // if (backendName == "alembic") return AlembicMeshAnimationWriter{};
    throw std::runtime_error("unknown writer backend: " + backendName);
}

int main(int argc, char** argv) {
    auto writer_v = pickWriter(parseBackendFlag(argc, argv));
    std::visit([&](auto& writer) {
        exportMeshAnimation(writer, /* ... */);
    }, writer_v);
}
```

`std::visit` does a single dispatch at the entry point (one switch on the variant's type tag). The compiler emits one monomorphic copy of `exportMeshAnimation` per variant alternative; **inside the lambda, every method call on `writer` is non-virtual and inlinable.** Zero overhead on the per-frame hot loop.

If a future user wants build-time selection only (smaller binary, single backend), that is achievable by skipping the variant and instantiating the chosen backend directly. The contract works either way.

## Stage C-1 — FlatBuffers backend (first implementation)

**Library**: google/flatbuffers, vendored under `include/flatbuffers/`. Single-header generator + ~10-file runtime; Apache-2.0.

**Schema** (`schemas/mesh_cache.fbs`):

```fbs
namespace ysim.mesh_cache.fb;

struct Vec3 { x: float; y: float; z: float; }
struct Vec2 { u: float; v: float; }

table FaceSet {
    material_slot_name: string;
    face_indices: [int32];
}

table TopologyHeader {
    track_name: string;
    triangle_indices: [uint32];
    face_counts: [int32];
    fps: float;
    start_frame: int32;
    end_frame: int32;
    rest_normals: [Vec3];
    rest_uvs: [Vec2];
    material_slot_names: [string];
    face_sets: [FaceSet];
}

table FramePositions {
    frame_index: int32;
    positions: [Vec3];
    normals: [Vec3];                // empty = not provided this frame
    velocities: [Vec3];             // empty = not provided this frame
}

table MeshCacheFile {
    schema_version: int32 = 1;
    topology: TopologyHeader;
    frames: [FramePositions];
}

root_type MeshCacheFile;
file_identifier "YMC1";
file_extension "ymc";
```

CMake adds the `.fbs` schema as a custom build step (`flatc --cpp ...`) that emits a generated header under `build/include/generated/`. Only the FlatBuffers backend's `.cpp` includes the generated header; the contract header (`MeshCacheTypes.hpp`) stays format-agnostic.

**Backend implementation** (`include/FlatBuffersMeshAnimationWriter.hpp` + `.cpp`):

```cpp
class FlatBuffersMeshAnimationWriter {
    flatbuffers::FlatBufferBuilder fbb_;
    std::ofstream out_;
    // In-memory accumulators; FlatBuffers builds the whole buffer before
    // flush. Memory-bounded; Stage C-4 can add a streaming variant if
    // multi-minute animations become a use case.
    TopologyHeader pending_topology_;
    std::vector<FaceSetAssignment> pending_face_sets_;
    std::vector<std::pair<int32_t, std::vector<tinym::vec3>>> pending_positions_;
    std::vector<std::pair<int32_t, std::vector<tinym::vec3>>> pending_normals_;
    std::vector<std::pair<int32_t, std::vector<tinym::vec3>>> pending_velocities_;

public:
    const char* backendName() const { return "flatbuffers"; }
    bool open(const std::string& path);
    bool writeTopology(const TopologyHeader& topo);
    bool writeFaceSets(const std::vector<FaceSetAssignment>& assignments);
    bool writeFrame(int32_t frame_index, const std::vector<tinym::vec3>& positions);
    bool writeFrameNormals(int32_t frame_index, const std::vector<tinym::vec3>& normals);
    bool writeFrameVelocities(int32_t frame_index, const std::vector<tinym::vec3>& velocities);
    bool close();  // builds the FlatBuffer from pending_* and writes to out_.
};
```

**Harness verification** (Stage C-1 self-test block):
- Build a synthetic 4×4 cloth, pump 10 frames, write through the writer to `/tmp/`.
- Read back via the generated FlatBuffers reader; assert frame count == 10, topology vertex count matches what was written, frame[0] position byte-equality.
- Bug-probe: stub one of the `writeFrame` calls inside the export loop → reader sees `frames.size() != 10` → assertion FAILs.

Self-test count after C-1: 51 → 52 (one new clause).

## Stage C-3 — Alembic backend

Same contract; new type `AlembicMeshAnimationWriter` appended to the variant. Internal mapping:

| Contract call | Alembic operation |
|---|---|
| `open(path)` | `Alembic::Abc::OArchive(Alembic::AbcCoreOgawa::WriteArchive(), path)` |
| `writeTopology(topo)` | Construct `OPolyMesh` under `archive.getTop()`, build `TimeSampling(1.0 / fps, start_frame / fps)`, write **first sample** with positions, indices, counts, optional rest normals + UVs |
| `writeFaceSets(assignments)` | For each assignment, `meshObj.getSchema().createFaceSet(name)` + `OFaceSetSchema::Sample::setFaces(...)` |
| `writeFrame(f, pos)` (f > start_frame) | `OPolyMeshSchema::Sample` with positions only; topology omitted → Alembic infers constant topology, Unreal's "Apply Constant Topology Optimizations" lights up |
| `writeFrameNormals` / `writeFrameVelocities` | `ON3fGeomParam::Sample` / Alembic velocity attribute when motion-blur path lands |
| `close()` | Archive destructor flushes + closes |

**Axis convention**: write Y-up at export (Alembic default, matches Maya/Blender). Unreal's importer's "Convert Scene" option flips on import. Document in the user-facing export guide.

## Standing constraints introduced by this design

- **MESH-CACHE-WRITER-PORTABILITY** (added when C-1 ships): any change to the contract (new method, signature widening) MUST update every backend (FlatBuffers; Alembic when C-3 lands) in the same commit. Documented in `docs/roles/PLANNER.md`'s Standing constraints subsection.

## Open sub-questions

1. **In-memory vs streamed write**: Stage C-1 buffers all frames before flush. Streaming (write each frame immediately) is memory-bounded for long animations. Defer to Stage C-4 (streaming variant) unless a slice hits memory pressure earlier.
2. **Multi-material per mesh**: current `GeneralMesh::material` is a single `Material` struct. FaceSets require per-face material assignment. Stage C-2 territory; design lives in that slice's plan.
3. **Rigid bodies as cache objects**: a rigid body's "positions per frame" derive from `transformPosition` + `rotationQuat` + base mesh vertices. The current shape — bake transform into per-vertex positions — keeps the writer uniform across cloth and rigid. Alembic alternative (`OXform` parent + constant child) is more compact but Unreal's Geometry Cache prefers baked positions. Stage C-3 picks; default plan = baked positions.
