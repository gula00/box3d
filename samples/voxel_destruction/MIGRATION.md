# Unity VoxelEngine migration map

This document maps `Kugelhaufen/VoxelEngine` to the native Box3D/Sokol
implementation. Equivalent responsibilities are mapped even when Unity
`MonoBehaviour`, `ScriptableObject`, or editor-window classes have no literal
native counterpart.

## Core runtime

| Unity code | Native implementation | State |
| --- | --- | --- |
| `VoxelData`, `VoxelMap`, `ArrayLooper3D` | `voxel_types.h` | Filled state, material, RGBA, 3D bounds/index conversion, cell centers. |
| `Chunk`, `VoxelChunkMeshCreator`, `MeshCreatorJob`, `VoxelMeshLookupTable` | `voxel_chunk_mesh.*` | 16-cubed chunks, exposed-face meshes, neighbor dirtying, worker CPU builds, main-thread GPU registration, arbitrary RGBA grouping. |
| `BoxColliderOptimisationJob` | `voxel_collider.*` | Worker-safe greedy box covering. |
| `VoxelObjPhysicsManager` | `VoxelBody` | Dynamic pieces use greedy box hulls; static objects use per-chunk triangle-mesh colliders. Each voxel has the Unity mass of 0.01; physics enable/disable and kinematic velocity preservation are available at runtime. |
| `VoxelObj`, `VoxelObjHolder` | `voxel_body.*` | Voxel map, Box3D body, chunk mesh cache, colliders, ID/version/update state, arbitrary cell edits, dirty chunk queries, coordinate conversion and rendering. |
| `JobCallbackManager`, `CounterInterlocked` | `VoxelJobSystem`, `VoxelBodyUpdateManager` | Worker pool, main-thread completions, queued next updates, stale-version reruns and cancellation. |
| `MultibleVoxelObjUpdater` | `VoxelBodyUpdateManager::RequestUpdates` | Aggregate multi-object completion. |
| `StandardVoxelReplaceUpdater` | `VoxelReplaceMode::standard` | Existing body is removed while new bodies are built over subsequent frames. |
| `AntiFlickerVoxelReplaceUpdater` | `VoxelReplaceMode::antiFlicker` | Existing body stays visible while disabled new bodies are built, followed by an atomic reveal/swap. |
| Immediate update handlers | `VoxelReplaceMode::immediate`, immediate body update | Same-frame CPU update and replacement. |
| `SerializedVoxelMap` | `voxel_serialization.*`, `voxel_deflate.*` | Native `B3VX` RLE plus direct Unity YAML `.asset` import and export using the package script GUID. The private five-byte Filled/R/G/B/A Deflate payload supports stored, fixed, and dynamic Huffman blocks. |
| `VoxelObjStartInitializer` | `InitializeSingleVoxelBody`, reset and load paths | Immediate or worker-backed initialization, optional initial physics, procedural maps, `.b3vox`, Unity `.asset`, and mesh-derived initialization are available from native UI/code. |
| `VoxelObjVoxelCountDestroyer` | minimum fragment filtering and object budget | Undersized pieces are discarded and old small debris can be removed under budget pressure. |
| Preview mesh creator | `VoxelChunkMeshCache` | The same cached surface mesh is used for preview and runtime rendering. |

## Connected components

| Unity code | Native implementation | State |
| --- | --- | --- |
| PI first pass and Z-line offsets | `voxel_connectivity.cpp` | Uninterrupted Z runs receive initial labels. |
| label-equivalence and `UnionFindJob` | `voxel_connectivity.cpp` | X/Y neighboring runs are unioned and relabeled into six-neighbor components. |
| `BlobExtractor`, `PiBlobExtractor` | `BuildVoxelPiece` | Components are cropped into local maps on a worker. Collider runs and CPU chunk meshes are precomputed with each piece. |
| `SimpleGravityBlobAnalysisJob`, gravity extractor | `FindMostGroundedComponent` | Selects the component with the most support in the first occupied Y layer. |
| `ConnectedComponentPhysics` | fracture worker plus body/replace update managers | Version-checked extraction, body creation, update policies and completion callbacks. |
| `VoxelObjAmountManager` | `ManageVoxelObjectBudget` | 3500-box default budget, hysteresis, managed-size threshold and oldest-small-fragment removal. |

The CCL request runs off the main thread. Its Z-line first pass, label-offset
application, and equivalence-pair generation are also internally parallel;
Union-Find merging stays deterministic and sequential.

## Voronoi runtime

| Unity code | Native implementation | State |
| --- | --- | --- |
| `VoxelMapSphereCutter`, `SphereVoroniFracturer` | `CutVoxelSphere`, `CopyVoxelSphere`, `CutAndCopyVoxelSphere`, fracture worker | Impact-local spherical selection plus copy-only, cut-only and cut-and-copy map operations. |
| `SeedGenerator` | deterministic sphere rejection sampling and `GenerateVoxelSeedsInBox` | Configurable seed count/radius and filled-voxel probability sampling in a bounded box. |
| `VoroniLabelMapGeneratorJob`, `VoroniFracturer` | `GenerateVoxelVoronoiGroups`, `BuildVoxelPiece` | Nearest-seed labels and cropped fragments. |
| `VoxelMapEdgeDestroyerJob`, `VoxelEdgeDestroyer` | `DestroyVoxelFragmentEdges` | Same six-neighbor `> 3` survival rule. |
| `OuterLayerDestroyJob` | `DestroyVoxelOuterLayer` | Removes a voxel when any six-neighbor is empty or out of bounds. |
| `VoroniVoxelObjUpdaterSettings` | sample controls | Radius, seed count, minimum size, erosion and replacement strategy are selectable. |
| Full/update handlers | `VoxelBodyUpdateManager`, `VoxelReplaceUpdater` | Async, immediate, connected-component and aggregate update responsibilities are native modules. |

## Editor-tool replacements

| Unity editor tool | Native implementation |
| --- | --- |
| `VoxelMapResizer` | `ResizeVoxelMapNearest` plus asynchronous resize UI |
| `HollowVoxelObj` | `HollowVoxelMap` plus thickness control and async scene update |
| `Voxelizer` | `VoxelizeTriangleMesh` and `LoadObjVoxelTriangleMesh` |
| `MeshColliderTool` | Not needed: OBJ triangles are loaded directly and static chunk mesh colliders are generated automatically. |
| inspector/preview windows | ImGui controls and the live cached voxel mesh |

The OBJ path accepts scale, Z-up conversion, and scan voxel size. Mesh
voxelization marks surface cells, fills closed interiors, reads real MTL
material assignments, and transfers diffuse colors or UV-interpolated diffuse
texture pixels. Windows texture decoding uses WIC for PNG, JPEG, BMP, TIFF and
other installed codecs.

The sample can also assemble a configurable multi-building scene. Every
building is an independent voxel object with internal 16-cubed chunks, hit
events, automatic post-edit CCL, repeated fracture, and shared scene-level
collider budgeting.

## Demo script replacements

| Unity demo script | Native implementation |
| --- | --- |
| `PlayerController` | Box3D sample orbit/pan/zoom camera and camera pick rays. |
| `WeaponManager` | `Q` and the ImGui weapon-profile selector cycle the Box3D sample ball plus the Unity Small/Big prefab profiles. |
| `StandardWeapon` | `LaunchWreckingBall` spawns a bullet body from the camera ray with the selected radius, speed and mass. |
| `FreeWreckingBall` | Speed-scaled radius/seed region, target-size guard, kinematic hold during fracture and post-fracture velocity restoration. |
| `AutoGameObjDestroyer` | Per-projectile birth step and profile lifetime cleanup. |

The Unity Small/Big profiles use their prefab values: radius 0.5, speeds
40/50, masses 100/300, fracture radii 10/30, seed counts 30/40, target
threshold 500, 0.9 post-fracture velocity and a three-second lifetime. The
Box3D sample-style Shift+right-click sphere remains the default as requested.

## Intentional native differences

- Connectivity is represented by one Box3D rigid body per component, not voxel
  joints.
- Dynamic fragments inherit source point velocity and angular velocity and
  receive an impact-directed kick. Unity VoxelEngine does not explicitly
  transfer all of this momentum.
- The native wrecking ball follows `FreeWreckingBall`: remaining speed scales
  the radius and seed region, the projectile is held kinematic during the
  asynchronous fracture, resumes at 0.8 velocity, and expires after five
  seconds.
- Rendering is Sokol rasterization; physics is Box3D. No Unity GameObject,
  prefab, asset database, or ScriptableObject is required at runtime.
- Native saves use `B3VX` when material indices must be retained. Unity
  `SerializedVoxelMap` YAML assets can be loaded and written directly; that
  format intentionally contains only Filled/R/G/B/A because the Unity class has
  no material-index field.

## Verification

The `BOX3D_VOXEL_SELF_TEST` path covers resizing, hollowing, collider merging,
CCL, grounded selection, sphere copy/cut/Voronoi/outer-layer logic,
texture-sampled OBJ/MTL voxelization, native serialization, Unity YAML asset
import/export, and stored/dynamic Deflate payloads.

Runtime validation paths separately cover:

- queued/stale asynchronous voxel-map updates;
- automatic connected-component extraction after an ordinary voxel-map update;
- Immediate, Standard, and Anti-flicker replacement;
- immediate CCL/fracture computation;
- direct fracture;
- a real bullet sphere hitting a static chunk mesh collider;
- Unity Small/Big projectile profile values, mass override and real Small-ball fracture;
- physics enable/disable and kinematic velocity restoration;
- immediate and asynchronous start initialization;
- asynchronous resize;
- OBJ import and voxelization.
- multi-object large-scene construction and repeated-fracture stress.

## Runtime optimizations

- Filled-voxel counts are maintained incrementally, making UI totals,
  fracture thresholds, stress selection, and object-budget comparisons O(1)
  per body.
- A voxel edit marks only its owning 16-cubed chunk and a boundary neighbor
  when required. Worker jobs rebuild only those dirty render chunks.
- Static triangle colliders and dynamic greedy-box colliders are owned per
  chunk after the first edit, allowing later edits to replace only affected
  Box3D shapes while preserving body velocity and updating mass once.
- Render surfaces use exact-color greedy quad merging. A flat region emits one
  quad instead of one quad per exposed voxel.
- OBJ voxelization uses a median-split triangle BVH for closest-surface
  queries, followed by exterior flood fill for solid interior detection. This
  replaces the former full triangle scan and six parity rays per voxel.
- Dynamic fragments below an independent, configurable cleanup voxel threshold
  are removed after remaining outside the camera frustum for a configurable
  grace period. Replacement batches no longer pause unrelated cleanup; bodies
  owned by a replacement batch and bodies with asynchronous work in flight
  remain protected. Collider-budget and offscreen deletion now share a
  per-frame destruction budget and run only after contact events are consumed,
  avoiding stale shape/body references and large one-frame destruction spikes.
  The HUD reports small, protected, eligible, outside, waiting, and removed body
  counts. Cleanup pauses while the app is paused/minimized.
- Debug GJK overlap validation uses a witness-point tolerance scaled to the
  local point magnitude instead of a single absolute `FLT_EPSILON`. This avoids
  false breakpoint exits during dense, high-speed sphere/voxel contacts without
  changing the Release collision result.
- Dynamic-tree proxy removal recomputes the ancestor `b3_enlargedNode` flag from
  the surviving children. Removing voxel colliders can therefore no longer
  leave a stale enlarged marker that trips validation on the next physics step.
