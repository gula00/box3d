# Box3D voxel destruction sample

This sample mirrors the core behavior of `Kugelhaufen/VoxelEngine` without Unity:

- voxels use the Unity sample's 0.2 m cell size;
- filled voxels use six-neighbor connectivity;
- one connected component maps to one Box3D body;
- consecutive voxels are greedily merged into box hull shapes;
- a wrecking-ball hit scales its cut region by remaining speed, freezes during
  the asynchronous fracture, partitions with Voronoi seeds, resumes at 0.8
  velocity, and expires after five seconds;
- the remaining structure is relabeled and disconnected components become dynamic bodies;
- no joints are used for voxel-to-voxel connectivity.

Rendering uses cached per-chunk surface meshes with voxel RGBA colors. Dynamic
physics uses greedily merged box hulls; static voxel objects use chunk triangle
mesh colliders.

See [MIGRATION.md](MIGRATION.md) for a Unity-runtime-to-Box3D mapping and the
remaining behavioral differences.

Controls:

- `Shift + right click`: launch the sample-style sphere from the camera;
- `Q`: cycle Box3D sample, Unity small-ball, and Unity big-ball weapon profiles;
- `X`: fracture the center without waiting for a collision;
- `C`: carve the center and run an asynchronous voxel-object update;
- `H`: hollow the first voxel object using the selected wall thickness;
- `R`: restart the sample.

The controls panel also exposes `.b3vox` save/load, Unity `SerializedVoxelMap`
`.asset` import/export, immediate or asynchronous initialization, runtime
physics/kinematic switching, textured OBJ/MTL voxelization, nearest-neighbor
resize, updater strategy, erosion settings, fragment limits, collider
budgeting, and configurable multi-building scene generation.
