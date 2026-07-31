// SPDX-FileCopyrightText: 2026
// SPDX-License-Identifier: MIT

#pragma once

#include "voxel_destruction/voxel_types.h"

#include <vector>

struct VoxelBoxRun
{
	Int3 lower;
	Int3 upper;
};

struct VoxelChunkColliderBuild
{
	int chunkIndex = 0;
	std::vector<VoxelBoxRun> runs;
};

using VoxelChunkColliderBuildBatch = std::vector<VoxelChunkColliderBuild>;

// Greedily covers every filled voxel exactly once with axis-aligned boxes.
// This is the native equivalent of VoxelEngine's BoxColliderOptimisationJob.
std::vector<VoxelBoxRun> BuildVoxelBoxRuns( const std::vector<VoxelCell>& cells, Int3 dimensions );
VoxelChunkColliderBuildBatch BuildVoxelChunkBoxRuns( const std::vector<VoxelCell>& cells, Int3 dimensions,
													 const std::vector<int>& chunkIndices );
