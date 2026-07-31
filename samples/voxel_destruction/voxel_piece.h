// SPDX-FileCopyrightText: 2026
// SPDX-License-Identifier: MIT

#pragma once

#include "voxel_destruction/voxel_body.h"

struct VoxelSourceSnapshot
{
	Int3 dimensions;
	std::vector<VoxelCell> cells;
	b3Pos position;
	b3Quat rotation;
	b3Vec3 linearVelocity;
	b3Vec3 angularVelocity;
	bool dynamic;
};

// Crops a component into a new local voxel map and precomputes its collider
// boxes. This function is pure and safe to run on a worker thread.
VoxelPieceBuild BuildVoxelPiece( const VoxelSourceSnapshot& source, const std::vector<int>& indices, bool dynamic,
								 b3Vec3 localKick );
