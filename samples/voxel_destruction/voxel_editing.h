// SPDX-FileCopyrightText: 2026
// SPDX-License-Identifier: MIT

#pragma once

#include "voxel_destruction/voxel_types.h"

#include <vector>

// Nearest-neighbor equivalent of Unity EditorTools.VoxelMapResizer.
std::vector<VoxelCell> ResizeVoxelMapNearest( const std::vector<VoxelCell>& source, Int3 sourceDimensions,
											  Int3 targetDimensions );

// Keeps the requested number of filled surface layers and clears the interior.
// Equivalent to the Unity HollowVoxelObj editor tool.
std::vector<VoxelCell> HollowVoxelMap( const std::vector<VoxelCell>& source, Int3 dimensions, int wallThickness );
