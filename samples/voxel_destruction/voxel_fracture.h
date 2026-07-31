// SPDX-FileCopyrightText: 2026
// SPDX-License-Identifier: MIT

#pragma once

#include "voxel_destruction/voxel_types.h"

#include "box3d/box3d.h"

#include <cstdint>
#include <vector>

std::vector<int> CutVoxelSphere( const std::vector<VoxelCell>& cells, Int3 dimensions, b3Vec3 center, float radius );

struct VoxelSphereRegion
{
	Int3 dimensions = {};
	Int3 sourceOffset = {};
	std::vector<VoxelCell> cells;
};

VoxelSphereRegion CopyVoxelSphere( const std::vector<VoxelCell>& cells, Int3 dimensions, Int3 center, int radius );
VoxelSphereRegion CutAndCopyVoxelSphere( std::vector<VoxelCell>* cells, Int3 dimensions, Int3 center, int radius );
int EraseVoxelSphere( std::vector<VoxelCell>* cells, Int3 dimensions, Int3 center, int radius );

std::vector<b3Vec3> GenerateVoxelSeedsInBox( const std::vector<VoxelCell>& cells, Int3 dimensions, Int3 lower,
											Int3 upper, float probability, uint32_t randomSeed );

std::vector<std::vector<int>> GenerateVoxelVoronoiGroups( const std::vector<int>& voxelIndices, Int3 dimensions,
														  const std::vector<b3Vec3>& seeds );

// Native equivalent of VoxelMapEdgeDestroyerJob.
std::vector<int> DestroyVoxelFragmentEdges( const std::vector<int>& group, Int3 dimensions );

// Native equivalent of OuterLayerDestroyJob: removes a voxel if any of its six
// neighbors is empty or outside the map.
std::vector<int> DestroyVoxelOuterLayer( const std::vector<int>& group, Int3 dimensions );
