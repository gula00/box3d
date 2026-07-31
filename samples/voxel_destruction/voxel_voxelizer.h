// SPDX-FileCopyrightText: 2026
// SPDX-License-Identifier: MIT

#pragma once

#include "voxel_destruction/voxel_types.h"

#include "box3d/box3d.h"

#include <array>
#include <cstdint>
#include <vector>

struct VoxelTextureImage
{
	int width = 0;
	int height = 0;
	std::vector<uint8_t> rgba;
};

struct VoxelTriangleSurface
{
	uint32_t color = 0xC58B54FFu;
	int textureIndex = -1;
	std::array<b3Vec2, 3> uv = {};
};

struct VoxelTriangleMesh
{
	std::vector<b3Vec3> vertices;
	std::vector<uint32_t> indices;

	// Optional 0xRRGGBBAA value per triangle.
	std::vector<uint32_t> triangleColors;

	// Optional material/UV data used to reproduce Unity's hit-point texture
	// sampling. When present this takes precedence over triangleColors.
	std::vector<VoxelTriangleSurface> triangleSurfaces;
	std::vector<VoxelTextureImage> textures;
};

struct VoxelizedTriangleMesh
{
	Int3 dimensions = {};
	std::vector<VoxelCell> cells;
	b3Vec3 sourceBoundsLower = {};
	b3Vec3 sourceBoundsUpper = {};
};

// Native data equivalent of the Unity EditorTools.Voxelizer. It marks both
// surface samples and the interior of closed triangle meshes.
VoxelizedTriangleMesh VoxelizeTriangleMesh( const VoxelTriangleMesh& mesh, float scanVoxelSize );
