// SPDX-FileCopyrightText: 2026
// SPDX-License-Identifier: MIT

#pragma once

#include "box3d/box3d.h"

#include <cstdint>

constexpr float kVoxelSize = 0.2f;
constexpr float kVoxelMass = 0.01f;
constexpr float kVoxelDensity = kVoxelMass / ( kVoxelSize * kVoxelSize * kVoxelSize );
constexpr int kVoxelChunkSize = 16;
constexpr int kVoxelMaterialCount = 5;

struct Int3
{
	int x;
	int y;
	int z;
};

struct VoxelCell
{
	uint8_t filled = 0;
	uint8_t material = 0;
	uint8_t r = 255;
	uint8_t g = 255;
	uint8_t b = 255;
	uint8_t a = 255;
};

inline int GetIndex( Int3 p, Int3 dimensions )
{
	return ( p.x * dimensions.y + p.y ) * dimensions.z + p.z;
}

inline Int3 GetCoordinates( int index, Int3 dimensions )
{
	Int3 p;
	p.x = index / ( dimensions.y * dimensions.z );
	index -= p.x * dimensions.y * dimensions.z;
	p.y = index / dimensions.z;
	p.z = index - p.y * dimensions.z;
	return p;
}

inline bool InBounds( Int3 p, Int3 dimensions )
{
	return 0 <= p.x && p.x < dimensions.x && 0 <= p.y && p.y < dimensions.y && 0 <= p.z && p.z < dimensions.z;
}

inline b3Vec3 GetCellCenter( Int3 p, Int3 dimensions )
{
	return {
		( p.x + 0.5f - 0.5f * dimensions.x ) * kVoxelSize,
		( p.y + 0.5f - 0.5f * dimensions.y ) * kVoxelSize,
		( p.z + 0.5f - 0.5f * dimensions.z ) * kVoxelSize,
	};
}

inline b3HexColor GetVoxelColor( uint8_t material )
{
	constexpr b3HexColor colors[kVoxelMaterialCount] = {
		b3_colorLightSteelBlue,
		b3_colorCornflowerBlue,
		b3_colorSteelBlue,
		b3_colorLightSkyBlue,
		b3_colorWheat,
	};
	return colors[material % kVoxelMaterialCount];
}
