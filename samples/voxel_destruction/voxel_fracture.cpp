// SPDX-FileCopyrightText: 2026
// SPDX-License-Identifier: MIT

#include "voxel_destruction/voxel_fracture.h"

#include <cfloat>
#include <cstdint>
#include <algorithm>
#include <random>

namespace
{

constexpr Int3 neighborOffsets[6] = {
	{ -1, 0, 0 }, { 1, 0, 0 }, { 0, -1, 0 }, { 0, 1, 0 }, { 0, 0, -1 }, { 0, 0, 1 },
};

std::vector<uint8_t> BuildMemberMap( const std::vector<int>& group, Int3 dimensions )
{
	std::vector<uint8_t> memberMap( dimensions.x * dimensions.y * dimensions.z, 0 );
	for ( int index : group )
	{
		memberMap[index] = 1;
	}
	return memberMap;
}

} // namespace

std::vector<int> CutVoxelSphere( const std::vector<VoxelCell>& cells, Int3 dimensions, b3Vec3 center, float radius )
{
	std::vector<int> result;
	float radiusSquared = radius * radius;
	for ( int index = 0; index < static_cast<int>( cells.size() ); ++index )
	{
		if ( cells[index].filled == 0 )
		{
			continue;
		}

		b3Vec3 voxelCenter = GetCellCenter( GetCoordinates( index, dimensions ), dimensions );
		b3Vec3 delta = {
			voxelCenter.x - center.x,
			voxelCenter.y - center.y,
			voxelCenter.z - center.z,
		};
		float distanceSquared = delta.x * delta.x + delta.y * delta.y + delta.z * delta.z;
		if ( distanceSquared <= radiusSquared )
		{
			result.push_back( index );
		}
	}
	return result;
}

VoxelSphereRegion CopyVoxelSphere( const std::vector<VoxelCell>& cells, Int3 dimensions, Int3 center, int radius )
{
	VoxelSphereRegion result;
	if ( radius < 0 || cells.size() != static_cast<size_t>( dimensions.x * dimensions.y * dimensions.z ) )
	{
		return result;
	}

	Int3 lower = {
		std::max( 0, center.x - radius ),
		std::max( 0, center.y - radius ),
		std::max( 0, center.z - radius ),
	};
	Int3 upper = {
		std::min( dimensions.x - 1, center.x + radius ),
		std::min( dimensions.y - 1, center.y + radius ),
		std::min( dimensions.z - 1, center.z + radius ),
	};
	if ( upper.x < lower.x || upper.y < lower.y || upper.z < lower.z )
	{
		return result;
	}

	result.sourceOffset = lower;
	result.dimensions = { upper.x - lower.x + 1, upper.y - lower.y + 1, upper.z - lower.z + 1 };
	result.cells.resize(
		static_cast<size_t>( result.dimensions.x ) * result.dimensions.y * result.dimensions.z );
	int radiusSquared = radius * radius;
	for ( int x = lower.x; x <= upper.x; ++x )
	{
		for ( int y = lower.y; y <= upper.y; ++y )
		{
			for ( int z = lower.z; z <= upper.z; ++z )
			{
				int dx = x - center.x;
				int dy = y - center.y;
				int dz = z - center.z;
				if ( dx * dx + dy * dy + dz * dz > radiusSquared )
				{
					continue;
				}
				Int3 local = { x - lower.x, y - lower.y, z - lower.z };
				result.cells[GetIndex( local, result.dimensions )] = cells[GetIndex( { x, y, z }, dimensions )];
			}
		}
	}
	return result;
}

int EraseVoxelSphere( std::vector<VoxelCell>* cells, Int3 dimensions, Int3 center, int radius )
{
	if ( cells == nullptr || radius < 0 ||
		 cells->size() != static_cast<size_t>( dimensions.x * dimensions.y * dimensions.z ) )
	{
		return 0;
	}

	int erased = 0;
	int radiusSquared = radius * radius;
	for ( int x = std::max( 0, center.x - radius ); x <= std::min( dimensions.x - 1, center.x + radius ); ++x )
	{
		for ( int y = std::max( 0, center.y - radius ); y <= std::min( dimensions.y - 1, center.y + radius ); ++y )
		{
			for ( int z = std::max( 0, center.z - radius ); z <= std::min( dimensions.z - 1, center.z + radius ); ++z )
			{
				int dx = x - center.x;
				int dy = y - center.y;
				int dz = z - center.z;
				if ( dx * dx + dy * dy + dz * dz > radiusSquared )
				{
					continue;
				}
				VoxelCell& cell = ( *cells )[GetIndex( { x, y, z }, dimensions )];
				erased += cell.filled != 0;
				cell.filled = 0;
			}
		}
	}
	return erased;
}

VoxelSphereRegion CutAndCopyVoxelSphere( std::vector<VoxelCell>* cells, Int3 dimensions, Int3 center, int radius )
{
	if ( cells == nullptr )
	{
		return {};
	}
	VoxelSphereRegion result = CopyVoxelSphere( *cells, dimensions, center, radius );
	EraseVoxelSphere( cells, dimensions, center, radius );
	return result;
}

std::vector<b3Vec3> GenerateVoxelSeedsInBox( const std::vector<VoxelCell>& cells, Int3 dimensions, Int3 lower,
											Int3 upper, float probability, uint32_t randomSeed )
{
	std::vector<b3Vec3> seeds;
	if ( cells.size() != static_cast<size_t>( dimensions.x * dimensions.y * dimensions.z ) )
	{
		return seeds;
	}
	lower = { std::max( 0, lower.x ), std::max( 0, lower.y ), std::max( 0, lower.z ) };
	upper = {
		std::min( dimensions.x - 1, upper.x ),
		std::min( dimensions.y - 1, upper.y ),
		std::min( dimensions.z - 1, upper.z ),
	};
	probability = std::clamp( probability, 0.0f, 1.0f );
	std::mt19937 random( randomSeed );
	std::uniform_real_distribution<float> distribution( 0.0f, 1.0f );
	for ( int x = lower.x; x <= upper.x; ++x )
	{
		for ( int y = lower.y; y <= upper.y; ++y )
		{
			for ( int z = lower.z; z <= upper.z; ++z )
			{
				Int3 voxel = { x, y, z };
				if ( cells[GetIndex( voxel, dimensions )].filled && distribution( random ) <= probability )
				{
					seeds.push_back( GetCellCenter( voxel, dimensions ) );
				}
			}
		}
	}
	return seeds;
}

std::vector<std::vector<int>> GenerateVoxelVoronoiGroups( const std::vector<int>& voxelIndices, Int3 dimensions,
														  const std::vector<b3Vec3>& seeds )
{
	std::vector<std::vector<int>> groups( seeds.size() );
	if ( seeds.empty() )
	{
		return groups;
	}

	for ( int index : voxelIndices )
	{
		b3Vec3 center = GetCellCenter( GetCoordinates( index, dimensions ), dimensions );
		int nearestSeed = 0;
		float nearestDistanceSquared = FLT_MAX;

		for ( int seedIndex = 0; seedIndex < static_cast<int>( seeds.size() ); ++seedIndex )
		{
			b3Vec3 delta = {
				center.x - seeds[seedIndex].x,
				center.y - seeds[seedIndex].y,
				center.z - seeds[seedIndex].z,
			};
			float distanceSquared = delta.x * delta.x + delta.y * delta.y + delta.z * delta.z;
			if ( distanceSquared < nearestDistanceSquared )
			{
				nearestDistanceSquared = distanceSquared;
				nearestSeed = seedIndex;
			}
		}

		groups[nearestSeed].push_back( index );
	}
	return groups;
}

std::vector<int> DestroyVoxelFragmentEdges( const std::vector<int>& group, Int3 dimensions )
{
	std::vector<uint8_t> memberMap = BuildMemberMap( group, dimensions );
	std::vector<int> result;
	result.reserve( group.size() );

	for ( int index : group )
	{
		Int3 p = GetCoordinates( index, dimensions );
		int neighborCount = 0;
		for ( Int3 offset : neighborOffsets )
		{
			Int3 neighbor = { p.x + offset.x, p.y + offset.y, p.z + offset.z };
			if ( InBounds( neighbor, dimensions ) && memberMap[GetIndex( neighbor, dimensions )] != 0 )
			{
				neighborCount += 1;
			}
		}

		if ( neighborCount > 3 )
		{
			result.push_back( index );
		}
	}
	return result;
}

std::vector<int> DestroyVoxelOuterLayer( const std::vector<int>& group, Int3 dimensions )
{
	std::vector<uint8_t> memberMap = BuildMemberMap( group, dimensions );
	std::vector<int> result;
	result.reserve( group.size() );

	for ( int index : group )
	{
		Int3 p = GetCoordinates( index, dimensions );
		bool isInterior = true;
		for ( Int3 offset : neighborOffsets )
		{
			Int3 neighbor = { p.x + offset.x, p.y + offset.y, p.z + offset.z };
			if ( InBounds( neighbor, dimensions ) == false || memberMap[GetIndex( neighbor, dimensions )] == 0 )
			{
				isInterior = false;
				break;
			}
		}

		if ( isInterior )
		{
			result.push_back( index );
		}
	}
	return result;
}
