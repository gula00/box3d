// SPDX-FileCopyrightText: 2026
// SPDX-License-Identifier: MIT

#include "voxel_destruction/voxel_editing.h"

#include <algorithm>
#include <cstdint>

std::vector<VoxelCell> ResizeVoxelMapNearest( const std::vector<VoxelCell>& source, Int3 sourceDimensions,
											  Int3 targetDimensions )
{
	if ( sourceDimensions.x <= 0 || sourceDimensions.y <= 0 || sourceDimensions.z <= 0 || targetDimensions.x <= 0 ||
		 targetDimensions.y <= 0 || targetDimensions.z <= 0 ||
		 source.size() != static_cast<size_t>( sourceDimensions.x * sourceDimensions.y * sourceDimensions.z ) )
	{
		return {};
	}

	std::vector<VoxelCell> result( targetDimensions.x * targetDimensions.y * targetDimensions.z );
	for ( int x = 0; x < targetDimensions.x; ++x )
	{
		for ( int y = 0; y < targetDimensions.y; ++y )
		{
			for ( int z = 0; z < targetDimensions.z; ++z )
			{
				Int3 sourceIndex = {
					std::min( sourceDimensions.x - 1, x * sourceDimensions.x / targetDimensions.x ),
					std::min( sourceDimensions.y - 1, y * sourceDimensions.y / targetDimensions.y ),
					std::min( sourceDimensions.z - 1, z * sourceDimensions.z / targetDimensions.z ),
				};
				result[GetIndex( { x, y, z }, targetDimensions )] = source[GetIndex( sourceIndex, sourceDimensions )];
			}
		}
	}
	return result;
}

std::vector<VoxelCell> HollowVoxelMap( const std::vector<VoxelCell>& source, Int3 dimensions, int wallThickness )
{
	if ( wallThickness < 1 ||
		 source.size() != static_cast<size_t>( dimensions.x * dimensions.y * dimensions.z ) )
	{
		return {};
	}

	constexpr Int3 neighbors[6] = {
		{ -1, 0, 0 }, { 1, 0, 0 }, { 0, -1, 0 }, { 0, 1, 0 }, { 0, 0, -1 }, { 0, 0, 1 },
	};

	std::vector<uint8_t> shell( source.size(), 0 );
	for ( int index = 0; index < static_cast<int>( source.size() ); ++index )
	{
		if ( source[index].filled == 0 )
		{
			continue;
		}

		Int3 p = GetCoordinates( index, dimensions );
		for ( Int3 offset : neighbors )
		{
			Int3 neighbor = { p.x + offset.x, p.y + offset.y, p.z + offset.z };
			if ( InBounds( neighbor, dimensions ) == false || source[GetIndex( neighbor, dimensions )].filled == 0 )
			{
				shell[index] = 1;
				break;
			}
		}
	}

	for ( int iteration = 1; iteration < wallThickness; ++iteration )
	{
		std::vector<uint8_t> additions( source.size(), 0 );
		for ( int index = 0; index < static_cast<int>( source.size() ); ++index )
		{
			if ( source[index].filled == 0 || shell[index] != 0 )
			{
				continue;
			}

			Int3 p = GetCoordinates( index, dimensions );
			for ( Int3 offset : neighbors )
			{
				Int3 neighbor = { p.x + offset.x, p.y + offset.y, p.z + offset.z };
				if ( InBounds( neighbor, dimensions ) && shell[GetIndex( neighbor, dimensions )] != 0 )
				{
					additions[index] = 1;
					break;
				}
			}
		}

		for ( size_t index = 0; index < shell.size(); ++index )
		{
			shell[index] = static_cast<uint8_t>( shell[index] | additions[index] );
		}
	}

	std::vector<VoxelCell> result = source;
	for ( size_t index = 0; index < result.size(); ++index )
	{
		if ( shell[index] == 0 )
		{
			result[index].filled = 0;
		}
	}
	return result;
}
