// SPDX-FileCopyrightText: 2026
// SPDX-License-Identifier: MIT

#include "voxel_destruction/voxel_collider.h"

#include <algorithm>
#include <cstdint>

namespace
{

std::vector<VoxelBoxRun> BuildVoxelBoxRunsInBounds( const std::vector<VoxelCell>& cells, Int3 dimensions, Int3 lower,
													Int3 upper )
{
	Int3 localDimensions = { upper.x - lower.x, upper.y - lower.y, upper.z - lower.z };
	std::vector<uint8_t> used( static_cast<size_t>( localDimensions.x * localDimensions.y * localDimensions.z ), 0 );
	std::vector<VoxelBoxRun> boxes;
	auto getUsedIndex = [lower, localDimensions]( Int3 voxel )
	{
		Int3 local = { voxel.x - lower.x, voxel.y - lower.y, voxel.z - lower.z };
		return GetIndex( local, localDimensions );
	};

	for ( int x = lower.x; x < upper.x; ++x )
	{
		for ( int y = lower.y; y < upper.y; ++y )
		{
			for ( int z = lower.z; z < upper.z; ++z )
			{
				Int3 start = { x, y, z };
				int startIndex = GetIndex( start, dimensions );
				if ( cells[startIndex].filled == 0 || used[getUsedIndex( start )] != 0 )
				{
					continue;
				}

				Int3 end = start;

				for ( int scanZ = z + 1; scanZ < upper.z; ++scanZ )
				{
					int index = GetIndex( { x, y, scanZ }, dimensions );
					if ( cells[index].filled == 0 || used[getUsedIndex( { x, y, scanZ } )] != 0 )
					{
						break;
					}
					end.z = scanZ;
				}

				for ( int scanY = y + 1; scanY < upper.y; ++scanY )
				{
					bool canExtend = true;
					for ( int scanZ = z; scanZ <= end.z; ++scanZ )
					{
						int index = GetIndex( { x, scanY, scanZ }, dimensions );
						if ( cells[index].filled == 0 || used[getUsedIndex( { x, scanY, scanZ } )] != 0 )
						{
							canExtend = false;
							break;
						}
					}

					if ( canExtend == false )
					{
						break;
					}
					end.y = scanY;
				}

				for ( int scanX = x + 1; scanX < upper.x; ++scanX )
				{
					bool canExtend = true;
					for ( int scanY = y; scanY <= end.y && canExtend; ++scanY )
					{
						for ( int scanZ = z; scanZ <= end.z; ++scanZ )
						{
							int index = GetIndex( { scanX, scanY, scanZ }, dimensions );
							if ( cells[index].filled == 0 || used[getUsedIndex( { scanX, scanY, scanZ } )] != 0 )
							{
								canExtend = false;
								break;
							}
						}
					}

					if ( canExtend == false )
					{
						break;
					}
					end.x = scanX;
				}

				for ( int markX = x; markX <= end.x; ++markX )
				{
					for ( int markY = y; markY <= end.y; ++markY )
					{
						for ( int markZ = z; markZ <= end.z; ++markZ )
						{
							used[getUsedIndex( { markX, markY, markZ } )] = 1;
						}
					}
				}

				boxes.push_back( { start, end } );
			}
		}
	}

	return boxes;
}

} // namespace

std::vector<VoxelBoxRun> BuildVoxelBoxRuns( const std::vector<VoxelCell>& cells, Int3 dimensions )
{
	return BuildVoxelBoxRunsInBounds( cells, dimensions, { 0, 0, 0 }, dimensions );
}

VoxelChunkColliderBuildBatch BuildVoxelChunkBoxRuns( const std::vector<VoxelCell>& cells, Int3 dimensions,
													 const std::vector<int>& chunkIndices )
{
	Int3 chunkDimensions = {
		( dimensions.x + kVoxelChunkSize - 1 ) / kVoxelChunkSize,
		( dimensions.y + kVoxelChunkSize - 1 ) / kVoxelChunkSize,
		( dimensions.z + kVoxelChunkSize - 1 ) / kVoxelChunkSize,
	};
	int chunkCount = chunkDimensions.x * chunkDimensions.y * chunkDimensions.z;
	VoxelChunkColliderBuildBatch builds;
	builds.reserve( chunkIndices.size() );
	for ( int chunkIndex : chunkIndices )
	{
		if ( chunkIndex < 0 || chunkIndex >= chunkCount )
		{
			continue;
		}
		Int3 chunk = GetCoordinates( chunkIndex, chunkDimensions );
		Int3 lower = { chunk.x * kVoxelChunkSize, chunk.y * kVoxelChunkSize, chunk.z * kVoxelChunkSize };
		Int3 upper = {
			std::min( lower.x + kVoxelChunkSize, dimensions.x ),
			std::min( lower.y + kVoxelChunkSize, dimensions.y ),
			std::min( lower.z + kVoxelChunkSize, dimensions.z ),
		};
		builds.push_back( { chunkIndex, BuildVoxelBoxRunsInBounds( cells, dimensions, lower, upper ) } );
	}
	return builds;
}
