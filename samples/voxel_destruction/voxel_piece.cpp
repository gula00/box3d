// SPDX-FileCopyrightText: 2026
// SPDX-License-Identifier: MIT

#include "voxel_destruction/voxel_piece.h"

#include <algorithm>
#include <utility>

namespace
{

static b3Vec3 Add( b3Vec3 a, b3Vec3 b )
{
	return { a.x + b.x, a.y + b.y, a.z + b.z };
}

} // namespace

VoxelPieceBuild BuildVoxelPiece( const VoxelSourceSnapshot& source, const std::vector<int>& indices, bool dynamic,
								 b3Vec3 localKick )
{
	Int3 lower = source.dimensions;
	Int3 upper = { 0, 0, 0 };
	for ( int index : indices )
	{
		Int3 p = GetCoordinates( index, source.dimensions );
		lower.x = std::min( lower.x, p.x );
		lower.y = std::min( lower.y, p.y );
		lower.z = std::min( lower.z, p.z );
		upper.x = std::max( upper.x, p.x );
		upper.y = std::max( upper.y, p.y );
		upper.z = std::max( upper.z, p.z );
	}

	Int3 dimensions = { upper.x - lower.x + 1, upper.y - lower.y + 1, upper.z - lower.z + 1 };
	std::vector<VoxelCell> cells( dimensions.x * dimensions.y * dimensions.z );
	for ( int index : indices )
	{
		Int3 oldP = GetCoordinates( index, source.dimensions );
		Int3 newP = { oldP.x - lower.x, oldP.y - lower.y, oldP.z - lower.z };
		cells[GetIndex( newP, dimensions )] = source.cells[index];
	}

	b3Vec3 localOffset = {
		( lower.x + 0.5f * dimensions.x - 0.5f * source.dimensions.x ) * kVoxelSize,
		( lower.y + 0.5f * dimensions.y - 0.5f * source.dimensions.y ) * kVoxelSize,
		( lower.z + 0.5f * dimensions.z - 0.5f * source.dimensions.z ) * kVoxelSize,
	};
	b3Vec3 worldOffset = b3RotateVector( source.rotation, localOffset );

	b3Vec3 linearVelocity = b3Vec3_zero;
	if ( source.dynamic )
	{
		linearVelocity = Add( source.linearVelocity, b3Cross( source.angularVelocity, worldOffset ) );
	}
	linearVelocity = Add( linearVelocity, b3RotateVector( source.rotation, localKick ) );

	std::vector<VoxelBoxRun> colliderRuns = BuildVoxelBoxRuns( cells, dimensions );
	VoxelChunkMeshBuildBatch meshBuilds = BuildVoxelChunkMeshes( dimensions, cells );
	return {
		dimensions,
		std::move( cells ),
		source.position + worldOffset,
		source.rotation,
		linearVelocity,
		source.dynamic ? source.angularVelocity : b3Vec3_zero,
		dynamic,
		std::move( colliderRuns ),
		std::move( meshBuilds ),
	};
}
