// SPDX-FileCopyrightText: 2026
// SPDX-License-Identifier: MIT

#include "voxel_destruction/voxel_chunk_mesh.h"

#include "gfx/draw.h"

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <unordered_map>
#include <utility>

namespace
{

struct Face
{
	Int3 neighbor;
	b3Vec3 normal;
	b3Vec3 corners[4];
};

constexpr float h = 0.5f * kVoxelSize;

const Face faces[6] = {
	{ { 1, 0, 0 }, { 1.0f, 0.0f, 0.0f }, { { h, -h, -h }, { h, h, -h }, { h, h, h }, { h, -h, h } } },
	{ { -1, 0, 0 }, { -1.0f, 0.0f, 0.0f }, { { -h, -h, h }, { -h, h, h }, { -h, h, -h }, { -h, -h, -h } } },
	{ { 0, 1, 0 }, { 0.0f, 1.0f, 0.0f }, { { -h, h, -h }, { -h, h, h }, { h, h, h }, { h, h, -h } } },
	{ { 0, -1, 0 }, { 0.0f, -1.0f, 0.0f }, { { -h, -h, h }, { -h, -h, -h }, { h, -h, -h }, { h, -h, h } } },
	{ { 0, 0, 1 }, { 0.0f, 0.0f, 1.0f }, { { -h, -h, h }, { h, -h, h }, { h, h, h }, { -h, h, h } } },
	{ { 0, 0, -1 }, { 0.0f, 0.0f, -1.0f }, { { h, -h, -h }, { -h, -h, -h }, { -h, h, -h }, { h, h, -h } } },
};

std::atomic<uint32_t> nextMeshHash = 0xC0000001u;

int GetComponent( Int3 value, int axis )
{
	return axis == 0 ? value.x : axis == 1 ? value.y : value.z;
}

void SetComponent( Int3& value, int axis, int component )
{
	if ( axis == 0 )
	{
		value.x = component;
	}
	else if ( axis == 1 )
	{
		value.y = component;
	}
	else
	{
		value.z = component;
	}
}

void SetComponent( b3Vec3& value, int axis, float component )
{
	if ( axis == 0 )
	{
		value.x = component;
	}
	else if ( axis == 1 )
	{
		value.y = component;
	}
	else
	{
		value.z = component;
	}
}

uint32_t GetRgba( const VoxelCell& cell )
{
	return ( static_cast<uint32_t>( cell.r ) << 24 ) | ( static_cast<uint32_t>( cell.g ) << 16 ) |
		   ( static_cast<uint32_t>( cell.b ) << 8 ) | cell.a;
}

VoxelCpuMesh& GetColoredMesh( VoxelChunkMeshBuild& build, std::unordered_map<uint32_t, size_t>& meshIndices,
							 uint32_t rgba )
{
	auto [iterator, inserted] = meshIndices.try_emplace( rgba, build.coloredMeshes.size() );
	if ( inserted )
	{
		build.coloredMeshes.push_back( { rgba, {} } );
	}
	return build.coloredMeshes[iterator->second].mesh;
}

void AddFace( VoxelCpuMesh& mesh, b3Vec3 center, const Face& face, b3Vec3 halfExtents )
{
	uint32_t firstVertex = static_cast<uint32_t>( mesh.vertices.size() );
	for ( b3Vec3 corner : face.corners )
	{
		b3Vec3 offset = {
			corner.x < 0.0f ? -halfExtents.x : halfExtents.x,
			corner.y < 0.0f ? -halfExtents.y : halfExtents.y,
			corner.z < 0.0f ? -halfExtents.z : halfExtents.z,
		};
		MeshVertex vertex = {
			{ center.x + offset.x, center.y + offset.y, center.z + offset.z },
			{ face.normal.x, face.normal.y, face.normal.z },
		};
		mesh.vertices.push_back( vertex );
	}

	mesh.indices.push_back( firstVertex + 0 );
	mesh.indices.push_back( firstVertex + 1 );
	mesh.indices.push_back( firstVertex + 2 );
	mesh.indices.push_back( firstVertex + 0 );
	mesh.indices.push_back( firstVertex + 2 );
	mesh.indices.push_back( firstVertex + 3 );
}

VoxelChunkMeshBuild BuildChunkCpuMesh( int chunkIndex, Int3 lower, Int3 upper, Int3 dimensions,
									   const std::vector<VoxelCell>& cells )
{
	VoxelChunkMeshBuild build;
	build.chunkIndex = chunkIndex;
	std::unordered_map<uint32_t, size_t> meshIndices;

	for ( const Face& face : faces )
	{
		int normalAxis = face.neighbor.x != 0 ? 0 : face.neighbor.y != 0 ? 1 : 2;
		int uAxis = ( normalAxis + 1 ) % 3;
		int vAxis = ( normalAxis + 2 ) % 3;
		int normalLower = GetComponent( lower, normalAxis );
		int normalUpper = GetComponent( upper, normalAxis );
		int uLower = GetComponent( lower, uAxis );
		int uUpper = GetComponent( upper, uAxis );
		int vLower = GetComponent( lower, vAxis );
		int vUpper = GetComponent( upper, vAxis );
		int uCount = uUpper - uLower;
		int vCount = vUpper - vLower;
		std::vector<uint64_t> mask( static_cast<size_t>( uCount * vCount ) );

		for ( int normal = normalLower; normal < normalUpper; ++normal )
		{
			std::fill( mask.begin(), mask.end(), uint64_t{ 0 } );
			for ( int u = 0; u < uCount; ++u )
			{
				for ( int v = 0; v < vCount; ++v )
				{
					Int3 voxel = {};
					SetComponent( voxel, normalAxis, normal );
					SetComponent( voxel, uAxis, uLower + u );
					SetComponent( voxel, vAxis, vLower + v );
					const VoxelCell& cell = cells[GetIndex( voxel, dimensions )];
					if ( cell.filled == 0 )
					{
						continue;
					}

					Int3 neighbor = {
						voxel.x + face.neighbor.x,
						voxel.y + face.neighbor.y,
						voxel.z + face.neighbor.z,
					};
					if ( InBounds( neighbor, dimensions ) && cells[GetIndex( neighbor, dimensions )].filled != 0 )
					{
						continue;
					}
					mask[static_cast<size_t>( u * vCount + v )] = ( uint64_t{ 1 } << 32 ) | GetRgba( cell );
				}
			}

			for ( int u = 0; u < uCount; ++u )
			{
				for ( int v = 0; v < vCount; )
				{
					uint64_t key = mask[static_cast<size_t>( u * vCount + v )];
					if ( key == 0 )
					{
						v += 1;
						continue;
					}

					int width = 1;
					while ( v + width < vCount && mask[static_cast<size_t>( u * vCount + v + width )] == key )
					{
						width += 1;
					}

					int height = 1;
					while ( u + height < uCount )
					{
						bool matches = true;
						for ( int offset = 0; offset < width; ++offset )
						{
							if ( mask[static_cast<size_t>( ( u + height ) * vCount + v + offset )] != key )
							{
								matches = false;
								break;
							}
						}
						if ( matches == false )
						{
							break;
						}
						height += 1;
					}

					for ( int row = 0; row < height; ++row )
					{
						std::fill_n( mask.begin() + ( u + row ) * vCount + v, width, uint64_t{ 0 } );
					}

					Int3 first = {};
					Int3 last = {};
					SetComponent( first, normalAxis, normal );
					SetComponent( last, normalAxis, normal );
					SetComponent( first, uAxis, uLower + u );
					SetComponent( last, uAxis, uLower + u + height - 1 );
					SetComponent( first, vAxis, vLower + v );
					SetComponent( last, vAxis, vLower + v + width - 1 );
					b3Vec3 firstCenter = GetCellCenter( first, dimensions );
					b3Vec3 lastCenter = GetCellCenter( last, dimensions );
					b3Vec3 center = {
						0.5f * ( firstCenter.x + lastCenter.x ),
						0.5f * ( firstCenter.y + lastCenter.y ),
						0.5f * ( firstCenter.z + lastCenter.z ),
					};
					b3Vec3 halfExtents = { h, h, h };
					SetComponent( halfExtents, uAxis, height * h );
					SetComponent( halfExtents, vAxis, width * h );
					uint32_t rgba = static_cast<uint32_t>( key );
					AddFace( GetColoredMesh( build, meshIndices, rgba ), center, face, halfExtents );
					v += width;
				}
			}
		}
	}
	return build;
}

} // namespace

VoxelChunkMeshBuildBatch BuildVoxelChunkMeshes( Int3 dimensions, const std::vector<VoxelCell>& cells )
{
	Int3 chunkDimensions = {
		( dimensions.x + kVoxelChunkSize - 1 ) / kVoxelChunkSize,
		( dimensions.y + kVoxelChunkSize - 1 ) / kVoxelChunkSize,
		( dimensions.z + kVoxelChunkSize - 1 ) / kVoxelChunkSize,
	};

	std::vector<int> chunkIndices( static_cast<size_t>( chunkDimensions.x * chunkDimensions.y * chunkDimensions.z ) );
	for ( int chunkIndex = 0; chunkIndex < static_cast<int>( chunkIndices.size() ); ++chunkIndex )
	{
		chunkIndices[chunkIndex] = chunkIndex;
	}
	return BuildVoxelChunkMeshes( dimensions, cells, chunkIndices );
}

VoxelChunkMeshBuildBatch BuildVoxelChunkMeshes( Int3 dimensions, const std::vector<VoxelCell>& cells,
												const std::vector<int>& chunkIndices )
{
	Int3 chunkDimensions = {
		( dimensions.x + kVoxelChunkSize - 1 ) / kVoxelChunkSize,
		( dimensions.y + kVoxelChunkSize - 1 ) / kVoxelChunkSize,
		( dimensions.z + kVoxelChunkSize - 1 ) / kVoxelChunkSize,
	};
	int chunkCount = chunkDimensions.x * chunkDimensions.y * chunkDimensions.z;
	VoxelChunkMeshBuildBatch result;
	result.reserve( chunkIndices.size() );
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
		result.push_back( BuildChunkCpuMesh( chunkIndex, lower, upper, dimensions, cells ) );
	}
	return result;
}

VoxelChunkMeshCache::VoxelChunkMeshCache( Int3 dimensions, const std::vector<VoxelCell>& cells )
	: m_dimensions( dimensions )
{
	InitializeChunks();
	RebuildDirty( cells );
}

VoxelChunkMeshCache::VoxelChunkMeshCache( Int3 dimensions, VoxelChunkMeshBuildBatch prebuiltMeshes )
	: m_dimensions( dimensions )
{
	InitializeChunks();
	ApplyBuildBatch( std::move( prebuiltMeshes ) );
}

VoxelChunkMeshCache::~VoxelChunkMeshCache()
{
	for ( Chunk& chunk : m_chunks )
	{
		ReleaseChunkMeshes( chunk );
	}
}

void VoxelChunkMeshCache::InitializeChunks()
{
	m_chunkDimensions = {
		( m_dimensions.x + kVoxelChunkSize - 1 ) / kVoxelChunkSize,
		( m_dimensions.y + kVoxelChunkSize - 1 ) / kVoxelChunkSize,
		( m_dimensions.z + kVoxelChunkSize - 1 ) / kVoxelChunkSize,
	};

	m_chunks.reserve( m_chunkDimensions.x * m_chunkDimensions.y * m_chunkDimensions.z );
	for ( int x = 0; x < m_chunkDimensions.x; ++x )
	{
		for ( int y = 0; y < m_chunkDimensions.y; ++y )
		{
			for ( int z = 0; z < m_chunkDimensions.z; ++z )
			{
				Chunk chunk;
				chunk.lower = { x * kVoxelChunkSize, y * kVoxelChunkSize, z * kVoxelChunkSize };
				chunk.upper = {
					std::min( chunk.lower.x + kVoxelChunkSize, m_dimensions.x ),
					std::min( chunk.lower.y + kVoxelChunkSize, m_dimensions.y ),
					std::min( chunk.lower.z + kVoxelChunkSize, m_dimensions.z ),
				};
				m_chunks.push_back( chunk );
			}
		}
	}
}

void VoxelChunkMeshCache::MarkAllDirty()
{
	for ( Chunk& chunk : m_chunks )
	{
		chunk.dirty = true;
	}
}

int VoxelChunkMeshCache::GetChunkIndex( Int3 chunkCoordinates ) const
{
	return ( chunkCoordinates.x * m_chunkDimensions.y + chunkCoordinates.y ) * m_chunkDimensions.z + chunkCoordinates.z;
}

void VoxelChunkMeshCache::MarkVoxelDirty( Int3 voxel )
{
	if ( InBounds( voxel, m_dimensions ) == false )
	{
		return;
	}

	Int3 chunk = { voxel.x / kVoxelChunkSize, voxel.y / kVoxelChunkSize, voxel.z / kVoxelChunkSize };
	m_chunks[GetChunkIndex( chunk )].dirty = true;

	const int localX = voxel.x % kVoxelChunkSize;
	const int localY = voxel.y % kVoxelChunkSize;
	const int localZ = voxel.z % kVoxelChunkSize;
	const Int3 neighbors[6] = {
		{ chunk.x - 1, chunk.y, chunk.z }, { chunk.x + 1, chunk.y, chunk.z }, { chunk.x, chunk.y - 1, chunk.z },
		{ chunk.x, chunk.y + 1, chunk.z }, { chunk.x, chunk.y, chunk.z - 1 }, { chunk.x, chunk.y, chunk.z + 1 },
	};
	const bool touchesBorder[6] = {
		localX == 0,
		localX == kVoxelChunkSize - 1,
		localY == 0,
		localY == kVoxelChunkSize - 1,
		localZ == 0,
		localZ == kVoxelChunkSize - 1,
	};

	for ( int i = 0; i < 6; ++i )
	{
		if ( touchesBorder[i] == false || InBounds( neighbors[i], m_chunkDimensions ) == false )
		{
			continue;
		}
		m_chunks[GetChunkIndex( neighbors[i] )].dirty = true;
	}
}

bool VoxelChunkMeshCache::MarkChunkDirty( Int3 chunkCoordinates )
{
	if ( InBounds( chunkCoordinates, m_chunkDimensions ) == false )
	{
		return false;
	}
	m_chunks[GetChunkIndex( chunkCoordinates )].dirty = true;
	return true;
}

void VoxelChunkMeshCache::RebuildDirty( const std::vector<VoxelCell>& cells )
{
	for ( Chunk& chunk : m_chunks )
	{
		if ( chunk.dirty )
		{
			RebuildChunk( chunk, cells );
		}
	}
}

void VoxelChunkMeshCache::ReleaseChunkMeshes( Chunk& chunk )
{
	for ( Chunk::ColoredMesh& mesh : chunk.meshes )
	{
		if ( IsMeshHandleValid( mesh.handle ) )
		{
			ReleaseMeshReference( mesh.handle );
		}
	}
	chunk.meshes.clear();
	chunk.triangleCount = 0;
}

void VoxelChunkMeshCache::RebuildChunk( Chunk& chunk, const std::vector<VoxelCell>& cells )
{
	int chunkIndex = static_cast<int>( &chunk - m_chunks.data() );
	ApplyChunkBuild( BuildChunkCpuMesh( chunkIndex, chunk.lower, chunk.upper, m_dimensions, cells ) );
}

void VoxelChunkMeshCache::ApplyChunkBuild( VoxelChunkMeshBuild build )
{
	if ( build.chunkIndex < 0 || build.chunkIndex >= static_cast<int>( m_chunks.size() ) )
	{
		return;
	}

	Chunk& chunk = m_chunks[build.chunkIndex];
	ReleaseChunkMeshes( chunk );
	for ( VoxelColoredCpuMesh& coloredMesh : build.coloredMeshes )
	{
		VoxelCpuMesh& mesh = coloredMesh.mesh;
		if ( mesh.indices.empty() )
		{
			continue;
		}

		uint32_t hash = nextMeshHash.fetch_add( 1, std::memory_order_relaxed );
		MeshHandle handle = RegisterMesh( hash, mesh.vertices.data(), static_cast<int>( mesh.vertices.size() ),
										 mesh.indices.data(), static_cast<int>( mesh.indices.size() ), "voxel chunk" );
		chunk.meshes.push_back( { handle, coloredMesh.rgba } );
		chunk.triangleCount += static_cast<int>( mesh.indices.size() / 3 );
	}

	chunk.dirty = false;
}

void VoxelChunkMeshCache::ApplyBuildBatch( VoxelChunkMeshBuildBatch builds )
{
	for ( VoxelChunkMeshBuild& build : builds )
	{
		ApplyChunkBuild( std::move( build ) );
	}
}

void VoxelChunkMeshCache::Draw( b3WorldTransform transform ) const
{
	b3Transform relativeTransform = b3ToRelativeTransform( transform, GetDrawOrigin() );
	for ( const Chunk& chunk : m_chunks )
	{
		for ( const Chunk::ColoredMesh& mesh : chunk.meshes )
		{
			if ( IsMeshHandleValid( mesh.handle ) == false )
			{
				continue;
			}

			b3HexColor color = static_cast<b3HexColor>( mesh.rgba >> 8 );
			float alpha = static_cast<float>( mesh.rgba & 0xFFu ) / 255.0f;
			AppendMesh( mesh.handle, relativeTransform, b3Vec3_one, MakeColorAlpha( color, alpha ), 0.0f, 0.8f,
						MESH_MATERIAL_MODE_SOLID, 1.0f, TRANSPARENT_SHADOW_FULL );
		}
	}
}

int VoxelChunkMeshCache::GetChunkCount() const
{
	return static_cast<int>( m_chunks.size() );
}

int VoxelChunkMeshCache::GetDirtyChunkCount() const
{
	int count = 0;
	for ( const Chunk& chunk : m_chunks )
	{
		count += chunk.dirty;
	}
	return count;
}

std::vector<int> VoxelChunkMeshCache::GetDirtyChunkIndices() const
{
	std::vector<int> indices;
	indices.reserve( static_cast<size_t>( GetDirtyChunkCount() ) );
	for ( int index = 0; index < static_cast<int>( m_chunks.size() ); ++index )
	{
		if ( m_chunks[index].dirty )
		{
			indices.push_back( index );
		}
	}
	return indices;
}

int VoxelChunkMeshCache::GetMeshCount() const
{
	int count = 0;
	for ( const Chunk& chunk : m_chunks )
	{
		count += static_cast<int>( chunk.meshes.size() );
	}
	return count;
}

int VoxelChunkMeshCache::GetTriangleCount() const
{
	int count = 0;
	for ( const Chunk& chunk : m_chunks )
	{
		count += chunk.triangleCount;
	}
	return count;
}
