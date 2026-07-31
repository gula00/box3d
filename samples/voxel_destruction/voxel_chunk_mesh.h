// SPDX-FileCopyrightText: 2026
// SPDX-License-Identifier: MIT

#pragma once

#include "gfx/geometry_registry.h"
#include "voxel_destruction/voxel_types.h"

#include <cstdint>
#include <vector>

struct VoxelCpuMesh
{
	std::vector<MeshVertex> vertices;
	std::vector<uint32_t> indices;
};

struct VoxelColoredCpuMesh
{
	uint32_t rgba = 0;
	VoxelCpuMesh mesh;
};

struct VoxelChunkMeshBuild
{
	int chunkIndex = 0;
	std::vector<VoxelColoredCpuMesh> coloredMeshes;
};

using VoxelChunkMeshBuildBatch = std::vector<VoxelChunkMeshBuild>;

// Pure CPU mesh generation. Safe to call from a worker thread.
VoxelChunkMeshBuildBatch BuildVoxelChunkMeshes( Int3 dimensions, const std::vector<VoxelCell>& cells );
VoxelChunkMeshBuildBatch BuildVoxelChunkMeshes( Int3 dimensions, const std::vector<VoxelCell>& cells,
												const std::vector<int>& chunkIndices );

class VoxelChunkMeshCache
{
public:
	VoxelChunkMeshCache( Int3 dimensions, const std::vector<VoxelCell>& cells );
	VoxelChunkMeshCache( Int3 dimensions, VoxelChunkMeshBuildBatch prebuiltMeshes );
	~VoxelChunkMeshCache();

	VoxelChunkMeshCache( const VoxelChunkMeshCache& ) = delete;
	VoxelChunkMeshCache& operator=( const VoxelChunkMeshCache& ) = delete;

	void MarkAllDirty();
	void MarkVoxelDirty( Int3 voxel );
	bool MarkChunkDirty( Int3 chunkCoordinates );
	void RebuildDirty( const std::vector<VoxelCell>& cells );
	void ApplyBuildBatch( VoxelChunkMeshBuildBatch builds );
	void Draw( b3WorldTransform transform ) const;

	int GetChunkCount() const;
	int GetDirtyChunkCount() const;
	std::vector<int> GetDirtyChunkIndices() const;
	int GetMeshCount() const;
	int GetTriangleCount() const;

private:
	struct Chunk
	{
		struct ColoredMesh
		{
			MeshHandle handle = InvalidMeshHandle();
			uint32_t rgba = 0;
		};

		Int3 lower;
		Int3 upper;
		std::vector<ColoredMesh> meshes;
		bool dirty = true;
		int triangleCount = 0;
	};

	void RebuildChunk( Chunk& chunk, const std::vector<VoxelCell>& cells );
	void InitializeChunks();
	void ApplyChunkBuild( VoxelChunkMeshBuild build );
	void ReleaseChunkMeshes( Chunk& chunk );
	int GetChunkIndex( Int3 chunkCoordinates ) const;

	Int3 m_dimensions;
	Int3 m_chunkDimensions;
	std::vector<Chunk> m_chunks;
};
