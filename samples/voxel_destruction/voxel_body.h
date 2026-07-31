// SPDX-FileCopyrightText: 2026
// SPDX-License-Identifier: MIT

#pragma once

#include "voxel_destruction/voxel_chunk_mesh.h"
#include "voxel_destruction/voxel_collider.h"
#include "voxel_destruction/voxel_types.h"

#include "box3d/box3d.h"

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

class VoxelBody
{
public:
	VoxelBody( b3WorldId worldId, Int3 dimensions, std::vector<VoxelCell> cells, b3Pos position, b3Quat rotation, bool dynamic,
			   b3Vec3 linearVelocity, b3Vec3 angularVelocity, std::vector<VoxelBoxRun> colliderRuns = {},
			   VoxelChunkMeshBuildBatch meshBuilds = {} );
	~VoxelBody();

	VoxelBody( const VoxelBody& ) = delete;
	VoxelBody& operator=( const VoxelBody& ) = delete;

	void RebuildColliders();
	void RebuildColliders( const std::vector<VoxelBoxRun>& runs );
	void RebuildChunkColliders( const VoxelChunkColliderBuildBatch& builds );
	void RebuildStaticMeshColliders( const VoxelChunkMeshBuildBatch& meshBuilds );
	void SetPhysicsEnabled( bool enabled );
	void SetKinematic( bool kinematic );
	bool IsKinematic() const;
	int GetFilledCount() const;
	int GetMapVoxelCount() const;
	int GetRenderedVoxelCount() const;
	bool SetVoxel( Int3 voxel, VoxelCell value );
	bool SetVoxelFilled( Int3 voxel, bool filled );
	bool MarkVoxelDirty( Int3 voxel );
	bool MarkChunkDirty( Int3 chunk );
	int GetPendingChunkCount() const;
	b3Vec3 VoxelToLocalPosition( Int3 voxel ) const;
	b3Pos VoxelToWorldPosition( Int3 voxel ) const;
	Int3 LocalPositionToVoxel( b3Vec3 localPosition ) const;
	Int3 WorldPositionToVoxel( b3Pos worldPosition ) const;
	void CommitVoxelUpdates();
	void ApplyPreparedUpdate( const VoxelChunkColliderBuildBatch& colliderBuilds, VoxelChunkMeshBuildBatch meshBuilds );
	void SetActive( bool active );
	void Draw() const;

	b3WorldId m_worldId;
	Int3 m_dimensions;
	std::vector<VoxelCell> m_cells;
	std::unique_ptr<VoxelChunkMeshCache> m_meshCache;
	std::vector<b3ShapeId> m_colliderShapes;
	std::vector<b3MeshData*> m_collisionMeshes;
	b3BodyId m_bodyId = b3_nullBodyId;
	bool m_dynamic = false;
	uint64_t m_id = 0;
	uint64_t m_version = 1;
	bool m_updating = false;
	bool m_updatePending = false;
	bool m_visible = true;
	int m_boxCount = 0;
	bool m_dynamicCollidersChunked = false;
	int m_offscreenFrames = 0;

private:
	struct StaticChunkCollider
	{
		b3ShapeId shapeId = b3_nullShapeId;
		b3MeshData* mesh = nullptr;
	};

	void ClearColliders();
	void ClearStaticChunkCollider( StaticChunkCollider& collider );

	int m_filledCount = 0;
	std::unordered_map<int, std::vector<b3ShapeId>> m_dynamicChunkColliderShapes;
	std::unordered_map<int, StaticChunkCollider> m_staticChunkColliders;
	bool m_kinematic = false;
	b3Vec3 m_savedLinearVelocity = b3Vec3_zero;
	b3Vec3 m_savedAngularVelocity = b3Vec3_zero;
};

struct VoxelPieceBuild
{
	Int3 dimensions;
	std::vector<VoxelCell> cells;
	b3Pos position;
	b3Quat rotation;
	b3Vec3 linearVelocity;
	b3Vec3 angularVelocity;
	bool dynamic;
	std::vector<VoxelBoxRun> colliderRuns;
	VoxelChunkMeshBuildBatch meshBuilds;
};
