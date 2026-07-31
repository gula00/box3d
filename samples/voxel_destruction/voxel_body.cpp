// SPDX-FileCopyrightText: 2026
// SPDX-License-Identifier: MIT

#include "voxel_destruction/voxel_body.h"

#include "voxel_destruction/voxel_collider.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <utility>

namespace
{

std::atomic<uint64_t> nextVoxelBodyId = 1;

static b3Vec3 Add( b3Vec3 a, b3Vec3 b )
{
	return { a.x + b.x, a.y + b.y, a.z + b.z };
}

static b3Vec3 Scale( b3Vec3 v, float scale )
{
	return { scale * v.x, scale * v.y, scale * v.z };
}

} // namespace

VoxelBody::VoxelBody( b3WorldId worldId, Int3 dimensions, std::vector<VoxelCell> cells, b3Pos position, b3Quat rotation,
					  bool dynamic, b3Vec3 linearVelocity, b3Vec3 angularVelocity, std::vector<VoxelBoxRun> colliderRuns,
					  VoxelChunkMeshBuildBatch meshBuilds )
	: m_worldId( worldId ), m_dimensions( dimensions ), m_cells( std::move( cells ) ), m_dynamic( dynamic )
{
	m_id = nextVoxelBodyId.fetch_add( 1, std::memory_order_relaxed );
	m_filledCount = static_cast<int>( std::count_if(
		m_cells.begin(), m_cells.end(), []( const VoxelCell& cell ) { return cell.filled != 0; } ) );
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = dynamic ? b3_dynamicBody : b3_staticBody;
	bodyDef.position = position;
	bodyDef.rotation = rotation;
	bodyDef.name = dynamic ? "voxel fragment" : "voxel structure";
	m_bodyId = b3CreateBody( worldId, &bodyDef );
	b3Body_SetUserData( m_bodyId, this );

	if ( meshBuilds.empty() )
	{
		meshBuilds = BuildVoxelChunkMeshes( m_dimensions, m_cells );
	}

	if ( dynamic )
	{
		if ( colliderRuns.empty() )
		{
			RebuildColliders();
		}
		else
		{
			RebuildColliders( colliderRuns );
		}
	}
	else
	{
		RebuildStaticMeshColliders( meshBuilds );
	}
	if ( dynamic )
	{
		b3Body_SetLinearVelocity( m_bodyId, linearVelocity );
		b3Body_SetAngularVelocity( m_bodyId, angularVelocity );
	}

	m_meshCache = std::make_unique<VoxelChunkMeshCache>( m_dimensions, std::move( meshBuilds ) );
}

VoxelBody::~VoxelBody()
{
	ClearColliders();

	if ( B3_IS_NON_NULL( m_bodyId ) && b3Body_IsValid( m_bodyId ) )
	{
		b3DestroyBody( m_bodyId );
	}
}

void VoxelBody::ClearColliders()
{
	for ( b3ShapeId shapeId : m_colliderShapes )
	{
		if ( b3Shape_IsValid( shapeId ) )
		{
			b3DestroyShape( shapeId, false );
		}
	}
	m_colliderShapes.clear();
	for ( b3MeshData* mesh : m_collisionMeshes )
	{
		b3DestroyMesh( mesh );
	}
	m_collisionMeshes.clear();
	for ( auto& [chunkIndex, shapes] : m_dynamicChunkColliderShapes )
	{
		(void)chunkIndex;
		for ( b3ShapeId shapeId : shapes )
		{
			if ( b3Shape_IsValid( shapeId ) )
			{
				b3DestroyShape( shapeId, false );
			}
		}
	}
	m_dynamicChunkColliderShapes.clear();
	for ( auto& [chunkIndex, collider] : m_staticChunkColliders )
	{
		(void)chunkIndex;
		ClearStaticChunkCollider( collider );
	}
	m_staticChunkColliders.clear();
	m_boxCount = 0;
	m_dynamicCollidersChunked = false;
}

void VoxelBody::ClearStaticChunkCollider( StaticChunkCollider& collider )
{
	if ( b3Shape_IsValid( collider.shapeId ) )
	{
		b3DestroyShape( collider.shapeId, false );
	}
	collider.shapeId = b3_nullShapeId;
	if ( collider.mesh != nullptr )
	{
		b3DestroyMesh( collider.mesh );
	}
	collider.mesh = nullptr;
}

void VoxelBody::RebuildColliders()
{
	RebuildColliders( BuildVoxelBoxRuns( m_cells, m_dimensions ) );
}

void VoxelBody::RebuildColliders( const std::vector<VoxelBoxRun>& runs )
{
	b3Vec3 linearVelocity = m_dynamic ? b3Body_GetLinearVelocity( m_bodyId ) : b3Vec3_zero;
	b3Vec3 angularVelocity = m_dynamic ? b3Body_GetAngularVelocity( m_bodyId ) : b3Vec3_zero;
	ClearColliders();

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = kVoxelDensity;
	shapeDef.updateBodyMass = false;
	shapeDef.baseMaterial.friction = 0.7f;
	shapeDef.baseMaterial.restitution = 0.05f;
	shapeDef.baseMaterial.customColor = b3MakeDebugColor( b3_colorDarkSlateGray, b3_debugMaterialMatte );

	m_boxCount = 0;
	for ( const VoxelBoxRun& run : runs )
	{
		b3Vec3 lower = GetCellCenter( run.lower, m_dimensions );
		b3Vec3 upper = GetCellCenter( run.upper, m_dimensions );
		b3Vec3 center = Scale( Add( lower, upper ), 0.5f );
		float hx = 0.5f * ( run.upper.x - run.lower.x + 1 ) * kVoxelSize;
		float hy = 0.5f * ( run.upper.y - run.lower.y + 1 ) * kVoxelSize;
		float hz = 0.5f * ( run.upper.z - run.lower.z + 1 ) * kVoxelSize;
		b3BoxHull box = b3MakeOffsetBoxHull( hx, hy, hz, center );
		m_colliderShapes.push_back( b3CreateHullShape( m_bodyId, &shapeDef, &box.base ) );
		m_boxCount += 1;
	}

	if ( m_dynamic )
	{
		b3Body_ApplyMassFromShapes( m_bodyId );
		b3Body_SetLinearVelocity( m_bodyId, linearVelocity );
		b3Body_SetAngularVelocity( m_bodyId, angularVelocity );
	}
}

void VoxelBody::RebuildChunkColliders( const VoxelChunkColliderBuildBatch& builds )
{
	b3Vec3 linearVelocity = b3Body_GetLinearVelocity( m_bodyId );
	b3Vec3 angularVelocity = b3Body_GetAngularVelocity( m_bodyId );
	if ( m_dynamicCollidersChunked == false )
	{
		ClearColliders();
		m_dynamicCollidersChunked = true;
	}

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = kVoxelDensity;
	shapeDef.updateBodyMass = false;
	shapeDef.baseMaterial.friction = 0.7f;
	shapeDef.baseMaterial.restitution = 0.05f;
	shapeDef.baseMaterial.customColor = b3MakeDebugColor( b3_colorDarkSlateGray, b3_debugMaterialMatte );

	for ( const VoxelChunkColliderBuild& build : builds )
	{
		std::vector<b3ShapeId>& shapes = m_dynamicChunkColliderShapes[build.chunkIndex];
		for ( b3ShapeId shapeId : shapes )
		{
			if ( b3Shape_IsValid( shapeId ) )
			{
				b3DestroyShape( shapeId, false );
			}
		}
		m_boxCount -= static_cast<int>( shapes.size() );
		shapes.clear();
		shapes.reserve( build.runs.size() );

		for ( const VoxelBoxRun& run : build.runs )
		{
			b3Vec3 lower = GetCellCenter( run.lower, m_dimensions );
			b3Vec3 upper = GetCellCenter( run.upper, m_dimensions );
			b3Vec3 center = Scale( Add( lower, upper ), 0.5f );
			float hx = 0.5f * ( run.upper.x - run.lower.x + 1 ) * kVoxelSize;
			float hy = 0.5f * ( run.upper.y - run.lower.y + 1 ) * kVoxelSize;
			float hz = 0.5f * ( run.upper.z - run.lower.z + 1 ) * kVoxelSize;
			b3BoxHull box = b3MakeOffsetBoxHull( hx, hy, hz, center );
			shapes.push_back( b3CreateHullShape( m_bodyId, &shapeDef, &box.base ) );
			m_boxCount += 1;
		}
	}

	b3Body_ApplyMassFromShapes( m_bodyId );
	b3Body_SetLinearVelocity( m_bodyId, linearVelocity );
	b3Body_SetAngularVelocity( m_bodyId, angularVelocity );
}

void VoxelBody::RebuildStaticMeshColliders( const VoxelChunkMeshBuildBatch& meshBuilds )
{
	m_boxCount = 0;

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.baseMaterial.friction = 0.7f;
	shapeDef.baseMaterial.restitution = 0.05f;
	shapeDef.baseMaterial.customColor = b3MakeDebugColor( b3_colorDarkSlateGray, b3_debugMaterialMatte );

	for ( const VoxelChunkMeshBuild& chunk : meshBuilds )
	{
		StaticChunkCollider& collider = m_staticChunkColliders[chunk.chunkIndex];
		ClearStaticChunkCollider( collider );
		std::vector<b3Vec3> vertices;
		std::vector<int32_t> indices;
		for ( const VoxelColoredCpuMesh& coloredMesh : chunk.coloredMeshes )
		{
			int32_t vertexOffset = static_cast<int32_t>( vertices.size() );
			vertices.reserve( vertices.size() + coloredMesh.mesh.vertices.size() );
			for ( const MeshVertex& vertex : coloredMesh.mesh.vertices )
			{
				vertices.push_back( { vertex.position[0], vertex.position[1], vertex.position[2] } );
			}
			indices.reserve( indices.size() + coloredMesh.mesh.indices.size() );
			for ( uint32_t index : coloredMesh.mesh.indices )
			{
				indices.push_back( vertexOffset + static_cast<int32_t>( index ) );
			}
		}

		if ( indices.empty() )
		{
			continue;
		}

		b3MeshDef meshDef = {};
		meshDef.vertices = vertices.data();
		meshDef.vertexCount = static_cast<int>( vertices.size() );
		meshDef.indices = indices.data();
		meshDef.triangleCount = static_cast<int>( indices.size() / 3 );
		meshDef.weldVertices = true;
		meshDef.weldTolerance = 0.001f * kVoxelSize;
		meshDef.useMedianSplit = true;
		meshDef.identifyEdges = true;
		b3MeshData* mesh = b3CreateMesh( &meshDef, nullptr, 0 );
		if ( mesh == nullptr )
		{
			continue;
		}

		collider.mesh = mesh;
		collider.shapeId = b3CreateMeshShape( m_bodyId, &shapeDef, mesh, b3Vec3_one );
	}
}

void VoxelBody::SetPhysicsEnabled( bool enabled )
{
	if ( enabled == m_dynamic || B3_IS_NULL( m_bodyId ) || b3Body_IsValid( m_bodyId ) == false )
	{
		return;
	}

	ClearColliders();
	m_dynamic = enabled;
	m_kinematic = false;
	b3Body_SetType( m_bodyId, enabled ? b3_dynamicBody : b3_staticBody );
	if ( enabled )
	{
		RebuildColliders();
	}
	else
	{
		VoxelChunkMeshBuildBatch meshBuilds = BuildVoxelChunkMeshes( m_dimensions, m_cells );
		RebuildStaticMeshColliders( meshBuilds );
	}
	m_version += 1;
}

void VoxelBody::SetKinematic( bool kinematic )
{
	if ( m_dynamic == false || kinematic == m_kinematic || B3_IS_NULL( m_bodyId ) ||
		 b3Body_IsValid( m_bodyId ) == false )
	{
		return;
	}

	if ( kinematic )
	{
		m_savedLinearVelocity = b3Body_GetLinearVelocity( m_bodyId );
		m_savedAngularVelocity = b3Body_GetAngularVelocity( m_bodyId );
		b3Body_SetType( m_bodyId, b3_kinematicBody );
	}
	else
	{
		b3Body_SetType( m_bodyId, b3_dynamicBody );
		b3Body_SetLinearVelocity( m_bodyId, m_savedLinearVelocity );
		b3Body_SetAngularVelocity( m_bodyId, m_savedAngularVelocity );
	}
	m_kinematic = kinematic;
}

bool VoxelBody::IsKinematic() const
{
	return m_kinematic;
}

int VoxelBody::GetFilledCount() const
{
	return m_filledCount;
}

int VoxelBody::GetMapVoxelCount() const
{
	return static_cast<int>( m_cells.size() );
}

int VoxelBody::GetRenderedVoxelCount() const
{
	return GetFilledCount();
}

bool VoxelBody::SetVoxel( Int3 voxel, VoxelCell value )
{
	if ( InBounds( voxel, m_dimensions ) == false )
	{
		return false;
	}

	value.filled = value.filled != 0 ? 1 : 0;
	VoxelCell& cell = m_cells[GetIndex( voxel, m_dimensions )];
	if ( cell.filled == value.filled && cell.material == value.material && cell.r == value.r && cell.g == value.g &&
		 cell.b == value.b && cell.a == value.a )
	{
		return false;
	}

	m_filledCount += static_cast<int>( value.filled != 0 ) - static_cast<int>( cell.filled != 0 );
	cell = value;
	m_version += 1;
	m_meshCache->MarkVoxelDirty( voxel );
	m_updatePending = true;
	return true;
}

bool VoxelBody::SetVoxelFilled( Int3 voxel, bool filled )
{
	if ( InBounds( voxel, m_dimensions ) == false )
	{
		return false;
	}
	VoxelCell value = m_cells[GetIndex( voxel, m_dimensions )];
	value.filled = filled ? 1 : 0;
	return SetVoxel( voxel, value );
}

bool VoxelBody::MarkVoxelDirty( Int3 voxel )
{
	if ( InBounds( voxel, m_dimensions ) == false )
	{
		return false;
	}
	m_version += 1;
	m_meshCache->MarkVoxelDirty( voxel );
	m_updatePending = true;
	return true;
}

bool VoxelBody::MarkChunkDirty( Int3 chunk )
{
	if ( m_meshCache->MarkChunkDirty( chunk ) == false )
	{
		return false;
	}
	m_version += 1;
	m_updatePending = true;
	return true;
}

int VoxelBody::GetPendingChunkCount() const
{
	return m_meshCache->GetDirtyChunkCount();
}

b3Vec3 VoxelBody::VoxelToLocalPosition( Int3 voxel ) const
{
	return GetCellCenter( voxel, m_dimensions );
}

b3Pos VoxelBody::VoxelToWorldPosition( Int3 voxel ) const
{
	return b3Body_GetWorldPoint( m_bodyId, VoxelToLocalPosition( voxel ) );
}

Int3 VoxelBody::LocalPositionToVoxel( b3Vec3 localPosition ) const
{
	return {
		static_cast<int>( floorf( localPosition.x / kVoxelSize + 0.5f * m_dimensions.x ) ),
		static_cast<int>( floorf( localPosition.y / kVoxelSize + 0.5f * m_dimensions.y ) ),
		static_cast<int>( floorf( localPosition.z / kVoxelSize + 0.5f * m_dimensions.z ) ),
	};
}

Int3 VoxelBody::WorldPositionToVoxel( b3Pos worldPosition ) const
{
	return LocalPositionToVoxel( b3Body_GetLocalPoint( m_bodyId, worldPosition ) );
}

void VoxelBody::CommitVoxelUpdates()
{
	if ( m_updatePending == false )
	{
		return;
	}

	std::vector<int> dirtyChunks = m_meshCache->GetDirtyChunkIndices();
	VoxelChunkMeshBuildBatch meshBuilds = BuildVoxelChunkMeshes( m_dimensions, m_cells, dirtyChunks );
	if ( m_dynamic )
	{
		std::vector<int> colliderChunks = dirtyChunks;
		if ( m_dynamicCollidersChunked == false )
		{
			colliderChunks.resize( static_cast<size_t>( m_meshCache->GetChunkCount() ) );
			for ( int index = 0; index < static_cast<int>( colliderChunks.size() ); ++index )
			{
				colliderChunks[index] = index;
			}
		}
		RebuildChunkColliders( BuildVoxelChunkBoxRuns( m_cells, m_dimensions, colliderChunks ) );
	}
	else
	{
		RebuildStaticMeshColliders( meshBuilds );
	}
	m_meshCache->ApplyBuildBatch( std::move( meshBuilds ) );
	m_updatePending = false;
}

void VoxelBody::ApplyPreparedUpdate( const VoxelChunkColliderBuildBatch& colliderBuilds,
									 VoxelChunkMeshBuildBatch meshBuilds )
{
	if ( m_dynamic )
	{
		RebuildChunkColliders( colliderBuilds );
	}
	else
	{
		RebuildStaticMeshColliders( meshBuilds );
	}
	m_meshCache->ApplyBuildBatch( std::move( meshBuilds ) );
	m_updatePending = false;
}

void VoxelBody::SetActive( bool active )
{
	if ( B3_IS_NULL( m_bodyId ) || b3Body_IsValid( m_bodyId ) == false )
	{
		return;
	}

	m_visible = active;
	if ( active )
	{
		m_offscreenFrames = 0;
		b3Body_Enable( m_bodyId );
	}
	else
	{
		b3Body_Disable( m_bodyId );
	}
}

void VoxelBody::Draw() const
{
	if ( m_visible == false || B3_IS_NULL( m_bodyId ) || b3Body_IsValid( m_bodyId ) == false )
	{
		return;
	}

	m_meshCache->Draw( b3Body_GetTransform( m_bodyId ) );
}
