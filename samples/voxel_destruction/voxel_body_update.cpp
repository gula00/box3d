// SPDX-FileCopyrightText: 2026
// SPDX-License-Identifier: MIT

#include "voxel_destruction/voxel_body_update.h"

#include "voxel_destruction/voxel_collider.h"

#include <algorithm>
#include <utility>

VoxelBodyUpdateManager::VoxelBodyUpdateManager( VoxelJobSystem& jobs, std::vector<std::unique_ptr<VoxelBody>>& bodies )
	: m_jobs( jobs ), m_bodies( bodies )
{
}

VoxelBody* VoxelBodyUpdateManager::FindBody( uint64_t id ) const
{
	auto iterator = std::find_if( m_bodies.begin(), m_bodies.end(),
								  [id]( const std::unique_ptr<VoxelBody>& body ) { return body->m_id == id; } );
	return iterator == m_bodies.end() ? nullptr : iterator->get();
}

void VoxelBodyUpdateManager::RequestUpdate( VoxelBody& body, Completion completion, bool immediate )
{
	auto [iterator, inserted] = m_states.try_emplace( body.m_id );
	State& state = iterator->second;
	if ( inserted == false )
	{
		state.rerun = true;
		state.nextImmediate = state.nextImmediate || immediate;
		if ( completion )
		{
			state.nextCallbacks.push_back( std::move( completion ) );
		}
		return;
	}

	if ( completion )
	{
		state.currentCallbacks.push_back( std::move( completion ) );
	}
	StartUpdate( body, state, immediate );
}

void VoxelBodyUpdateManager::RequestUpdates( const std::vector<VoxelBody*>& bodies, std::function<void()> completion,
											 bool immediate )
{
	int validCount = 0;
	for ( VoxelBody* body : bodies )
	{
		validCount += body != nullptr;
	}
	if ( validCount == 0 )
	{
		if ( completion )
		{
			completion();
		}
		return;
	}

	auto remaining = std::make_shared<int>( validCount );
	for ( VoxelBody* body : bodies )
	{
		if ( body == nullptr )
		{
			continue;
		}
		RequestUpdate(
			*body,
			[remaining, completion]( VoxelBody*, bool )
			{
				*remaining -= 1;
				if ( *remaining == 0 && completion )
				{
					completion();
				}
			},
			immediate );
	}
}

void VoxelBodyUpdateManager::StartUpdate( VoxelBody& body, State& state, bool immediate )
{
	body.m_updating = true;
	uint64_t bodyId = body.m_id;
	uint64_t version = body.m_version;
	Int3 dimensions = body.m_dimensions;
	std::vector<int> dirtyChunks = body.m_meshCache->GetDirtyChunkIndices();
	bool dynamic = body.m_dynamic;
	std::vector<int> colliderChunks = dirtyChunks;
	if ( dynamic && body.m_dynamicCollidersChunked == false )
	{
		colliderChunks.resize( static_cast<size_t>( body.m_meshCache->GetChunkCount() ) );
		for ( int index = 0; index < static_cast<int>( colliderChunks.size() ); ++index )
		{
			colliderChunks[index] = index;
		}
	}

	if ( immediate )
	{
		auto result = std::make_shared<PreparedUpdate>();
		if ( dynamic )
		{
			result->colliderBuilds = BuildVoxelChunkBoxRuns( body.m_cells, dimensions, colliderChunks );
		}
		result->meshBuilds = BuildVoxelChunkMeshes( dimensions, body.m_cells, dirtyChunks );
		CompleteUpdate( bodyId, version, std::move( result ) );
		return;
	}

	std::vector<VoxelCell> cells = body.m_cells;
	auto result = std::make_shared<PreparedUpdate>();
	m_jobs.Submit(
		[result, dimensions, dynamic, cells = std::move( cells ), dirtyChunks = std::move( dirtyChunks ),
		 colliderChunks = std::move( colliderChunks )]() mutable
		{
			if ( dynamic )
			{
				result->colliderBuilds = BuildVoxelChunkBoxRuns( cells, dimensions, colliderChunks );
			}
			result->meshBuilds = BuildVoxelChunkMeshes( dimensions, cells, dirtyChunks );
		},
		[this, bodyId, version, result] { CompleteUpdate( bodyId, version, result ); } );
}

void VoxelBodyUpdateManager::CompleteUpdate( uint64_t bodyId, uint64_t version, std::shared_ptr<PreparedUpdate> result )
{
	auto stateIterator = m_states.find( bodyId );
	if ( stateIterator == m_states.end() )
	{
		return;
	}

	VoxelBody* body = FindBody( bodyId );
	bool applied = body != nullptr && body->m_version == version;
	if ( applied )
	{
		body->ApplyPreparedUpdate( result->colliderBuilds, std::move( result->meshBuilds ) );
	}

	State& state = stateIterator->second;
	std::vector<Completion> callbacks = std::move( state.currentCallbacks );
	for ( Completion& callback : callbacks )
	{
		callback( body, applied );
	}

	stateIterator = m_states.find( bodyId );
	if ( stateIterator == m_states.end() )
	{
		return;
	}
	body = FindBody( bodyId );
	State& refreshedState = stateIterator->second;
	if ( body != nullptr && ( refreshedState.rerun || body->m_updatePending ) )
	{
		bool immediate = refreshedState.nextImmediate;
		refreshedState.rerun = false;
		refreshedState.nextImmediate = false;
		refreshedState.currentCallbacks = std::move( refreshedState.nextCallbacks );
		StartUpdate( *body, refreshedState, immediate );
		return;
	}

	if ( body != nullptr )
	{
		body->m_updating = false;
	}
	std::vector<Completion> finalCallbacks = std::move( refreshedState.nextCallbacks );
	m_states.erase( stateIterator );
	for ( Completion& callback : finalCallbacks )
	{
		callback( body, false );
	}
}

void VoxelBodyUpdateManager::CancelAll()
{
	for ( const auto& entry : m_states )
	{
		VoxelBody* body = FindBody( entry.first );
		if ( body != nullptr )
		{
			body->m_updating = false;
		}
	}
	m_states.clear();
}

int VoxelBodyUpdateManager::GetUpdatingBodyCount() const
{
	return static_cast<int>( m_states.size() );
}
