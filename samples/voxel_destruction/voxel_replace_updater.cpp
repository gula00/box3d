// SPDX-FileCopyrightText: 2026
// SPDX-License-Identifier: MIT

#include "voxel_destruction/voxel_replace_updater.h"

#include <algorithm>
#include <utility>

VoxelReplaceUpdater::VoxelReplaceUpdater( b3WorldId worldId, std::vector<std::unique_ptr<VoxelBody>>& bodies )
	: m_worldId( worldId ), m_bodies( bodies )
{
}

VoxelBody* VoxelReplaceUpdater::CreatePiece( VoxelPieceBuild& piece, bool active )
{
	auto body = std::make_unique<VoxelBody>( m_worldId, piece.dimensions, std::move( piece.cells ), piece.position,
											piece.rotation, piece.dynamic, piece.linearVelocity, piece.angularVelocity,
											std::move( piece.colliderRuns ), std::move( piece.meshBuilds ) );
	VoxelBody* result = body.get();
	m_bodies.push_back( std::move( body ) );
	if ( active == false )
	{
		result->SetActive( false );
	}
	return result;
}

void VoxelReplaceUpdater::EraseBody( uint64_t id )
{
	auto iterator = std::find_if( m_bodies.begin(), m_bodies.end(),
								  [id]( const std::unique_ptr<VoxelBody>& body ) { return body->m_id == id; } );
	if ( iterator != m_bodies.end() )
	{
		m_bodies.erase( iterator );
	}
}

VoxelBody* VoxelReplaceUpdater::FindBody( uint64_t id ) const
{
	auto iterator = std::find_if( m_bodies.begin(), m_bodies.end(),
								  [id]( const std::unique_ptr<VoxelBody>& body ) { return body->m_id == id; } );
	return iterator == m_bodies.end() ? nullptr : iterator->get();
}

void VoxelReplaceUpdater::Replace( VoxelBody* existingBody, std::vector<VoxelPieceBuild> pieces, VoxelReplaceMode mode,
								   Completion completion )
{
	if ( existingBody == nullptr )
	{
		if ( completion )
		{
			completion();
		}
		return;
	}

	PendingReplacement replacement;
	replacement.existingBodyId = existingBody->m_id;
	replacement.pieces = std::move( pieces );
	replacement.mode = mode;
	replacement.completion = std::move( completion );

	if ( mode == VoxelReplaceMode::immediate )
	{
		EraseBody( replacement.existingBodyId );
		for ( VoxelPieceBuild& piece : replacement.pieces )
		{
			CreatePiece( piece, true );
		}
		if ( replacement.completion )
		{
			replacement.completion();
		}
		return;
	}

	if ( mode == VoxelReplaceMode::standard )
	{
		// Mirrors StandardVoxelReplaceUpdater: existing data changes are applied
		// immediately while new object updates may still be in flight.
		EraseBody( replacement.existingBodyId );
	}

	m_pending.push_back( std::move( replacement ) );
}

void VoxelReplaceUpdater::FinishFrontReplacement()
{
	PendingReplacement replacement = std::move( m_pending.front() );
	m_pending.pop_front();

	if ( replacement.mode == VoxelReplaceMode::antiFlicker )
	{
		EraseBody( replacement.existingBodyId );
		for ( uint64_t id : replacement.createdBodyIds )
		{
			VoxelBody* body = FindBody( id );
			if ( body != nullptr )
			{
				body->SetActive( true );
			}
		}
	}

	if ( replacement.completion )
	{
		replacement.completion();
	}
}

void VoxelReplaceUpdater::Update()
{
	if ( m_pending.empty() )
	{
		return;
	}

	PendingReplacement& replacement = m_pending.front();
	int remainingBudget = m_bodiesPerFrame;
	while ( remainingBudget > 0 && replacement.nextPiece < replacement.pieces.size() )
	{
		bool active = replacement.mode != VoxelReplaceMode::antiFlicker;
		VoxelBody* body = CreatePiece( replacement.pieces[replacement.nextPiece], active );
		replacement.createdBodyIds.push_back( body->m_id );
		replacement.nextPiece += 1;
		remainingBudget -= 1;
	}

	if ( replacement.nextPiece == replacement.pieces.size() )
	{
		FinishFrontReplacement();
	}
}

void VoxelReplaceUpdater::CancelAll()
{
	for ( const PendingReplacement& replacement : m_pending )
	{
		for ( uint64_t id : replacement.createdBodyIds )
		{
			EraseBody( id );
		}

		VoxelBody* existing = FindBody( replacement.existingBodyId );
		if ( existing != nullptr )
		{
			existing->m_updating = false;
		}
	}
	m_pending.clear();
}

bool VoxelReplaceUpdater::IsBusy() const
{
	return m_pending.empty() == false;
}

bool VoxelReplaceUpdater::OwnsBody( uint64_t id ) const
{
	for ( const PendingReplacement& replacement : m_pending )
	{
		if ( replacement.existingBodyId == id ||
			 std::find( replacement.createdBodyIds.begin(), replacement.createdBodyIds.end(), id ) !=
				 replacement.createdBodyIds.end() )
		{
			return true;
		}
	}
	return false;
}

int VoxelReplaceUpdater::GetPendingPieceCount() const
{
	int count = 0;
	for ( const PendingReplacement& replacement : m_pending )
	{
		count += static_cast<int>( replacement.pieces.size() - replacement.nextPiece );
	}
	return count;
}

void VoxelReplaceUpdater::SetBodiesPerFrame( int count )
{
	m_bodiesPerFrame = std::max( 1, count );
}
