// SPDX-FileCopyrightText: 2026
// SPDX-License-Identifier: MIT

#pragma once

#include "voxel_destruction/voxel_body.h"

#include <deque>
#include <functional>
#include <memory>
#include <vector>

enum class VoxelReplaceMode
{
	immediate,
	standard,
	antiFlicker,
};

class VoxelReplaceUpdater
{
public:
	using Completion = std::function<void()>;

	VoxelReplaceUpdater( b3WorldId worldId, std::vector<std::unique_ptr<VoxelBody>>& bodies );

	void Replace( VoxelBody* existingBody, std::vector<VoxelPieceBuild> pieces, VoxelReplaceMode mode,
				  Completion completion = {} );
	void Update();
	void CancelAll();

	bool IsBusy() const;
	bool OwnsBody( uint64_t id ) const;
	int GetPendingPieceCount() const;
	void SetBodiesPerFrame( int count );

private:
	struct PendingReplacement
	{
		uint64_t existingBodyId = 0;
		std::vector<VoxelPieceBuild> pieces;
		std::vector<uint64_t> createdBodyIds;
		size_t nextPiece = 0;
		VoxelReplaceMode mode = VoxelReplaceMode::antiFlicker;
		Completion completion;
	};

	VoxelBody* CreatePiece( VoxelPieceBuild& piece, bool active );
	void EraseBody( uint64_t id );
	VoxelBody* FindBody( uint64_t id ) const;
	void FinishFrontReplacement();

	b3WorldId m_worldId;
	std::vector<std::unique_ptr<VoxelBody>>& m_bodies;
	std::deque<PendingReplacement> m_pending;
	int m_bodiesPerFrame = 2;
};
