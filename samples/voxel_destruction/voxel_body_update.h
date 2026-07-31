// SPDX-FileCopyrightText: 2026
// SPDX-License-Identifier: MIT

#pragma once

#include "voxel_destruction/voxel_body.h"
#include "voxel_destruction/voxel_job_system.h"

#include <functional>
#include <memory>
#include <unordered_map>
#include <vector>

class VoxelBodyUpdateManager
{
public:
	using Completion = std::function<void( VoxelBody* body, bool applied )>;

	VoxelBodyUpdateManager( VoxelJobSystem& jobs, std::vector<std::unique_ptr<VoxelBody>>& bodies );

	void RequestUpdate( VoxelBody& body, Completion completion = {}, bool immediate = false );
	void RequestUpdates( const std::vector<VoxelBody*>& bodies, std::function<void()> completion = {},
						 bool immediate = false );
	void CancelAll();
	int GetUpdatingBodyCount() const;

private:
	struct PreparedUpdate
	{
		VoxelChunkColliderBuildBatch colliderBuilds;
		VoxelChunkMeshBuildBatch meshBuilds;
	};

	struct State
	{
		bool rerun = false;
		bool nextImmediate = false;
		std::vector<Completion> currentCallbacks;
		std::vector<Completion> nextCallbacks;
	};

	VoxelBody* FindBody( uint64_t id ) const;
	void StartUpdate( VoxelBody& body, State& state, bool immediate );
	void CompleteUpdate( uint64_t bodyId, uint64_t version, std::shared_ptr<PreparedUpdate> result );

	VoxelJobSystem& m_jobs;
	std::vector<std::unique_ptr<VoxelBody>>& m_bodies;
	std::unordered_map<uint64_t, State> m_states;
};
