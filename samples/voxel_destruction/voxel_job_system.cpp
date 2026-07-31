// SPDX-FileCopyrightText: 2026
// SPDX-License-Identifier: MIT

#include "voxel_destruction/voxel_job_system.h"

#include <algorithm>
#include <utility>

VoxelJobSystem::VoxelJobSystem( int workerCount )
{
	workerCount = std::max( 1, workerCount );
	m_workers.reserve( workerCount );
	for ( int i = 0; i < workerCount; ++i )
	{
		m_workers.emplace_back( &VoxelJobSystem::WorkerMain, this );
	}
}

VoxelJobSystem::~VoxelJobSystem()
{
	{
		std::lock_guard<std::mutex> lock( m_mutex );
		m_stopping = true;
	}
	m_condition.notify_all();

	for ( std::thread& worker : m_workers )
	{
		worker.join();
	}
}

void VoxelJobSystem::Submit( std::function<void()> work, std::function<void()> completion )
{
	{
		std::lock_guard<std::mutex> lock( m_mutex );
		if ( m_stopping )
		{
			return;
		}
		m_jobs.push( { std::move( work ), std::move( completion ) } );
	}
	m_condition.notify_one();
}

void VoxelJobSystem::WorkerMain()
{
	for ( ;; )
	{
		Job job;
		{
			std::unique_lock<std::mutex> lock( m_mutex );
			m_condition.wait( lock, [this] { return m_stopping || m_jobs.empty() == false; } );
			if ( m_stopping && m_jobs.empty() )
			{
				return;
			}

			job = std::move( m_jobs.front() );
			m_jobs.pop();
			m_activeJobs += 1;
		}

		job.work();

		{
			std::lock_guard<std::mutex> lock( m_mutex );
			m_activeJobs -= 1;
			m_completions.push( std::move( job.completion ) );
		}
	}
}

int VoxelJobSystem::PumpCompletions()
{
	std::queue<std::function<void()>> completions;
	{
		std::lock_guard<std::mutex> lock( m_mutex );
		std::swap( completions, m_completions );
	}

	int count = 0;
	while ( completions.empty() == false )
	{
		completions.front()();
		completions.pop();
		count += 1;
	}
	return count;
}

int VoxelJobSystem::GetPendingJobCount() const
{
	std::lock_guard<std::mutex> lock( m_mutex );
	return static_cast<int>( m_jobs.size() ) + m_activeJobs;
}
