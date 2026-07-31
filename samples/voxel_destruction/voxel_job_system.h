// SPDX-FileCopyrightText: 2026
// SPDX-License-Identifier: MIT

#pragma once

#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

class VoxelJobSystem
{
public:
	explicit VoxelJobSystem( int workerCount );
	~VoxelJobSystem();

	VoxelJobSystem( const VoxelJobSystem& ) = delete;
	VoxelJobSystem& operator=( const VoxelJobSystem& ) = delete;

	void Submit( std::function<void()> work, std::function<void()> completion );
	int PumpCompletions();
	int GetPendingJobCount() const;

private:
	struct Job
	{
		std::function<void()> work;
		std::function<void()> completion;
	};

	void WorkerMain();

	mutable std::mutex m_mutex;
	std::condition_variable m_condition;
	std::queue<Job> m_jobs;
	std::queue<std::function<void()>> m_completions;
	std::vector<std::thread> m_workers;
	bool m_stopping = false;
	int m_activeJobs = 0;
};
