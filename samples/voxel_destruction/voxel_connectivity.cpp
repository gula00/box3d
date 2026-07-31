// SPDX-FileCopyrightText: 2026
// SPDX-License-Identifier: MIT

#include "voxel_destruction/voxel_connectivity.h"

#include <algorithm>
#include <atomic>
#include <thread>
#include <unordered_map>
#include <utility>

namespace
{

class UnionFind
{
public:
	explicit UnionFind( int count ) : m_parent( count ), m_size( count, 1 )
	{
		for ( int i = 0; i < count; ++i )
		{
			m_parent[i] = i;
		}
	}

	int Find( int value )
	{
		int root = value;
		while ( m_parent[root] != root )
		{
			root = m_parent[root];
		}
		while ( m_parent[value] != value )
		{
			int next = m_parent[value];
			m_parent[value] = root;
			value = next;
		}
		return root;
	}

	void Merge( int a, int b )
	{
		int rootA = Find( a );
		int rootB = Find( b );
		if ( rootA == rootB )
		{
			return;
		}

		if ( m_size[rootA] < m_size[rootB] )
		{
			std::swap( rootA, rootB );
		}
		m_parent[rootB] = rootA;
		m_size[rootA] += m_size[rootB];
	}

private:
	std::vector<int> m_parent;
	std::vector<int> m_size;
};

template <typename Function>
void ParallelFor( int count, Function function )
{
	unsigned hardwareThreads = std::thread::hardware_concurrency();
	int workerCount = std::min( count, std::min( 4, static_cast<int>( hardwareThreads == 0 ? 1 : hardwareThreads ) ) );
	if ( workerCount <= 1 || count < 16 )
	{
		for ( int index = 0; index < count; ++index )
		{
			function( index );
		}
		return;
	}

	std::atomic<int> nextIndex = 0;
	std::vector<std::thread> workers;
	workers.reserve( workerCount );
	for ( int worker = 0; worker < workerCount; ++worker )
	{
		workers.emplace_back(
			[&]
			{
				for ( ;; )
				{
					int index = nextIndex.fetch_add( 1, std::memory_order_relaxed );
					if ( index >= count )
					{
						return;
					}
					function( index );
				}
			} );
	}
	for ( std::thread& worker : workers )
	{
		worker.join();
	}
}

} // namespace

std::vector<std::vector<int>> LabelVoxelComponents( const std::vector<VoxelCell>& cells, Int3 dimensions )
{
	std::vector<int> labels( cells.size(), 0 );
	int lineCount = dimensions.x * dimensions.y;
	std::vector<int> lineLabelCounts( lineCount, 0 );

	// Match VoxelEngine's first pass: each uninterrupted run on a Z line gets
	// one provisional label. Every XY line is independent at this stage.
	ParallelFor(
		lineCount,
		[&]( int lineIndex )
		{
			int x = lineIndex / dimensions.y;
			int y = lineIndex % dimensions.y;
			int activeLabel = 0;
			int localLabelCount = 0;
			for ( int z = 0; z < dimensions.z; ++z )
			{
				int index = GetIndex( { x, y, z }, dimensions );
				if ( cells[index].filled == 0 )
				{
					activeLabel = 0;
					continue;
				}

				if ( activeLabel == 0 )
				{
					activeLabel = ++localLabelCount;
				}
				labels[index] = activeLabel;
			}
			lineLabelCounts[lineIndex] = localLabelCount;
		} );

	std::vector<int> lineLabelOffsets( lineCount, 0 );
	int labelCount = 0;
	for ( int lineIndex = 0; lineIndex < lineCount; ++lineIndex )
	{
		lineLabelOffsets[lineIndex] = labelCount;
		labelCount += lineLabelCounts[lineIndex];
	}

	if ( labelCount == 0 )
	{
		return {};
	}

	ParallelFor(
		lineCount,
		[&]( int lineIndex )
		{
			int x = lineIndex / dimensions.y;
			int y = lineIndex % dimensions.y;
			int offset = lineLabelOffsets[lineIndex];
			for ( int z = 0; z < dimensions.z; ++z )
			{
				int& label = labels[GetIndex( { x, y, z }, dimensions )];
				if ( label != 0 )
				{
					label += offset;
				}
			}
		} );

	UnionFind unionFind( labelCount + 1 );

	// Z adjacency is already represented by runs. Only compare X/Y neighbors,
	// equivalent to the label-equivalence job in the Unity implementation.
	std::vector<std::vector<std::pair<int, int>>> lineEquivalences( lineCount );
	ParallelFor(
		lineCount,
		[&]( int lineIndex )
		{
			int x = lineIndex / dimensions.y;
			int y = lineIndex % dimensions.y;
			std::vector<std::pair<int, int>>& equivalences = lineEquivalences[lineIndex];
			for ( int z = 0; z < dimensions.z; ++z )
			{
				int index = GetIndex( { x, y, z }, dimensions );
				int label = labels[index];
				if ( label == 0 )
				{
					continue;
				}

				if ( x > 0 )
				{
					int neighborLabel = labels[GetIndex( { x - 1, y, z }, dimensions )];
					if ( neighborLabel != 0 )
					{
						equivalences.emplace_back( label, neighborLabel );
					}
				}
				if ( y > 0 )
				{
					int neighborLabel = labels[GetIndex( { x, y - 1, z }, dimensions )];
					if ( neighborLabel != 0 )
					{
						equivalences.emplace_back( label, neighborLabel );
					}
				}
			}
		} );

	for ( const std::vector<std::pair<int, int>>& line : lineEquivalences )
	{
		for ( const std::pair<int, int>& equivalence : line )
		{
			unionFind.Merge( equivalence.first, equivalence.second );
		}
	}

	std::unordered_map<int, int> rootToComponent;
	std::vector<std::vector<int>> components;
	for ( int index = 0; index < static_cast<int>( labels.size() ); ++index )
	{
		if ( labels[index] == 0 )
		{
			continue;
		}

		int root = unionFind.Find( labels[index] );
		auto [iterator, inserted] = rootToComponent.emplace( root, static_cast<int>( components.size() ) );
		if ( inserted )
		{
			components.emplace_back();
		}
		components[iterator->second].push_back( index );
	}

	std::sort( components.begin(), components.end(),
			   []( const std::vector<int>& a, const std::vector<int>& b ) { return a.size() > b.size(); } );
	return components;
}

int FindMostGroundedComponent( const std::vector<std::vector<int>>& components, Int3 dimensions )
{
	for ( int y = 0; y < dimensions.y; ++y )
	{
		int bestComponent = -1;
		int bestCount = 0;
		for ( int componentIndex = 0; componentIndex < static_cast<int>( components.size() ); ++componentIndex )
		{
			int count = 0;
			for ( int voxelIndex : components[componentIndex] )
			{
				count += GetCoordinates( voxelIndex, dimensions ).y == y;
			}

			if ( count > bestCount )
			{
				bestCount = count;
				bestComponent = componentIndex;
			}
		}

		if ( bestComponent >= 0 )
		{
			return bestComponent;
		}
	}

	return components.empty() ? -1 : 0;
}
