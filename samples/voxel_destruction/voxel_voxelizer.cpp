// SPDX-FileCopyrightText: 2026
// SPDX-License-Identifier: MIT

#include "voxel_destruction/voxel_voxelizer.h"

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <numeric>
#include <vector>

namespace
{

static b3Vec3 Subtract( b3Vec3 a, b3Vec3 b )
{
	return { a.x - b.x, a.y - b.y, a.z - b.z };
}

static b3Vec3 Add( b3Vec3 a, b3Vec3 b )
{
	return { a.x + b.x, a.y + b.y, a.z + b.z };
}

static b3Vec3 Scale( b3Vec3 v, float scale )
{
	return { v.x * scale, v.y * scale, v.z * scale };
}

static float Dot( b3Vec3 a, b3Vec3 b )
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

static float DistanceSquared( b3Vec3 a, b3Vec3 b )
{
	b3Vec3 delta = Subtract( a, b );
	return Dot( delta, delta );
}

static float GetComponent( b3Vec3 value, int axis )
{
	return axis == 0 ? value.x : axis == 1 ? value.y : value.z;
}

b3Vec3 ClosestPointOnTriangle( b3Vec3 point, b3Vec3 a, b3Vec3 b, b3Vec3 c )
{
	b3Vec3 ab = Subtract( b, a );
	b3Vec3 ac = Subtract( c, a );
	b3Vec3 ap = Subtract( point, a );
	float d1 = Dot( ab, ap );
	float d2 = Dot( ac, ap );
	if ( d1 <= 0.0f && d2 <= 0.0f )
	{
		return a;
	}

	b3Vec3 bp = Subtract( point, b );
	float d3 = Dot( ab, bp );
	float d4 = Dot( ac, bp );
	if ( d3 >= 0.0f && d4 <= d3 )
	{
		return b;
	}

	float vc = d1 * d4 - d3 * d2;
	if ( vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f )
	{
		return Add( a, Scale( ab, d1 / ( d1 - d3 ) ) );
	}

	b3Vec3 cp = Subtract( point, c );
	float d5 = Dot( ab, cp );
	float d6 = Dot( ac, cp );
	if ( d6 >= 0.0f && d5 <= d6 )
	{
		return c;
	}

	float vb = d5 * d2 - d1 * d6;
	if ( vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f )
	{
		return Add( a, Scale( ac, d2 / ( d2 - d6 ) ) );
	}

	float va = d3 * d6 - d5 * d4;
	if ( va <= 0.0f && d4 - d3 >= 0.0f && d5 - d6 >= 0.0f )
	{
		b3Vec3 bc = Subtract( c, b );
		return Add( b, Scale( bc, ( d4 - d3 ) / ( ( d4 - d3 ) + ( d5 - d6 ) ) ) );
	}

	float denominator = 1.0f / ( va + vb + vc );
	float v = vb * denominator;
	float w = vc * denominator;
	return Add( a, Add( Scale( ab, v ), Scale( ac, w ) ) );
}

struct ClosestTriangleResult
{
	int triangle = 0;
	b3Vec3 point = b3Vec3_zero;
	float distanceSquared = FLT_MAX;
};

class TriangleBvh
{
public:
	explicit TriangleBvh( const VoxelTriangleMesh& mesh ) : m_mesh( mesh )
	{
		int triangleCount = static_cast<int>( mesh.indices.size() / 3 );
		m_triangles.resize( static_cast<size_t>( triangleCount ) );
		std::iota( m_triangles.begin(), m_triangles.end(), 0 );
		m_nodes.reserve( static_cast<size_t>( 2 * triangleCount ) );
		BuildNode( 0, triangleCount );
	}

	ClosestTriangleResult FindClosest( b3Vec3 point ) const
	{
		ClosestTriangleResult result;
		FindClosest( 0, point, &result );
		return result;
	}

private:
	struct Node
	{
		b3Vec3 lower = { FLT_MAX, FLT_MAX, FLT_MAX };
		b3Vec3 upper = { -FLT_MAX, -FLT_MAX, -FLT_MAX };
		int first = 0;
		int count = 0;
		int left = -1;
		int right = -1;
	};

	void GetTriangle( int triangle, b3Vec3* a, b3Vec3* b, b3Vec3* c ) const
	{
		*a = m_mesh.vertices[m_mesh.indices[3 * triangle + 0]];
		*b = m_mesh.vertices[m_mesh.indices[3 * triangle + 1]];
		*c = m_mesh.vertices[m_mesh.indices[3 * triangle + 2]];
	}

	b3Vec3 GetCentroid( int triangle ) const
	{
		b3Vec3 a, b, c;
		GetTriangle( triangle, &a, &b, &c );
		return Scale( Add( Add( a, b ), c ), 1.0f / 3.0f );
	}

	void IncludeTriangle( Node* node, int triangle ) const
	{
		b3Vec3 a, b, c;
		GetTriangle( triangle, &a, &b, &c );
		for ( b3Vec3 vertex : { a, b, c } )
		{
			node->lower.x = std::min( node->lower.x, vertex.x );
			node->lower.y = std::min( node->lower.y, vertex.y );
			node->lower.z = std::min( node->lower.z, vertex.z );
			node->upper.x = std::max( node->upper.x, vertex.x );
			node->upper.y = std::max( node->upper.y, vertex.y );
			node->upper.z = std::max( node->upper.z, vertex.z );
		}
	}

	int BuildNode( int first, int count )
	{
		int nodeIndex = static_cast<int>( m_nodes.size() );
		m_nodes.push_back( {} );
		m_nodes[nodeIndex].first = first;
		m_nodes[nodeIndex].count = count;
		for ( int offset = 0; offset < count; ++offset )
		{
			IncludeTriangle( &m_nodes[nodeIndex], m_triangles[first + offset] );
		}

		if ( count <= 8 )
		{
			return nodeIndex;
		}

		b3Vec3 extent = Subtract( m_nodes[nodeIndex].upper, m_nodes[nodeIndex].lower );
		int axis = extent.y > extent.x ? 1 : 0;
		if ( extent.z > GetComponent( extent, axis ) )
		{
			axis = 2;
		}
		int middle = first + count / 2;
		std::nth_element(
			m_triangles.begin() + first, m_triangles.begin() + middle, m_triangles.begin() + first + count,
			[this, axis]( int a, int b ) { return GetComponent( GetCentroid( a ), axis ) < GetComponent( GetCentroid( b ), axis ); } );
		int left = BuildNode( first, middle - first );
		int right = BuildNode( middle, first + count - middle );
		m_nodes[nodeIndex].left = left;
		m_nodes[nodeIndex].right = right;
		m_nodes[nodeIndex].count = 0;
		return nodeIndex;
	}

	static float DistanceSquaredToBounds( b3Vec3 point, const Node& node )
	{
		float distanceSquared = 0.0f;
		for ( int axis = 0; axis < 3; ++axis )
		{
			float value = GetComponent( point, axis );
			float lower = GetComponent( node.lower, axis );
			float upper = GetComponent( node.upper, axis );
			float distance = value < lower ? lower - value : value > upper ? value - upper : 0.0f;
			distanceSquared += distance * distance;
		}
		return distanceSquared;
	}

	void FindClosest( int nodeIndex, b3Vec3 point, ClosestTriangleResult* result ) const
	{
		const Node& node = m_nodes[nodeIndex];
		if ( DistanceSquaredToBounds( point, node ) > result->distanceSquared )
		{
			return;
		}

		if ( node.count > 0 )
		{
			for ( int offset = 0; offset < node.count; ++offset )
			{
				int triangle = m_triangles[node.first + offset];
				b3Vec3 a, b, c;
				GetTriangle( triangle, &a, &b, &c );
				b3Vec3 closestPoint = ClosestPointOnTriangle( point, a, b, c );
				float distanceSquared = DistanceSquared( point, closestPoint );
				if ( distanceSquared < result->distanceSquared )
				{
					result->triangle = triangle;
					result->point = closestPoint;
					result->distanceSquared = distanceSquared;
				}
			}
			return;
		}

		float leftDistance = DistanceSquaredToBounds( point, m_nodes[node.left] );
		float rightDistance = DistanceSquaredToBounds( point, m_nodes[node.right] );
		if ( rightDistance < leftDistance )
		{
			FindClosest( node.right, point, result );
			FindClosest( node.left, point, result );
		}
		else
		{
			FindClosest( node.left, point, result );
			FindClosest( node.right, point, result );
		}
	}

	const VoxelTriangleMesh& m_mesh;
	std::vector<int> m_triangles;
	std::vector<Node> m_nodes;
};

VoxelCell MakeCell( uint32_t rgba )
{
	return {
		1,
		0,
		static_cast<uint8_t>( rgba >> 24 ),
		static_cast<uint8_t>( rgba >> 16 ),
		static_cast<uint8_t>( rgba >> 8 ),
		static_cast<uint8_t>( rgba ),
	};
}

std::array<float, 3> GetTriangleBarycentric( b3Vec3 point, b3Vec3 a, b3Vec3 b, b3Vec3 c )
{
	b3Vec3 v0 = Subtract( b, a );
	b3Vec3 v1 = Subtract( c, a );
	b3Vec3 v2 = Subtract( point, a );
	float d00 = Dot( v0, v0 );
	float d01 = Dot( v0, v1 );
	float d11 = Dot( v1, v1 );
	float d20 = Dot( v2, v0 );
	float d21 = Dot( v2, v1 );
	float denominator = d00 * d11 - d01 * d01;
	if ( fabsf( denominator ) < 1.0e-12f )
	{
		return { 1.0f, 0.0f, 0.0f };
	}
	float inverseDenominator = 1.0f / denominator;
	float v = ( d11 * d20 - d01 * d21 ) * inverseDenominator;
	float w = ( d00 * d21 - d01 * d20 ) * inverseDenominator;
	return { 1.0f - v - w, v, w };
}

uint32_t SampleTexture( const VoxelTextureImage& texture, b3Vec2 uv )
{
	if ( texture.width <= 0 || texture.height <= 0 ||
		 texture.rgba.size() != static_cast<size_t>( texture.width ) * texture.height * 4 )
	{
		return 0xFFFFFFFFu;
	}

	float u = uv.x - floorf( uv.x );
	float v = uv.y - floorf( uv.y );
	int x = std::clamp( static_cast<int>( u * texture.width ), 0, texture.width - 1 );
	int y = std::clamp( static_cast<int>( ( 1.0f - v ) * texture.height ), 0, texture.height - 1 );
	size_t offset = 4 * ( static_cast<size_t>( y ) * texture.width + x );
	return static_cast<uint32_t>( texture.rgba[offset + 0] ) << 24 |
		   static_cast<uint32_t>( texture.rgba[offset + 1] ) << 16 |
		   static_cast<uint32_t>( texture.rgba[offset + 2] ) << 8 |
		   static_cast<uint32_t>( texture.rgba[offset + 3] );
}

uint32_t GetTriangleColor( const VoxelTriangleMesh& mesh, int triangle, b3Vec3 point )
{
	if ( triangle < static_cast<int>( mesh.triangleSurfaces.size() ) )
	{
		const VoxelTriangleSurface& surface = mesh.triangleSurfaces[triangle];
		if ( surface.textureIndex >= 0 && surface.textureIndex < static_cast<int>( mesh.textures.size() ) )
		{
			b3Vec3 a = mesh.vertices[mesh.indices[3 * triangle + 0]];
			b3Vec3 b = mesh.vertices[mesh.indices[3 * triangle + 1]];
			b3Vec3 c = mesh.vertices[mesh.indices[3 * triangle + 2]];
			std::array<float, 3> barycentric = GetTriangleBarycentric( point, a, b, c );
			b3Vec2 uv = {
				barycentric[0] * surface.uv[0].x + barycentric[1] * surface.uv[1].x +
					barycentric[2] * surface.uv[2].x,
				barycentric[0] * surface.uv[0].y + barycentric[1] * surface.uv[1].y +
					barycentric[2] * surface.uv[2].y,
			};
			return SampleTexture( mesh.textures[surface.textureIndex], uv );
		}
		return surface.color;
	}

	return triangle < static_cast<int>( mesh.triangleColors.size() ) ? mesh.triangleColors[triangle] : 0xC58B54FFu;
}

} // namespace

VoxelizedTriangleMesh VoxelizeTriangleMesh( const VoxelTriangleMesh& mesh, float scanVoxelSize )
{
	VoxelizedTriangleMesh result;
	if ( scanVoxelSize <= 0.0f || mesh.vertices.size() < 3 || mesh.indices.size() < 3 || mesh.indices.size() % 3 != 0 )
	{
		return result;
	}
	for ( uint32_t index : mesh.indices )
	{
		if ( index >= mesh.vertices.size() )
		{
			return result;
		}
	}

	b3Vec3 lower = { FLT_MAX, FLT_MAX, FLT_MAX };
	b3Vec3 upper = { -FLT_MAX, -FLT_MAX, -FLT_MAX };
	for ( b3Vec3 vertex : mesh.vertices )
	{
		lower.x = std::min( lower.x, vertex.x );
		lower.y = std::min( lower.y, vertex.y );
		lower.z = std::min( lower.z, vertex.z );
		upper.x = std::max( upper.x, vertex.x );
		upper.y = std::max( upper.y, vertex.y );
		upper.z = std::max( upper.z, vertex.z );
	}

	result.sourceBoundsLower = lower;
	result.sourceBoundsUpper = upper;
	result.dimensions = {
		std::max( 1, static_cast<int>( ceilf( ( upper.x - lower.x ) / scanVoxelSize ) ) ),
		std::max( 1, static_cast<int>( ceilf( ( upper.y - lower.y ) / scanVoxelSize ) ) ),
		std::max( 1, static_cast<int>( ceilf( ( upper.z - lower.z ) / scanVoxelSize ) ) ),
	};
	int64_t voxelCount = static_cast<int64_t>( result.dimensions.x ) * result.dimensions.y * result.dimensions.z;
	if ( voxelCount > 4 * 1024 * 1024 )
	{
		result.dimensions = {};
		return result;
	}
	result.cells.resize( static_cast<size_t>( voxelCount ) );

	float surfaceDistanceSquared = 0.75f * scanVoxelSize * scanVoxelSize;
	TriangleBvh bvh( mesh );
	std::vector<int> nearestTriangles( static_cast<size_t>( voxelCount ), 0 );
	for ( int x = 0; x < result.dimensions.x; ++x )
	{
		for ( int y = 0; y < result.dimensions.y; ++y )
		{
			for ( int z = 0; z < result.dimensions.z; ++z )
			{
				b3Vec3 center = {
					lower.x + ( x + 0.5f ) * scanVoxelSize,
					lower.y + ( y + 0.5f ) * scanVoxelSize,
					lower.z + ( z + 0.5f ) * scanVoxelSize,
				};

				ClosestTriangleResult closest = bvh.FindClosest( center );
				int voxelIndex = GetIndex( { x, y, z }, result.dimensions );
				nearestTriangles[voxelIndex] = closest.triangle;
				if ( closest.distanceSquared <= surfaceDistanceSquared )
				{
					uint32_t rgba = GetTriangleColor( mesh, closest.triangle, closest.point );
					result.cells[voxelIndex] = MakeCell( rgba );
				}
			}
		}
	}

	// A surface shell separates the solid interior from empty exterior space.
	// Flooding from the grid boundary avoids six ray casts per cell and handles
	// duplicate/coplanar source triangles without parity ambiguities.
	std::vector<uint8_t> exterior( static_cast<size_t>( voxelCount ), 0 );
	std::vector<int> queue;
	queue.reserve( static_cast<size_t>( voxelCount / 4 ) );
	for ( int x = 0; x < result.dimensions.x; ++x )
	{
		for ( int y = 0; y < result.dimensions.y; ++y )
		{
			for ( int z = 0; z < result.dimensions.z; ++z )
			{
				if ( x != 0 && x != result.dimensions.x - 1 && y != 0 && y != result.dimensions.y - 1 && z != 0 &&
					 z != result.dimensions.z - 1 )
				{
					continue;
				}
				int index = GetIndex( { x, y, z }, result.dimensions );
				if ( result.cells[index].filled == 0 && exterior[index] == 0 )
				{
					exterior[index] = 1;
					queue.push_back( index );
				}
			}
		}
	}

	constexpr Int3 neighbors[6] = {
		{ 1, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 }, { 0, -1, 0 }, { 0, 0, 1 }, { 0, 0, -1 },
	};
	for ( size_t cursor = 0; cursor < queue.size(); ++cursor )
	{
		Int3 voxel = GetCoordinates( queue[cursor], result.dimensions );
		for ( Int3 offset : neighbors )
		{
			Int3 neighbor = { voxel.x + offset.x, voxel.y + offset.y, voxel.z + offset.z };
			if ( InBounds( neighbor, result.dimensions ) == false )
			{
				continue;
			}
			int neighborIndex = GetIndex( neighbor, result.dimensions );
			if ( result.cells[neighborIndex].filled == 0 && exterior[neighborIndex] == 0 )
			{
				exterior[neighborIndex] = 1;
				queue.push_back( neighborIndex );
			}
		}
	}

	for ( int index = 0; index < static_cast<int>( result.cells.size() ); ++index )
	{
		if ( result.cells[index].filled != 0 || exterior[index] != 0 )
		{
			continue;
		}
		Int3 voxel = GetCoordinates( index, result.dimensions );
		b3Vec3 center = {
			lower.x + ( voxel.x + 0.5f ) * scanVoxelSize,
			lower.y + ( voxel.y + 0.5f ) * scanVoxelSize,
			lower.z + ( voxel.z + 0.5f ) * scanVoxelSize,
		};
		int triangle = nearestTriangles[index];
		b3Vec3 a = mesh.vertices[mesh.indices[3 * triangle + 0]];
		b3Vec3 b = mesh.vertices[mesh.indices[3 * triangle + 1]];
		b3Vec3 c = mesh.vertices[mesh.indices[3 * triangle + 2]];
		b3Vec3 closestPoint = ClosestPointOnTriangle( center, a, b, c );
		result.cells[index] = MakeCell( GetTriangleColor( mesh, triangle, closestPoint ) );
	}
	return result;
}
