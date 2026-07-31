// SPDX-FileCopyrightText: 2026
// SPDX-License-Identifier: MIT

#include "voxel_destruction/voxel_serialization.h"

#include "voxel_destruction/voxel_deflate.h"

#include <cctype>
#include <cstdlib>
#include <cstdint>
#include <fstream>
#include <filesystem>
#include <iterator>
#include <limits>

namespace
{

constexpr uint32_t kMagic = 0x58563342u; // B3VX
constexpr uint32_t kVersion = 1;

#pragma pack( push, 1 )
struct FileHeader
{
	uint32_t magic;
	uint32_t version;
	int32_t dimensions[3];
	uint64_t voxelCount;
	uint64_t runCount;
};

struct CellBytes
{
	uint8_t filled;
	uint8_t material;
	uint8_t r;
	uint8_t g;
	uint8_t b;
	uint8_t a;
};

struct Run
{
	uint32_t count;
	CellBytes cell;
};
#pragma pack( pop )

CellBytes ToBytes( const VoxelCell& cell )
{
	return { cell.filled, cell.material, cell.r, cell.g, cell.b, cell.a };
}

VoxelCell FromBytes( CellBytes bytes )
{
	return { bytes.filled, bytes.material, bytes.r, bytes.g, bytes.b, bytes.a };
}

bool Equal( CellBytes a, CellBytes b )
{
	return a.filled == b.filled && a.material == b.material && a.r == b.r && a.g == b.g && a.b == b.b && a.a == b.a;
}

void SetError( std::string* error, const char* message )
{
	if ( error != nullptr )
	{
		*error = message;
	}
}

} // namespace

std::vector<uint8_t> EncodeUnityVoxelPayload( const std::vector<VoxelCell>& cells )
{
	std::vector<uint8_t> uncompressed;
	uncompressed.reserve( 5 * cells.size() );
	for ( const VoxelCell& cell : cells )
	{
		uncompressed.push_back( cell.filled != 0 ? 1 : 0 );
		uncompressed.push_back( cell.r );
		uncompressed.push_back( cell.g );
		uncompressed.push_back( cell.b );
		uncompressed.push_back( cell.a );
	}
	return DeflateVoxelStoredBlocks( uncompressed );
}

bool DecodeUnityVoxelPayload( const std::vector<uint8_t>& compressed, Int3 dimensions, std::vector<VoxelCell>* cells,
							  std::string* error )
{
	if ( cells == nullptr || dimensions.x <= 0 || dimensions.y <= 0 || dimensions.z <= 0 )
	{
		SetError( error, "invalid Unity voxel payload dimensions" );
		return false;
	}

	uint64_t voxelCount = static_cast<uint64_t>( dimensions.x ) * dimensions.y * dimensions.z;
	if ( voxelCount > static_cast<uint64_t>( std::numeric_limits<int>::max() ) )
	{
		SetError( error, "Unity voxel payload is too large" );
		return false;
	}
	size_t expectedByteCount = static_cast<size_t>( 5 * voxelCount );
	std::vector<uint8_t> uncompressed;
	if ( InflateVoxelDeflate( compressed, expectedByteCount, &uncompressed, error ) == false )
	{
		return false;
	}
	if ( uncompressed.size() != expectedByteCount )
	{
		SetError( error, "Unity voxel payload size mismatch" );
		return false;
	}

	cells->resize( static_cast<size_t>( voxelCount ) );
	for ( size_t voxelIndex = 0; voxelIndex < static_cast<size_t>( voxelCount ); ++voxelIndex )
	{
		size_t offset = 5 * voxelIndex;
		( *cells )[voxelIndex] = {
			static_cast<uint8_t>( uncompressed[offset] == 1 ),
			0,
			uncompressed[offset + 1],
			uncompressed[offset + 2],
			uncompressed[offset + 3],
			uncompressed[offset + 4],
		};
	}
	return true;
}

bool LoadUnitySerializedVoxelMapAsset( const char* path, Int3* dimensions, std::vector<VoxelCell>* cells,
									  std::string* error )
{
	if ( path == nullptr || dimensions == nullptr || cells == nullptr )
	{
		SetError( error, "invalid Unity asset arguments" );
		return false;
	}

	std::ifstream stream( path, std::ios::binary );
	if ( stream.is_open() == false )
	{
		SetError( error, "could not open Unity voxel asset" );
		return false;
	}
	std::string text{ std::istreambuf_iterator<char>( stream ), std::istreambuf_iterator<char>() };

	size_t dimensionsKey = text.find( "voxelMapDimensions:" );
	size_t payloadKey = text.find( "compressedVoxelDataValues:" );
	if ( dimensionsKey == std::string::npos || payloadKey == std::string::npos || payloadKey <= dimensionsKey )
	{
		SetError( error, "Unity voxel asset fields were not found" );
		return false;
	}

	auto parseDimension = [&]( const char* name, int* value )
	{
		size_t position = text.find( name, dimensionsKey );
		if ( position == std::string::npos || position >= payloadKey )
		{
			return false;
		}
		position += 2;
		while ( position < payloadKey && std::isspace( static_cast<unsigned char>( text[position] ) ) )
		{
			position += 1;
		}
		char* end = nullptr;
		long parsed = std::strtol( text.c_str() + position, &end, 10 );
		if ( end == text.c_str() + position || parsed <= 0 || parsed > std::numeric_limits<int>::max() )
		{
			return false;
		}
		*value = static_cast<int>( parsed );
		return true;
	};

	Int3 parsedDimensions;
	if ( parseDimension( "x:", &parsedDimensions.x ) == false || parseDimension( "y:", &parsedDimensions.y ) == false ||
		 parseDimension( "z:", &parsedDimensions.z ) == false )
	{
		SetError( error, "invalid Unity voxel asset dimensions" );
		return false;
	}

	size_t payloadStart = payloadKey + sizeof( "compressedVoxelDataValues:" ) - 1;
	size_t payloadEnd = text.find_first_of( "\r\n", payloadStart );
	if ( payloadEnd == std::string::npos )
	{
		payloadEnd = text.size();
	}
	std::vector<uint8_t> compressed;
	int highNibble = -1;
	for ( size_t index = payloadStart; index < payloadEnd; ++index )
	{
		unsigned char character = static_cast<unsigned char>( text[index] );
		if ( std::isspace( character ) || character == '\'' || character == '"' )
		{
			continue;
		}
		int nibble = -1;
		if ( character >= '0' && character <= '9' )
		{
			nibble = character - '0';
		}
		else if ( character >= 'a' && character <= 'f' )
		{
			nibble = character - 'a' + 10;
		}
		else if ( character >= 'A' && character <= 'F' )
		{
			nibble = character - 'A' + 10;
		}
		else
		{
			SetError( error, "invalid Unity voxel asset hex payload" );
			return false;
		}

		if ( highNibble < 0 )
		{
			highNibble = nibble;
		}
		else
		{
			compressed.push_back( static_cast<uint8_t>( ( highNibble << 4 ) | nibble ) );
			highNibble = -1;
		}
	}
	if ( highNibble >= 0 || compressed.empty() )
	{
		SetError( error, "truncated Unity voxel asset hex payload" );
		return false;
	}

	if ( DecodeUnityVoxelPayload( compressed, parsedDimensions, cells, error ) == false )
	{
		return false;
	}
	*dimensions = parsedDimensions;
	return true;
}

bool SaveUnitySerializedVoxelMapAsset( const char* path, Int3 dimensions, const std::vector<VoxelCell>& cells,
									  std::string* error )
{
	uint64_t expectedCount = static_cast<uint64_t>( dimensions.x ) * dimensions.y * dimensions.z;
	if ( path == nullptr || path[0] == '\0' || dimensions.x <= 0 || dimensions.y <= 0 || dimensions.z <= 0 ||
		 expectedCount != cells.size() )
	{
		SetError( error, "invalid Unity voxel asset arguments" );
		return false;
	}

	std::vector<uint8_t> compressed = EncodeUnityVoxelPayload( cells );
	if ( compressed.empty() )
	{
		SetError( error, "could not encode Unity voxel payload" );
		return false;
	}

	std::ofstream stream( path, std::ios::binary | std::ios::trunc );
	if ( stream.is_open() == false )
	{
		SetError( error, "could not open Unity voxel asset for writing" );
		return false;
	}

	std::string assetName = std::filesystem::path( path ).stem().string();
	stream << "%YAML 1.1\n"
			  "%TAG !u! tag:unity3d.com,2011:\n"
			  "--- !u!114 &11400000\n"
			  "MonoBehaviour:\n"
			  "  m_ObjectHideFlags: 0\n"
			  "  m_CorrespondingSourceObject: {fileID: 0}\n"
			  "  m_PrefabInstance: {fileID: 0}\n"
			  "  m_PrefabAsset: {fileID: 0}\n"
			  "  m_GameObject: {fileID: 0}\n"
			  "  m_Enabled: 1\n"
			  "  m_EditorHideFlags: 0\n"
			  "  m_Script: {fileID: 11500000, guid: 347419e1422c35a4194f363558eccfd5, type: 3}\n"
		   << "  m_Name: " << assetName
		   << "\n"
			  "  m_EditorClassIdentifier:\n"
			  "  voxelMapDimensions:\n"
		   << "    x: " << dimensions.x << "\n"
		   << "    y: " << dimensions.y << "\n"
		   << "    z: " << dimensions.z << "\n"
			  "  compressedVoxelDataValues: ";
	constexpr char hex[] = "0123456789abcdef";
	for ( uint8_t byte : compressed )
	{
		stream.put( hex[byte >> 4] );
		stream.put( hex[byte & 0xF] );
	}
	stream.put( '\n' );
	if ( stream.good() == false )
	{
		SetError( error, "failed while writing Unity voxel asset" );
		return false;
	}
	return true;
}

bool SaveVoxelMap( const char* path, Int3 dimensions, const std::vector<VoxelCell>& cells, std::string* error )
{
	if ( dimensions.x <= 0 || dimensions.y <= 0 || dimensions.z <= 0 ||
		 static_cast<uint64_t>( dimensions.x ) * dimensions.y * dimensions.z != cells.size() )
	{
		SetError( error, "invalid voxel dimensions" );
		return false;
	}

	std::vector<Run> runs;
	runs.reserve( cells.size() / 4 + 1 );
	for ( size_t index = 0; index < cells.size(); )
	{
		CellBytes value = ToBytes( cells[index] );
		uint32_t count = 1;
		while ( index + count < cells.size() && count < std::numeric_limits<uint32_t>::max() &&
				Equal( value, ToBytes( cells[index + count] ) ) )
		{
			count += 1;
		}
		runs.push_back( { count, value } );
		index += count;
	}

	FileHeader header = {
		kMagic,
		kVersion,
		{ dimensions.x, dimensions.y, dimensions.z },
		static_cast<uint64_t>( cells.size() ),
		static_cast<uint64_t>( runs.size() ),
	};

	std::ofstream stream( path, std::ios::binary | std::ios::trunc );
	if ( stream.is_open() == false )
	{
		SetError( error, "could not open voxel file for writing" );
		return false;
	}
	stream.write( reinterpret_cast<const char*>( &header ), sizeof( header ) );
	stream.write( reinterpret_cast<const char*>( runs.data() ), static_cast<std::streamsize>( runs.size() * sizeof( Run ) ) );
	if ( stream.good() == false )
	{
		SetError( error, "failed while writing voxel file" );
		return false;
	}
	return true;
}

bool LoadVoxelMap( const char* path, Int3* dimensions, std::vector<VoxelCell>* cells, std::string* error )
{
	if ( dimensions == nullptr || cells == nullptr )
	{
		SetError( error, "null voxel output" );
		return false;
	}

	std::ifstream stream( path, std::ios::binary );
	if ( stream.is_open() == false )
	{
		SetError( error, "could not open voxel file" );
		return false;
	}

	FileHeader header = {};
	stream.read( reinterpret_cast<char*>( &header ), sizeof( header ) );
	if ( stream.good() == false || header.magic != kMagic || header.version != kVersion )
	{
		SetError( error, "invalid voxel file header" );
		return false;
	}

	Int3 fileDimensions = { header.dimensions[0], header.dimensions[1], header.dimensions[2] };
	uint64_t expectedCount = static_cast<uint64_t>( fileDimensions.x ) * fileDimensions.y * fileDimensions.z;
	if ( fileDimensions.x <= 0 || fileDimensions.y <= 0 || fileDimensions.z <= 0 || expectedCount != header.voxelCount ||
		 header.voxelCount > static_cast<uint64_t>( std::numeric_limits<int>::max() ) )
	{
		SetError( error, "invalid voxel file dimensions" );
		return false;
	}

	std::vector<VoxelCell> output;
	output.reserve( static_cast<size_t>( header.voxelCount ) );
	for ( uint64_t runIndex = 0; runIndex < header.runCount; ++runIndex )
	{
		Run run = {};
		stream.read( reinterpret_cast<char*>( &run ), sizeof( run ) );
		if ( stream.good() == false || run.count == 0 || output.size() + run.count > header.voxelCount )
		{
			SetError( error, "invalid voxel run data" );
			return false;
		}

		output.insert( output.end(), run.count, FromBytes( run.cell ) );
	}

	if ( output.size() != header.voxelCount )
	{
		SetError( error, "voxel data is truncated" );
		return false;
	}

	*dimensions = fileDimensions;
	*cells = std::move( output );
	return true;
}
