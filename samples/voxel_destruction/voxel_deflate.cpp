// SPDX-FileCopyrightText: 2026
// SPDX-License-Identifier: MIT

#include "voxel_destruction/voxel_deflate.h"

#include <algorithm>
#include <array>

namespace
{

class BitReader
{
public:
	BitReader( const uint8_t* data, size_t size ) : m_data( data ), m_size( size ) {}

	bool ReadBits( int count, uint32_t* value )
	{
		if ( count < 0 || count > 24 )
		{
			return false;
		}
		while ( m_bitCount < count )
		{
			if ( m_byteIndex >= m_size )
			{
				return false;
			}
			m_bits |= static_cast<uint64_t>( m_data[m_byteIndex++] ) << m_bitCount;
			m_bitCount += 8;
		}
		uint64_t mask = count == 0 ? 0 : ( uint64_t{ 1 } << count ) - 1;
		*value = static_cast<uint32_t>( m_bits & mask );
		m_bits >>= count;
		m_bitCount -= count;
		return true;
	}

	void AlignToByte()
	{
		m_bits = 0;
		m_bitCount = 0;
	}

	bool ReadByte( uint8_t* value )
	{
		uint32_t bits = 0;
		if ( ReadBits( 8, &bits ) == false )
		{
			return false;
		}
		*value = static_cast<uint8_t>( bits );
		return true;
	}

private:
	const uint8_t* m_data;
	size_t m_size;
	size_t m_byteIndex = 0;
	uint64_t m_bits = 0;
	int m_bitCount = 0;
};

uint16_t ReverseBits( uint16_t value, int count )
{
	uint16_t result = 0;
	for ( int bit = 0; bit < count; ++bit )
	{
		result = static_cast<uint16_t>( ( result << 1 ) | ( value & 1u ) );
		value >>= 1;
	}
	return result;
}

class HuffmanTable
{
public:
	bool Build( const std::vector<uint8_t>& lengths )
	{
		m_entries.clear();
		std::array<int, 16> counts = {};
		for ( uint8_t length : lengths )
		{
			if ( length > 15 )
			{
				return false;
			}
			if ( length != 0 )
			{
				counts[length] += 1;
			}
		}

		std::array<int, 16> nextCode = {};
		int code = 0;
		for ( int bits = 1; bits <= 15; ++bits )
		{
			code = ( code + counts[bits - 1] ) << 1;
			nextCode[bits] = code;
		}

		for ( int symbol = 0; symbol < static_cast<int>( lengths.size() ); ++symbol )
		{
			int length = lengths[symbol];
			if ( length == 0 )
			{
				continue;
			}
			uint16_t reversed = ReverseBits( static_cast<uint16_t>( nextCode[length]++ ), length );
			m_entries.push_back( { reversed, static_cast<uint16_t>( symbol ), static_cast<uint8_t>( length ) } );
		}
		return m_entries.empty() == false;
	}

	bool Decode( BitReader& reader, int* symbol ) const
	{
		uint16_t code = 0;
		for ( int length = 1; length <= 15; ++length )
		{
			uint32_t bit = 0;
			if ( reader.ReadBits( 1, &bit ) == false )
			{
				return false;
			}
			code = static_cast<uint16_t>( code | ( bit << ( length - 1 ) ) );
			for ( const Entry& entry : m_entries )
			{
				if ( entry.length == length && entry.code == code )
				{
					*symbol = entry.symbol;
					return true;
				}
			}
		}
		return false;
	}

private:
	struct Entry
	{
		uint16_t code;
		uint16_t symbol;
		uint8_t length;
	};
	std::vector<Entry> m_entries;
};

bool BuildFixedTables( HuffmanTable* literalTable, HuffmanTable* distanceTable )
{
	std::vector<uint8_t> literalLengths( 288 );
	for ( int symbol = 0; symbol <= 143; ++symbol )
	{
		literalLengths[symbol] = 8;
	}
	for ( int symbol = 144; symbol <= 255; ++symbol )
	{
		literalLengths[symbol] = 9;
	}
	for ( int symbol = 256; symbol <= 279; ++symbol )
	{
		literalLengths[symbol] = 7;
	}
	for ( int symbol = 280; symbol <= 287; ++symbol )
	{
		literalLengths[symbol] = 8;
	}
	return literalTable->Build( literalLengths ) && distanceTable->Build( std::vector<uint8_t>( 32, 5 ) );
}

bool BuildDynamicTables( BitReader& reader, HuffmanTable* literalTable, HuffmanTable* distanceTable )
{
	uint32_t hlitBits = 0;
	uint32_t hdistBits = 0;
	uint32_t hclenBits = 0;
	if ( reader.ReadBits( 5, &hlitBits ) == false || reader.ReadBits( 5, &hdistBits ) == false ||
		 reader.ReadBits( 4, &hclenBits ) == false )
	{
		return false;
	}
	int literalCount = static_cast<int>( hlitBits ) + 257;
	int distanceCount = static_cast<int>( hdistBits ) + 1;
	int codeLengthCount = static_cast<int>( hclenBits ) + 4;

	constexpr int order[19] = { 16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15 };
	std::vector<uint8_t> codeLengths( 19, 0 );
	for ( int index = 0; index < codeLengthCount; ++index )
	{
		uint32_t length = 0;
		if ( reader.ReadBits( 3, &length ) == false )
		{
			return false;
		}
		codeLengths[order[index]] = static_cast<uint8_t>( length );
	}

	HuffmanTable codeLengthTable;
	if ( codeLengthTable.Build( codeLengths ) == false )
	{
		return false;
	}

	std::vector<uint8_t> lengths;
	lengths.reserve( literalCount + distanceCount );
	while ( static_cast<int>( lengths.size() ) < literalCount + distanceCount )
	{
		int symbol = 0;
		if ( codeLengthTable.Decode( reader, &symbol ) == false )
		{
			return false;
		}
		if ( symbol <= 15 )
		{
			lengths.push_back( static_cast<uint8_t>( symbol ) );
		}
		else if ( symbol == 16 )
		{
			if ( lengths.empty() )
			{
				return false;
			}
			uint32_t extra = 0;
			if ( reader.ReadBits( 2, &extra ) == false )
			{
				return false;
			}
			int repeat = static_cast<int>( extra ) + 3;
			uint8_t value = lengths.back();
			if ( static_cast<int>( lengths.size() ) + repeat > literalCount + distanceCount )
			{
				return false;
			}
			lengths.insert( lengths.end(), repeat, value );
		}
		else if ( symbol == 17 || symbol == 18 )
		{
			int extraBitCount = symbol == 17 ? 3 : 7;
			int baseRepeat = symbol == 17 ? 3 : 11;
			uint32_t extra = 0;
			if ( reader.ReadBits( extraBitCount, &extra ) == false )
			{
				return false;
			}
			int repeat = baseRepeat + static_cast<int>( extra );
			if ( static_cast<int>( lengths.size() ) + repeat > literalCount + distanceCount )
			{
				return false;
			}
			lengths.insert( lengths.end(), repeat, 0 );
		}
		else
		{
			return false;
		}
	}

	std::vector<uint8_t> literalLengths( lengths.begin(), lengths.begin() + literalCount );
	std::vector<uint8_t> distanceLengths( lengths.begin() + literalCount, lengths.end() );
	return literalLengths.size() > 256 && literalLengths[256] != 0 && literalTable->Build( literalLengths ) &&
		   distanceTable->Build( distanceLengths );
}

bool InflateCompressedBlock( BitReader& reader, const HuffmanTable& literalTable, const HuffmanTable& distanceTable,
							 size_t maximumOutputSize, std::vector<uint8_t>* output )
{
	constexpr int lengthBases[29] = {
		3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27,
		31, 35, 43, 51, 59, 67, 83, 99, 115, 131, 163, 195, 227, 258,
	};
	constexpr int lengthExtraBits[29] = {
		0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2,
		2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0,
	};
	constexpr int distanceBases[30] = {
		1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129,
		193, 257, 385, 513, 769, 1025, 1537, 2049, 3073, 4097, 6145,
		8193, 12289, 16385, 24577,
	};
	constexpr int distanceExtraBits[30] = {
		0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6,
		6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13,
	};

	for ( ;; )
	{
		int symbol = 0;
		if ( literalTable.Decode( reader, &symbol ) == false )
		{
			return false;
		}
		if ( symbol < 256 )
		{
			if ( output->size() >= maximumOutputSize )
			{
				return false;
			}
			output->push_back( static_cast<uint8_t>( symbol ) );
			continue;
		}
		if ( symbol == 256 )
		{
			return true;
		}
		if ( symbol < 257 || symbol > 285 )
		{
			return false;
		}

		int lengthIndex = symbol - 257;
		uint32_t lengthExtra = 0;
		if ( reader.ReadBits( lengthExtraBits[lengthIndex], &lengthExtra ) == false )
		{
			return false;
		}
		int length = lengthBases[lengthIndex] + static_cast<int>( lengthExtra );

		int distanceSymbol = 0;
		if ( distanceTable.Decode( reader, &distanceSymbol ) == false || distanceSymbol < 0 || distanceSymbol >= 30 )
		{
			return false;
		}
		uint32_t distanceExtra = 0;
		if ( reader.ReadBits( distanceExtraBits[distanceSymbol], &distanceExtra ) == false )
		{
			return false;
		}
		int distance = distanceBases[distanceSymbol] + static_cast<int>( distanceExtra );
		if ( distance <= 0 || static_cast<size_t>( distance ) > output->size() ||
			 output->size() + static_cast<size_t>( length ) > maximumOutputSize )
		{
			return false;
		}
		for ( int index = 0; index < length; ++index )
		{
			output->push_back( ( *output )[output->size() - distance] );
		}
	}
}

void SetError( std::string* error, const char* message )
{
	if ( error != nullptr )
	{
		*error = message;
	}
}

} // namespace

bool InflateVoxelDeflate( const std::vector<uint8_t>& compressed, size_t maximumOutputSize,
						  std::vector<uint8_t>* output, std::string* error )
{
	if ( output == nullptr || compressed.empty() )
	{
		SetError( error, "invalid Deflate input" );
		return false;
	}

	size_t start = 0;
	size_t size = compressed.size();
	if ( size >= 6 && ( compressed[0] & 0x0Fu ) == 8 &&
		 ( ( static_cast<int>( compressed[0] ) << 8 ) + compressed[1] ) % 31 == 0 )
	{
		start = 2;
		size -= 6;
	}

	BitReader reader( compressed.data() + start, size );
	output->clear();
	bool finalBlock = false;
	while ( finalBlock == false )
	{
		uint32_t finalBit = 0;
		uint32_t blockType = 0;
		if ( reader.ReadBits( 1, &finalBit ) == false || reader.ReadBits( 2, &blockType ) == false )
		{
			SetError( error, "truncated Deflate block header" );
			return false;
		}
		finalBlock = finalBit != 0;

		if ( blockType == 0 )
		{
			reader.AlignToByte();
			uint8_t lengthBytes[4];
			for ( uint8_t& byte : lengthBytes )
			{
				if ( reader.ReadByte( &byte ) == false )
				{
					SetError( error, "truncated stored Deflate block" );
					return false;
				}
			}
			uint16_t length = static_cast<uint16_t>( lengthBytes[0] | ( lengthBytes[1] << 8 ) );
			uint16_t inverseLength = static_cast<uint16_t>( lengthBytes[2] | ( lengthBytes[3] << 8 ) );
			if ( static_cast<uint16_t>( ~length ) != inverseLength || output->size() + length > maximumOutputSize )
			{
				SetError( error, "invalid stored Deflate block" );
				return false;
			}
			for ( int index = 0; index < length; ++index )
			{
				uint8_t byte = 0;
				if ( reader.ReadByte( &byte ) == false )
				{
					SetError( error, "truncated stored Deflate payload" );
					return false;
				}
				output->push_back( byte );
			}
		}
		else if ( blockType == 1 || blockType == 2 )
		{
			HuffmanTable literalTable;
			HuffmanTable distanceTable;
			bool built = blockType == 1 ? BuildFixedTables( &literalTable, &distanceTable ) :
										 BuildDynamicTables( reader, &literalTable, &distanceTable );
			if ( built == false ||
				 InflateCompressedBlock( reader, literalTable, distanceTable, maximumOutputSize, output ) == false )
			{
				SetError( error, "invalid Huffman Deflate block" );
				return false;
			}
		}
		else
		{
			SetError( error, "reserved Deflate block type" );
			return false;
		}
	}
	return true;
}

std::vector<uint8_t> DeflateVoxelStoredBlocks( const std::vector<uint8_t>& input )
{
	std::vector<uint8_t> output;
	size_t offset = 0;
	do
	{
		size_t remaining = input.size() - offset;
		uint16_t blockSize = static_cast<uint16_t>( std::min<size_t>( remaining, 65535 ) );
		bool finalBlock = offset + blockSize == input.size();
		output.push_back( finalBlock ? 1 : 0 );
		output.push_back( static_cast<uint8_t>( blockSize ) );
		output.push_back( static_cast<uint8_t>( blockSize >> 8 ) );
		uint16_t inverse = static_cast<uint16_t>( ~blockSize );
		output.push_back( static_cast<uint8_t>( inverse ) );
		output.push_back( static_cast<uint8_t>( inverse >> 8 ) );
		output.insert( output.end(), input.begin() + offset, input.begin() + offset + blockSize );
		offset += blockSize;
	} while ( offset < input.size() );
	return output;
}
