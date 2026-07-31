// SPDX-FileCopyrightText: 2026
// SPDX-License-Identifier: MIT

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

bool InflateVoxelDeflate( const std::vector<uint8_t>& compressed, size_t maximumOutputSize,
						  std::vector<uint8_t>* output, std::string* error );

// Emits valid raw RFC1951 stored blocks. Compression ratio is intentionally
// secondary because this is used for Unity interchange, while B3VX uses RLE.
std::vector<uint8_t> DeflateVoxelStoredBlocks( const std::vector<uint8_t>& input );
