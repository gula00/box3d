// SPDX-FileCopyrightText: 2026
// SPDX-License-Identifier: MIT

#pragma once

#include "voxel_destruction/voxel_types.h"

#include <string>
#include <vector>

bool SaveVoxelMap( const char* path, Int3 dimensions, const std::vector<VoxelCell>& cells, std::string* error );
bool LoadVoxelMap( const char* path, Int3* dimensions, std::vector<VoxelCell>* cells, std::string* error );

std::vector<uint8_t> EncodeUnityVoxelPayload( const std::vector<VoxelCell>& cells );
bool DecodeUnityVoxelPayload( const std::vector<uint8_t>& compressed, Int3 dimensions, std::vector<VoxelCell>* cells,
							  std::string* error );
bool LoadUnitySerializedVoxelMapAsset( const char* path, Int3* dimensions, std::vector<VoxelCell>* cells,
									  std::string* error );
bool SaveUnitySerializedVoxelMapAsset( const char* path, Int3 dimensions, const std::vector<VoxelCell>& cells,
									  std::string* error );
