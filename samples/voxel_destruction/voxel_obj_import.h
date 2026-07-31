// SPDX-FileCopyrightText: 2026
// SPDX-License-Identifier: MIT

#pragma once

#include "voxel_destruction/voxel_voxelizer.h"

#include <string>

bool LoadObjVoxelTriangleMesh( const char* path, float scale, bool zUp, VoxelTriangleMesh* mesh, std::string* error );
