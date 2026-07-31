// SPDX-FileCopyrightText: 2026
// SPDX-License-Identifier: MIT

#pragma once

#include "voxel_destruction/voxel_types.h"

#include <vector>

std::vector<std::vector<int>> LabelVoxelComponents( const std::vector<VoxelCell>& cells, Int3 dimensions );
int FindMostGroundedComponent( const std::vector<std::vector<int>>& components, Int3 dimensions );
