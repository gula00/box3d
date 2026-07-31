// SPDX-FileCopyrightText: 2026
// SPDX-License-Identifier: MIT

#include "voxel_destruction/voxel_self_test.h"

#include "voxel_destruction/voxel_collider.h"
#include "voxel_destruction/voxel_chunk_mesh.h"
#include "voxel_destruction/voxel_connectivity.h"
#include "voxel_destruction/voxel_deflate.h"
#include "voxel_destruction/voxel_editing.h"
#include "voxel_destruction/voxel_fracture.h"
#include "voxel_destruction/voxel_obj_import.h"
#include "voxel_destruction/voxel_serialization.h"
#include "voxel_destruction/voxel_voxelizer.h"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

namespace
{

bool Fail( std::string* failure, const char* message )
{
	if ( failure != nullptr )
	{
		*failure = message;
	}
	return false;
}

int CountFilled( const std::vector<VoxelCell>& cells )
{
	int count = 0;
	for ( const VoxelCell& cell : cells )
	{
		count += cell.filled != 0;
	}
	return count;
}

bool EqualCell( const VoxelCell& a, const VoxelCell& b )
{
	return a.filled == b.filled && a.material == b.material && a.r == b.r && a.g == b.g && a.b == b.b && a.a == b.a;
}

std::vector<uint8_t> DecodeHex( const char* hex )
{
	std::vector<uint8_t> result;
	for ( size_t index = 0; hex[index] != '\0' && hex[index + 1] != '\0'; index += 2 )
	{
		auto nibble = []( char value )
		{
			return value <= '9' ? value - '0' : value - 'A' + 10;
		};
		result.push_back( static_cast<uint8_t>( ( nibble( hex[index] ) << 4 ) | nibble( hex[index + 1] ) ) );
	}
	return result;
}

std::string EncodeHex( const std::vector<uint8_t>& bytes )
{
	constexpr char digits[] = "0123456789ABCDEF";
	std::string result;
	result.reserve( 2 * bytes.size() );
	for ( uint8_t byte : bytes )
	{
		result.push_back( digits[byte >> 4] );
		result.push_back( digits[byte & 0x0F] );
	}
	return result;
}

} // namespace

bool RunVoxelSelfTests( std::string* failure )
{
	Int3 dimensions = { 4, 4, 4 };
	VoxelCell filled = { 1, 2, 20, 40, 60, 255 };
	std::vector<VoxelCell> solid( 64, filled );

	std::vector<VoxelBoxRun> solidRuns = BuildVoxelBoxRuns( solid, dimensions );
	if ( solidRuns.size() != 1 || solidRuns[0].lower.x != 0 || solidRuns[0].upper.x != 3 )
	{
		return Fail( failure, "greedy collider test failed" );
	}
	VoxelChunkMeshBuildBatch solidMeshes = BuildVoxelChunkMeshes( dimensions, solid );
	if ( solidMeshes.size() != 1 || solidMeshes[0].coloredMeshes.size() != 1 ||
		 solidMeshes[0].coloredMeshes[0].mesh.indices.size() != 36 )
	{
		return Fail( failure, "greedy render mesh test failed" );
	}
	const VoxelCpuMesh& solidMesh = solidMeshes[0].coloredMeshes[0].mesh;
	for ( size_t triangle = 0; triangle < solidMesh.indices.size(); triangle += 3 )
	{
		const MeshVertex& v0 = solidMesh.vertices[solidMesh.indices[triangle + 0]];
		const MeshVertex& v1 = solidMesh.vertices[solidMesh.indices[triangle + 1]];
		const MeshVertex& v2 = solidMesh.vertices[solidMesh.indices[triangle + 2]];
		b3Vec3 edge1 = { v1.position[0] - v0.position[0], v1.position[1] - v0.position[1],
						v1.position[2] - v0.position[2] };
		b3Vec3 edge2 = { v2.position[0] - v0.position[0], v2.position[1] - v0.position[1],
						v2.position[2] - v0.position[2] };
		b3Vec3 normal = { v0.normal[0], v0.normal[1], v0.normal[2] };
		if ( b3Dot( b3Cross( edge1, edge2 ), normal ) <= 0.0f )
		{
			return Fail( failure, "greedy render mesh winding test failed" );
		}
	}
	VoxelChunkMeshBuildBatch noMeshes = BuildVoxelChunkMeshes( dimensions, solid, {} );
	if ( noMeshes.empty() == false )
	{
		return Fail( failure, "incremental render mesh test failed" );
	}
	Int3 twoChunkDimensions = { kVoxelChunkSize + 1, 1, 1 };
	std::vector<VoxelCell> twoChunkSolid( static_cast<size_t>( twoChunkDimensions.x ), filled );
	VoxelChunkMeshBuildBatch secondChunkMesh =
		BuildVoxelChunkMeshes( twoChunkDimensions, twoChunkSolid, { 1 } );
	VoxelChunkColliderBuildBatch secondChunkCollider =
		BuildVoxelChunkBoxRuns( twoChunkSolid, twoChunkDimensions, { 1 } );
	if ( secondChunkMesh.size() != 1 || secondChunkMesh[0].chunkIndex != 1 || secondChunkCollider.size() != 1 ||
		 secondChunkCollider[0].chunkIndex != 1 || secondChunkCollider[0].runs.size() != 1 ||
		 secondChunkCollider[0].runs[0].lower.x != kVoxelChunkSize )
	{
		return Fail( failure, "dirty chunk build test failed" );
	}

	std::vector<VoxelCell> resized = ResizeVoxelMapNearest( solid, dimensions, { 8, 8, 8 } );
	if ( resized.size() != 512 || CountFilled( resized ) != 512 || EqualCell( resized.back(), filled ) == false )
	{
		return Fail( failure, "nearest resize test failed" );
	}

	std::vector<VoxelCell> hollow = HollowVoxelMap( solid, dimensions, 1 );
	if ( CountFilled( hollow ) != 56 || hollow[GetIndex( { 1, 1, 1 }, dimensions )].filled != 0 )
	{
		return Fail( failure, "hollow test failed" );
	}

	std::vector<VoxelCell> componentsMap( solid.size() );
	componentsMap[GetIndex( { 0, 0, 0 }, dimensions )] = filled;
	componentsMap[GetIndex( { 0, 0, 1 }, dimensions )] = filled;
	componentsMap[GetIndex( { 3, 3, 3 }, dimensions )] = filled;
	std::vector<std::vector<int>> components = LabelVoxelComponents( componentsMap, dimensions );
	if ( components.size() != 2 || components[0].size() != 2 )
	{
		return Fail( failure, "connected component test failed" );
	}
	if ( FindMostGroundedComponent( components, dimensions ) != 0 )
	{
		return Fail( failure, "grounded component test failed" );
	}

	std::vector<int> sphere = CutVoxelSphere( solid, dimensions, { 0.0f, 0.0f, 0.0f }, 0.31f );
	if ( sphere.empty() )
	{
		return Fail( failure, "sphere cutter test failed" );
	}
	std::vector<std::vector<int>> voronoi =
		GenerateVoxelVoronoiGroups( sphere, dimensions, { { -0.2f, 0.0f, 0.0f }, { 0.2f, 0.0f, 0.0f } } );
	if ( voronoi.size() != 2 || voronoi[0].size() + voronoi[1].size() != sphere.size() )
	{
		return Fail( failure, "Voronoi label test failed" );
	}

	std::vector<int> allIndices;
	allIndices.reserve( solid.size() );
	for ( int index = 0; index < static_cast<int>( solid.size() ); ++index )
	{
		allIndices.push_back( index );
	}
	if ( DestroyVoxelOuterLayer( allIndices, dimensions ).size() != 8 )
	{
		return Fail( failure, "outer layer test failed" );
	}
	std::vector<VoxelCell> sphereSource( dimensions.x * dimensions.y * dimensions.z );
	for ( VoxelCell& cell : sphereSource )
	{
		cell.filled = 1;
		cell.a = 255;
	}
	VoxelSphereRegion sphereCopy = CopyVoxelSphere( sphereSource, dimensions, { 1, 1, 1 }, 1 );
	std::vector<VoxelCell> sphereCutSource = sphereSource;
	VoxelSphereRegion sphereCut = CutAndCopyVoxelSphere( &sphereCutSource, dimensions, { 1, 1, 1 }, 1 );
	if ( sphereCopy.dimensions.x != 3 || CountFilled( sphereCopy.cells ) != 7 || CountFilled( sphereCut.cells ) != 7 ||
		 CountFilled( sphereCutSource ) != 57 ||
		 GenerateVoxelSeedsInBox( sphereSource, dimensions, { 0, 0, 0 }, { 3, 3, 3 }, 1.0f, 7 ).size() != 64 )
	{
		return Fail( failure, "sphere copy/cut and box seed generation test failed" );
	}

	VoxelTriangleMesh cube;
	cube.vertices = {
		{ -0.5f, -0.5f, -0.5f }, { 0.5f, -0.5f, -0.5f }, { 0.5f, 0.5f, -0.5f },
		{ -0.5f, 0.5f, -0.5f },  { -0.5f, -0.5f, 0.5f }, { 0.5f, -0.5f, 0.5f },
		{ 0.5f, 0.5f, 0.5f },	  { -0.5f, 0.5f, 0.5f },
	};
	cube.indices = {
		0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4,
		1, 2, 6, 1, 6, 5, 2, 3, 7, 2, 7, 6, 3, 0, 4, 3, 4, 7,
	};
	cube.textures.push_back( { 1, 1, { 17, 34, 51, 68 } } );
	cube.triangleSurfaces.resize( cube.indices.size() / 3 );
	for ( VoxelTriangleSurface& surface : cube.triangleSurfaces )
	{
		surface.textureIndex = 0;
	}
	VoxelizedTriangleMesh voxelizedCube = VoxelizeTriangleMesh( cube, 0.25f );
	if ( voxelizedCube.dimensions.x != 4 || voxelizedCube.dimensions.y != 4 || voxelizedCube.dimensions.z != 4 ||
		 CountFilled( voxelizedCube.cells ) != 64 || voxelizedCube.cells.front().r != 17 ||
		 voxelizedCube.cells.front().g != 34 || voxelizedCube.cells.front().b != 51 ||
		 voxelizedCube.cells.front().a != 68 )
	{
		return Fail( failure, "textured triangle mesh voxelizer test failed" );
	}

	auto timestamp = std::chrono::steady_clock::now().time_since_epoch().count();
	std::filesystem::path objPath =
		std::filesystem::temp_directory_path() / ( "box3d_voxel_material_" + std::to_string( timestamp ) + ".obj" );
	std::filesystem::path mtlPath = objPath;
	mtlPath.replace_extension( ".mtl" );
	std::filesystem::path bmpPath = objPath;
	bmpPath.replace_extension( ".bmp" );
	{
		std::vector<uint8_t> bmp( 58, 0 );
		bmp[0] = 'B';
		bmp[1] = 'M';
		bmp[2] = 58;
		bmp[10] = 54;
		bmp[14] = 40;
		bmp[18] = 1;
		bmp[22] = 1;
		bmp[26] = 1;
		bmp[28] = 24;
		bmp[34] = 4;
		bmp[54] = 33;
		bmp[55] = 22;
		bmp[56] = 11;
		std::ofstream textureFile( bmpPath, std::ios::binary | std::ios::trunc );
		textureFile.write( reinterpret_cast<const char*>( bmp.data() ), static_cast<std::streamsize>( bmp.size() ) );
	}
	{
		std::ofstream materialFile( mtlPath, std::ios::binary | std::ios::trunc );
		materialFile << "newmtl test_material\nKd 0.2 0.4 0.6\nd 1\nmap_Kd " << bmpPath.filename().string() << "\n";
		std::ofstream objFile( objPath, std::ios::binary | std::ios::trunc );
		objFile << "mtllib " << mtlPath.filename().string()
				<< "\nv 0 0 0\nv 1 0 0\nv 0 1 0\nvt 0 0\nvt 1 0\nvt 0 1\nusemtl test_material\nf 1/1 2/2 3/3\n";
	}
	VoxelTriangleMesh importedMesh;
	std::string objError;
	bool objLoaded = LoadObjVoxelTriangleMesh( objPath.string().c_str(), 1.0f, false, &importedMesh, &objError );
	std::error_code removeError;
	std::filesystem::remove( objPath, removeError );
	std::filesystem::remove( mtlPath, removeError );
	std::filesystem::remove( bmpPath, removeError );
	if ( objLoaded == false || importedMesh.triangleSurfaces.size() != 1 || importedMesh.textures.size() != 1 ||
		 importedMesh.triangleSurfaces[0].textureIndex != 0 || importedMesh.textures[0].rgba.size() != 4 ||
		 importedMesh.textures[0].rgba[0] != 11 || importedMesh.textures[0].rgba[1] != 22 ||
		 importedMesh.textures[0].rgba[2] != 33 )
	{
		return Fail( failure, objError.empty() ? "OBJ material texture import test failed" : objError.c_str() );
	}

	std::filesystem::path path =
		std::filesystem::temp_directory_path() / ( "box3d_voxel_roundtrip_" + std::to_string( timestamp ) + ".b3vox" );
	std::string serializationError;
	if ( SaveVoxelMap( path.string().c_str(), dimensions, hollow, &serializationError ) == false )
	{
		return Fail( failure, serializationError.c_str() );
	}

	Int3 loadedDimensions;
	std::vector<VoxelCell> loaded;
	bool loadedSuccessfully = LoadVoxelMap( path.string().c_str(), &loadedDimensions, &loaded, &serializationError );
	std::filesystem::remove( path, removeError );
	if ( loadedSuccessfully == false || loadedDimensions.x != dimensions.x || loadedDimensions.y != dimensions.y ||
		 loadedDimensions.z != dimensions.z || loaded.size() != hollow.size() )
	{
		return Fail( failure, "serialization round-trip test failed" );
	}
	for ( size_t index = 0; index < loaded.size(); ++index )
	{
		if ( EqualCell( loaded[index], hollow[index] ) == false )
		{
			return Fail( failure, "serialization cell mismatch" );
		}
	}

	std::vector<uint8_t> unityPayload = EncodeUnityVoxelPayload( hollow );
	std::vector<VoxelCell> unityDecoded;
	if ( DecodeUnityVoxelPayload( unityPayload, dimensions, &unityDecoded, &serializationError ) == false ||
		 unityDecoded.size() != hollow.size() )
	{
		return Fail( failure, "Unity payload round-trip test failed" );
	}
	for ( size_t index = 0; index < unityDecoded.size(); ++index )
	{
		const VoxelCell& expected = hollow[index];
		const VoxelCell& actual = unityDecoded[index];
		if ( expected.filled != actual.filled || expected.r != actual.r || expected.g != actual.g ||
			 expected.b != actual.b || expected.a != actual.a )
		{
			return Fail( failure, "Unity payload cell mismatch" );
		}
	}

	std::filesystem::path unityExportPath =
		std::filesystem::temp_directory_path() / ( "box3d_unity_export_" + std::to_string( timestamp ) + ".asset" );
	if ( SaveUnitySerializedVoxelMapAsset(
			 unityExportPath.string().c_str(), dimensions, hollow, &serializationError ) == false )
	{
		return Fail( failure, serializationError.c_str() );
	}
	Int3 unityExportDimensions;
	std::vector<VoxelCell> unityExportCells;
	bool unityExportLoaded = LoadUnitySerializedVoxelMapAsset(
		unityExportPath.string().c_str(), &unityExportDimensions, &unityExportCells, &serializationError );
	std::ifstream unityExportFile( unityExportPath, std::ios::binary );
	std::string unityExportText{ std::istreambuf_iterator<char>( unityExportFile ), std::istreambuf_iterator<char>() };
	std::filesystem::remove( unityExportPath, removeError );
	if ( unityExportLoaded == false || unityExportDimensions.x != dimensions.x || unityExportCells.size() != hollow.size() ||
		 unityExportText.find( "guid: 347419e1422c35a4194f363558eccfd5" ) == std::string::npos )
	{
		return Fail( failure, "Unity SerializedVoxelMap export test failed" );
	}

	std::filesystem::path unityAssetPath =
		std::filesystem::temp_directory_path() / ( "box3d_unity_voxel_" + std::to_string( timestamp ) + ".asset" );
	{
		std::ofstream asset( unityAssetPath, std::ios::binary | std::ios::trunc );
		asset << "voxelMapDimensions: {x: 4, y: 4, z: 4}\n";
		asset << "compressedVoxelDataValues: " << EncodeHex( unityPayload ) << "\n";
	}
	Int3 unityAssetDimensions;
	std::vector<VoxelCell> unityAssetCells;
	bool unityAssetLoaded =
		LoadUnitySerializedVoxelMapAsset( unityAssetPath.string().c_str(), &unityAssetDimensions, &unityAssetCells,
										 &serializationError );
	std::filesystem::remove( unityAssetPath, removeError );
	if ( unityAssetLoaded == false || unityAssetDimensions.x != 4 || unityAssetCells.size() != hollow.size() )
	{
		return Fail( failure, "Unity SerializedVoxelMap asset test failed" );
	}

	const char* dynamicDeflateHex =
		"EDCC4B1681500000D02D8984E5BC24BF3E0A91D5EB18744C9A36BA7701378409E9BFFD28FB390CF2FC783A5FAE4559D5B7"
		"A6BD3F9EDDEBDD7F16D17215AF93CD7637159BCD66B3D96C369BCD66B3D96C369BCD66B3D96C369BCD66B3D96C369BCDF3"
		"CF5F";
	std::string deflateUnit =
		"aaaaaaaaaaaaaaaaaaaaaaaaabbbbbbbbbbbbbccccccccccddddddeeeeffghijklmnopqrstuvwxyz0123456789";
	std::string expectedDeflateText;
	for ( int repeat = 0; repeat < 100; ++repeat )
	{
		expectedDeflateText += deflateUnit;
	}
	std::vector<uint8_t> dynamicDecoded;
	if ( InflateVoxelDeflate( DecodeHex( dynamicDeflateHex ), expectedDeflateText.size(), &dynamicDecoded,
							  &serializationError ) == false ||
		 dynamicDecoded.size() != expectedDeflateText.size() ||
		 std::equal( dynamicDecoded.begin(), dynamicDecoded.end(), expectedDeflateText.begin() ) == false )
	{
		return Fail( failure, "dynamic-Huffman Deflate test failed" );
	}

	return true;
}
