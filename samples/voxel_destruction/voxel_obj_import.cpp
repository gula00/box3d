// SPDX-FileCopyrightText: 2026
// SPDX-License-Identifier: MIT

#include "voxel_destruction/voxel_obj_import.h"

#include "tiny_obj_loader.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <utility>

#if defined( _WIN32 )
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <wincodec.h>
#pragma comment( lib, "ole32.lib" )
#pragma comment( lib, "windowscodecs.lib" )
#endif

namespace
{

uint8_t ToByte( float value )
{
	return static_cast<uint8_t>( std::lround( 255.0f * std::clamp( value, 0.0f, 1.0f ) ) );
}

uint32_t PackColor( float r, float g, float b, float a )
{
	return static_cast<uint32_t>( ToByte( r ) ) << 24 | static_cast<uint32_t>( ToByte( g ) ) << 16 |
		   static_cast<uint32_t>( ToByte( b ) ) << 8 | static_cast<uint32_t>( ToByte( a ) );
}

#if defined( _WIN32 )
bool LoadTextureImage( const std::filesystem::path& path, VoxelTextureImage* image )
{
	HRESULT initializeResult = CoInitializeEx( nullptr, COINIT_MULTITHREADED );
	bool uninitialize = initializeResult == S_OK || initializeResult == S_FALSE;
	if ( FAILED( initializeResult ) && initializeResult != RPC_E_CHANGED_MODE )
	{
		return false;
	}

	IWICImagingFactory* factory = nullptr;
	IWICBitmapDecoder* decoder = nullptr;
	IWICBitmapFrameDecode* frame = nullptr;
	IWICFormatConverter* converter = nullptr;
	bool success = false;

	HRESULT result = CoCreateInstance(
		CLSID_WICImagingFactory, nullptr, CLSCTX_INPROC_SERVER, IID_PPV_ARGS( &factory ) );
	if ( SUCCEEDED( result ) )
	{
		result = factory->CreateDecoderFromFilename(
			path.c_str(), nullptr, GENERIC_READ, WICDecodeMetadataCacheOnLoad, &decoder );
	}
	if ( SUCCEEDED( result ) )
	{
		result = decoder->GetFrame( 0, &frame );
	}
	if ( SUCCEEDED( result ) )
	{
		result = factory->CreateFormatConverter( &converter );
	}
	if ( SUCCEEDED( result ) )
	{
		result = converter->Initialize(
			frame, GUID_WICPixelFormat32bppRGBA, WICBitmapDitherTypeNone, nullptr, 0.0, WICBitmapPaletteTypeCustom );
	}

	UINT width = 0;
	UINT height = 0;
	if ( SUCCEEDED( result ) )
	{
		result = converter->GetSize( &width, &height );
	}
	if ( SUCCEEDED( result ) && width > 0 && height > 0 &&
		 static_cast<uint64_t>( width ) * height <= 16384ull * 16384ull )
	{
		image->width = static_cast<int>( width );
		image->height = static_cast<int>( height );
		image->rgba.resize( static_cast<size_t>( width ) * height * 4 );
		result = converter->CopyPixels(
			nullptr, width * 4, static_cast<UINT>( image->rgba.size() ), image->rgba.data() );
		success = SUCCEEDED( result );
	}

	if ( converter != nullptr )
	{
		converter->Release();
	}
	if ( frame != nullptr )
	{
		frame->Release();
	}
	if ( decoder != nullptr )
	{
		decoder->Release();
	}
	if ( factory != nullptr )
	{
		factory->Release();
	}
	if ( uninitialize )
	{
		CoUninitialize();
	}
	if ( success == false )
	{
		*image = {};
	}
	return success;
}
#else
bool LoadTextureImage( const std::filesystem::path&, VoxelTextureImage* )
{
	return false;
}
#endif

b3Vec3 TransformVertex( const tinyobj::attrib_t& attributes, int vertexIndex, float scale, bool zUp )
{
	float x = scale * attributes.vertices[3 * vertexIndex + 0];
	float y = scale * attributes.vertices[3 * vertexIndex + 1];
	float z = scale * attributes.vertices[3 * vertexIndex + 2];
	return zUp ? b3Vec3{ y, z, x } : b3Vec3{ x, y, z };
}

} // namespace

bool LoadObjVoxelTriangleMesh( const char* path, float scale, bool zUp, VoxelTriangleMesh* mesh, std::string* error )
{
	if ( path == nullptr || path[0] == '\0' || mesh == nullptr || scale <= 0.0f )
	{
		if ( error != nullptr )
		{
			*error = "invalid OBJ import arguments";
		}
		return false;
	}

	std::filesystem::path objPath = std::filesystem::path( path );
	std::filesystem::path parentPath = objPath.parent_path();
	tinyobj::ObjReaderConfig config;
	config.triangulate = true;
	config.mtl_search_path = parentPath.empty() ? "./" : parentPath.generic_string() + "/";

	tinyobj::ObjReader reader;
	if ( reader.ParseFromFile( objPath.string(), config ) == false )
	{
		if ( error != nullptr )
		{
			*error = reader.Error().empty() ? "could not load OBJ" : reader.Error();
		}
		return false;
	}

	const tinyobj::attrib_t& attributes = reader.GetAttrib();
	const std::vector<tinyobj::shape_t>& shapes = reader.GetShapes();
	const std::vector<tinyobj::material_t>& materials = reader.GetMaterials();
	mesh->vertices.clear();
	mesh->indices.clear();
	mesh->triangleColors.clear();
	mesh->triangleSurfaces.clear();
	mesh->textures.clear();

	std::unordered_map<std::string, int> loadedTextures;
	auto getTextureIndex =
		[&]( const std::string& textureName )
		{
			if ( textureName.empty() )
			{
				return -1;
			}
			std::filesystem::path texturePath = parentPath / std::filesystem::path( textureName );
			std::string key = texturePath.lexically_normal().generic_string();
			auto existing = loadedTextures.find( key );
			if ( existing != loadedTextures.end() )
			{
				return existing->second;
			}

			VoxelTextureImage image;
			int textureIndex = -1;
			if ( LoadTextureImage( texturePath, &image ) )
			{
				textureIndex = static_cast<int>( mesh->textures.size() );
				mesh->textures.push_back( std::move( image ) );
			}
			loadedTextures.emplace( std::move( key ), textureIndex );
			return textureIndex;
		};

	for ( const tinyobj::shape_t& shape : shapes )
	{
		size_t baseIndex = 0;
		for ( size_t faceIndex = 0; faceIndex < shape.mesh.num_face_vertices.size(); ++faceIndex )
		{
			int faceVertexCount = shape.mesh.num_face_vertices[faceIndex];
			if ( faceVertexCount != 3 )
			{
				baseIndex += static_cast<size_t>( faceVertexCount );
				continue;
			}

			int materialIndex = faceIndex < shape.mesh.material_ids.size() ? shape.mesh.material_ids[faceIndex] : -1;
			VoxelTriangleSurface surface;
			if ( materialIndex >= 0 && materialIndex < static_cast<int>( materials.size() ) )
			{
				const tinyobj::material_t& material = materials[materialIndex];
				surface.color =
					PackColor( material.diffuse[0], material.diffuse[1], material.diffuse[2], material.dissolve );
				surface.textureIndex = getTextureIndex( material.diffuse_texname );
			}

			uint32_t firstVertex = static_cast<uint32_t>( mesh->vertices.size() );
			bool validTriangle = true;
			for ( int corner = 0; corner < 3; ++corner )
			{
				const tinyobj::index_t& index = shape.mesh.indices[baseIndex + corner];
				int vertexCount = static_cast<int>( attributes.vertices.size() / 3 );
				if ( index.vertex_index < 0 || index.vertex_index >= vertexCount )
				{
					validTriangle = false;
					break;
				}
				mesh->vertices.push_back( TransformVertex( attributes, index.vertex_index, scale, zUp ) );
				if ( index.texcoord_index >= 0 &&
					 2 * index.texcoord_index + 1 < static_cast<int>( attributes.texcoords.size() ) )
				{
					surface.uv[corner] = {
						attributes.texcoords[2 * index.texcoord_index + 0],
						attributes.texcoords[2 * index.texcoord_index + 1],
					};
				}
			}

			if ( validTriangle )
			{
				mesh->indices.push_back( firstVertex + 0 );
				mesh->indices.push_back( firstVertex + 1 );
				mesh->indices.push_back( firstVertex + 2 );
				mesh->triangleColors.push_back( surface.color );
				mesh->triangleSurfaces.push_back( surface );
			}
			else
			{
				mesh->vertices.resize( firstVertex );
			}
			baseIndex += 3;
		}
	}

	if ( mesh->indices.empty() )
	{
		if ( error != nullptr )
		{
			*error = "OBJ contains no usable triangles";
		}
		return false;
	}
	return true;
}
