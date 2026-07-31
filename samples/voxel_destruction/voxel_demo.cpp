// SPDX-FileCopyrightText: 2026
// SPDX-License-Identifier: MIT

#include "gfx/draw.h"
#include "gfx/keycodes.h"
#include "gfx/projection.h"
#include "sample.h"
#include "voxel_destruction/voxel_body.h"
#include "voxel_destruction/voxel_body_update.h"
#include "voxel_destruction/voxel_connectivity.h"
#include "voxel_destruction/voxel_editing.h"
#include "voxel_destruction/voxel_fracture.h"
#include "voxel_destruction/voxel_job_system.h"
#include "voxel_destruction/voxel_obj_import.h"
#include "voxel_destruction/voxel_piece.h"
#include "voxel_destruction/voxel_replace_updater.h"
#include "voxel_destruction/voxel_serialization.h"
#include "voxel_destruction/voxel_self_test.h"
#include "voxel_destruction/voxel_types.h"

#include "box3d/box3d.h"

#include <imgui.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace
{

static const char* g_voxelAssertPhase = "voxel demo construction";

static int VoxelAssertLogHandler( const char* condition, const char* fileName, int lineNumber )
{
	std::fprintf( stderr, "BOX3D ASSERTION: %s, %s, line %d, phase: %s\n", condition, fileName, lineNumber,
				  g_voxelAssertPhase );
	std::fflush( stderr );
	FILE* file = std::fopen( "voxel_assert.log", "a" );
	if ( file != nullptr )
	{
		std::fprintf( file, "BOX3D ASSERTION: %s, %s, line %d, phase: %s\n", condition, fileName, lineNumber,
					  g_voxelAssertPhase );
		std::fclose( file );
	}
	return 1;
}

static b3Vec3 Add( b3Vec3 a, b3Vec3 b )
{
	return { a.x + b.x, a.y + b.y, a.z + b.z };
}

static b3Vec3 Scale( b3Vec3 v, float scale )
{
	return { scale * v.x, scale * v.y, scale * v.z };
}

static b3Vec3 NormalizeOrUp( b3Vec3 v )
{
	float lengthSquared = v.x * v.x + v.y * v.y + v.z * v.z;
	if ( lengthSquared < 1.0e-6f )
	{
		return { 0.0f, 1.0f, 0.0f };
	}

	return Scale( v, 1.0f / sqrtf( lengthSquared ) );
}

static float Length( b3Vec3 v )
{
	return sqrtf( v.x * v.x + v.y * v.y + v.z * v.z );
}

struct FractureComputation
{
	std::vector<int> cutIndices;
	std::vector<std::vector<int>> connectedComponents;
	std::vector<std::vector<int>> voronoiGroups;
	std::vector<VoxelPieceBuild> pieces;
	bool valid = false;
};

struct WreckingBall
{
	b3BodyId bodyId = b3_nullBodyId;
	bool fracturing = false;
	float startSpeed = 0.0f;
	b3Vec3 lastVelocity = b3Vec3_zero;
	b3Pos lastPosition = b3Vec3_zero;
	b3Quat lastRotation = b3Quat_identity;
	int birthStep = 0;
};

struct ObjImportResult
{
	VoxelPieceBuild piece;
	std::string error;
	bool valid = false;
};

struct ConnectedComponentComputation
{
	std::vector<VoxelPieceBuild> pieces;
	int componentCount = 0;
};

class VoxelDestructionDemo : public Sample
{
public:
	explicit VoxelDestructionDemo( SampleContext* context )
		: Sample( context ), m_jobs( std::max( 1, context->workerCount - 1 ) ),
		  m_bodyUpdates( m_jobs, m_voxelBodies ), m_replacements( m_worldId, m_voxelBodies ), m_random( 0xB03D2026u )
	{
		b3SetAssertFcn( VoxelAssertLogHandler );
		if ( context->restart == false )
		{
			m_camera->SetView( 30.0f, 18.0f, 18.0f, { 0.0f, 4.0f, 0.0f } );
		}

		AddGroundBox( 20.0f );
		m_launchSpeedScale = 1.0f;
		m_autoTest = getenv( "BOX3D_VOXEL_AUTOTEST" ) != nullptr;
		m_budgetAutoTest = getenv( "BOX3D_VOXEL_BUDGET_AUTOTEST" ) != nullptr;
		if ( m_budgetAutoTest )
		{
			m_autoTest = true;
			m_maxManagedBoxCount = 1;
			m_colliderBudgetHysteresis = 0;
		}
		if ( getenv( "BOX3D_VOXEL_IMMEDIATE_CCL_AUTOTEST" ) != nullptr )
		{
			m_autoTest = true;
			m_immediateFractureComputation = true;
		}
		m_updateAutoTest = getenv( "BOX3D_VOXEL_UPDATE_AUTOTEST" ) != nullptr;
		m_ballAutoTest = getenv( "BOX3D_VOXEL_BALL_AUTOTEST" ) != nullptr;
		m_ballStressTest = getenv( "BOX3D_VOXEL_BALL_STRESS_TEST" ) != nullptr;
		m_resizeAutoTest = getenv( "BOX3D_VOXEL_RESIZE_AUTOTEST" ) != nullptr;
		m_stressTest = getenv( "BOX3D_VOXEL_STRESS_TEST" ) != nullptr;
		m_cclUpdateAutoTest = getenv( "BOX3D_VOXEL_CCL_UPDATE_AUTOTEST" ) != nullptr;
		m_largeSceneAutoTest = getenv( "BOX3D_VOXEL_LARGE_SCENE_AUTOTEST" ) != nullptr;
		m_physicsToggleAutoTest = getenv( "BOX3D_VOXEL_PHYSICS_TOGGLE_AUTOTEST" ) != nullptr;
		m_initializerAutoTest = getenv( "BOX3D_VOXEL_INITIALIZER_AUTOTEST" ) != nullptr;
		m_weaponProfileAutoTest = getenv( "BOX3D_VOXEL_WEAPON_PROFILE_AUTOTEST" ) != nullptr;
		m_offscreenCleanupAutoTest = getenv( "BOX3D_VOXEL_OFFSCREEN_CLEANUP_AUTOTEST" ) != nullptr;
		const char* objAutoTestPath = getenv( "BOX3D_VOXEL_OBJ_AUTOTEST" );
		if ( objAutoTestPath != nullptr )
		{
			snprintf( m_objPath, sizeof( m_objPath ), "%s", objAutoTestPath );
			m_objAutoTest = true;
		}
		const char* unityAssetAutoTestPath = getenv( "BOX3D_VOXEL_UNITY_ASSET_AUTOTEST" );
		if ( unityAssetAutoTestPath != nullptr )
		{
			snprintf( m_unityAssetPath, sizeof( m_unityAssetPath ), "%s", unityAssetAutoTestPath );
			m_unityAssetAutoTest = true;
		}
		if ( getenv( "BOX3D_VOXEL_SELF_TEST" ) != nullptr )
		{
			std::string failure;
			bool passed = RunVoxelSelfTests( &failure );
			assert( passed && failure.empty() );
		}
		const char* replaceMode = getenv( "BOX3D_VOXEL_REPLACE_MODE" );
		if ( replaceMode != nullptr )
		{
			if ( strcmp( replaceMode, "immediate" ) == 0 )
			{
				m_replaceMode = VoxelReplaceMode::immediate;
			}
			else if ( strcmp( replaceMode, "standard" ) == 0 )
			{
				m_replaceMode = VoxelReplaceMode::standard;
			}
		}
		ResetDemo( true );
		if ( m_ballStressTest )
		{
			ApplyWeaponProfile( 1 );
		}
		if ( m_offscreenCleanupAutoTest )
		{
			std::fprintf( stderr, "offscreen cleanup autotest: started\n" );
			m_offscreenCleanupGraceFrames = 2;
			Int3 dimensions = { 2, 2, 1 };
			std::vector<VoxelCell> cells( 4, { 1, 0, 180, 180, 180, 255 } );
			b3Pos cameraOrigin = m_camera->DrawOrigin();
			b3Pos position = b3OffsetPos( cameraOrigin, b3MulSV( 5.0f, m_camera->m_forward ) );
			VoxelBody* body = CreateVoxelBody(
				dimensions, std::move( cells ), position, b3Quat_identity, true, b3Vec3_zero, b3Vec3_zero );
			m_offscreenCleanupAutoTestBodyId = body->m_id;
		}
	}

	~VoxelDestructionDemo() override
	{
		g_voxelAssertPhase = "voxel demo destruction";
		if ( m_autoTest )
		{
			assert( m_updateFinishedCount > 0 );
			assert( m_lastCreatedBodyCount > 1 );
		}
		if ( m_updateAutoTest )
		{
			assert( m_inPlaceUpdateFinishedCount > 0 );
		}
		if ( m_ballAutoTest )
		{
			assert( m_updateFinishedCount > 0 );
			assert( m_lastCreatedBodyCount > 1 );
		}
		if ( m_resizeAutoTest )
		{
			assert( m_editorOperationFinishedCount > 0 );
			assert( m_voxelBodies.empty() == false );
			assert( m_voxelBodies.front()->m_dimensions.x == 28 );
		}
		if ( m_objAutoTest )
		{
			assert( m_objImportInProgress == false );
			assert( m_objImportStatus == "OBJ voxelization loaded" );
			assert( m_voxelBodies.empty() == false );
		}
		if ( m_stressTest )
		{
			assert( m_stressTriggeredCount >= 3 );
			assert( m_updateFinishedCount >= 3 );
		}
		if ( m_cclUpdateAutoTest )
		{
			assert( m_cclFinishedCount > 0 );
			assert( m_voxelBodies.size() >= 2 );
		}
		if ( m_budgetAutoTest )
		{
			assert( m_budgetRemovedBodyCount > 0 );
		}
		if ( m_largeSceneAutoTest )
		{
			assert( m_voxelBodies.size() == 6 );
		}
		if ( m_unityAssetAutoTest )
		{
			assert( m_serializationStatus == "loaded Unity SerializedVoxelMap asset" );
			assert( m_voxelBodies.empty() == false );
		}
		if ( m_physicsToggleAutoTest )
		{
			assert( m_physicsToggleAutoTestFinished );
		}
		if ( m_initializerAutoTest )
		{
			assert( m_initializerAutoTestFinished );
			assert( m_voxelBodies.empty() == false );
		}
		if ( m_weaponProfileAutoTest )
		{
			assert( m_weaponProfileAutoTestFinished );
			assert( m_updateFinishedCount > 0 );
			assert( m_lastCreatedBodyCount > 1 );
		}
		if ( m_offscreenCleanupAutoTest )
		{
			assert( m_offscreenCleanupAutoTestFinished );
			assert( m_offscreenCleanupRemovedBodyCount > 0 );
		}
	}

	void InitializeSingleVoxelBody( Int3 dimensions, std::vector<VoxelCell> cells, b3Pos position, bool physics,
									bool immediate, std::string readyStatus )
	{
		if ( immediate )
		{
			CreateVoxelBody(
				dimensions, std::move( cells ), position, b3Quat_identity, physics, b3Vec3_zero, b3Vec3_zero );
			m_initializationStatus = readyStatus;
			m_serializationStatus = std::move( readyStatus );
			m_initializationInProgress = false;
			return;
		}

		m_initializationInProgress = true;
		m_initializationStatus = "initializing voxel object asynchronously...";
		uint64_t generation = m_sceneGeneration;
		auto piece = std::make_shared<VoxelPieceBuild>();
		piece->dimensions = dimensions;
		piece->cells = std::move( cells );
		piece->position = position;
		piece->rotation = b3Quat_identity;
		piece->linearVelocity = b3Vec3_zero;
		piece->angularVelocity = b3Vec3_zero;
		piece->dynamic = physics;
		m_jobs.Submit(
			[piece]
			{
				if ( piece->dynamic )
				{
					piece->colliderRuns = BuildVoxelBoxRuns( piece->cells, piece->dimensions );
				}
				piece->meshBuilds = BuildVoxelChunkMeshes( piece->dimensions, piece->cells );
			},
			[this, piece, generation, readyStatus = std::move( readyStatus )]() mutable
			{
				m_initializationInProgress = false;
				if ( generation != m_sceneGeneration )
				{
					m_initializationStatus = "voxel initialization discarded after scene reset";
					return;
				}
				CreateVoxelBody(
					piece->dimensions, std::move( piece->cells ), piece->position, piece->rotation, piece->dynamic,
					piece->linearVelocity, piece->angularVelocity, std::move( piece->colliderRuns ),
					std::move( piece->meshBuilds ) );
				m_lastCreatedBodyCount = 1;
				m_initializationStatus = readyStatus;
				m_serializationStatus = std::move( readyStatus );
				if ( m_initializerAutoTest )
				{
					m_initializerAutoTestFinished = true;
				}
			} );
	}

	void ResetDemo( bool forceImmediate = false )
	{
		m_sceneGeneration += 1;
		DestroyWreckingBalls();
		m_bodyUpdates.CancelAll();
		m_replacements.CancelAll();
		m_pendingConnectedComponentBodies.clear();
		m_voxelBodies.clear();
		m_voxelBudgetCleanupPending = false;

		Int3 dimensions = { 56, 42, 7 };
		std::vector<VoxelCell> cells( dimensions.x * dimensions.y * dimensions.z );

		for ( int x = 0; x < dimensions.x; ++x )
		{
			for ( int y = 0; y < dimensions.y; ++y )
			{
				for ( int z = 0; z < dimensions.z; ++z )
				{
					bool leftWindow = 9 <= x && x <= 17 && 12 <= y && y <= 25;
					bool rightWindow = 38 <= x && x <= 46 && 12 <= y && y <= 25;
					bool window = leftWindow || rightWindow;

					VoxelCell& cell = cells[GetIndex( { x, y, z }, dimensions )];
					cell.filled = window ? 0 : 1;
					cell.material = static_cast<uint8_t>( ( x / 5 + y / 5 + z ) % 4 );
					uint32_t color = static_cast<uint32_t>( GetVoxelColor( cell.material ) );
					cell.r = static_cast<uint8_t>( color >> 16 );
					cell.g = static_cast<uint8_t>( color >> 8 );
					cell.b = static_cast<uint8_t>( color );
					cell.a = 255;
				}
			}
		}

		b3Pos position = { 0.0f, 0.5f * dimensions.y * kVoxelSize, 0.0f };
		InitializeSingleVoxelBody(
			dimensions, std::move( cells ), position, m_initializationPhysics,
			forceImmediate || m_immediateInitialization, "voxel wall initialized" );
		m_lastCutVoxelCount = 0;
		m_lastCreatedBodyCount = 1;
		m_impactCooldown = 0;
		m_budgetRemovedBodyCount = 0;
		m_offscreenCleanupRemovedBodyCount = 0;
	}

	void BuildLargeVoxelScene()
	{
		m_sceneGeneration += 1;
		DestroyWreckingBalls();
		m_bodyUpdates.CancelAll();
		m_replacements.CancelAll();
		m_pendingConnectedComponentBodies.clear();
		m_voxelBodies.clear();
		m_voxelBudgetCleanupPending = false;

		Int3 dimensions = { 40, 30, 16 };
		float spacingX = dimensions.x * kVoxelSize + 1.5f;
		float spacingZ = dimensions.z * kVoxelSize + 2.0f;
		for ( int row = 0; row < m_largeSceneRows; ++row )
		{
			for ( int column = 0; column < m_largeSceneColumns; ++column )
			{
				std::vector<VoxelCell> cells( dimensions.x * dimensions.y * dimensions.z );
				for ( int x = 0; x < dimensions.x; ++x )
				{
					for ( int y = 0; y < dimensions.y; ++y )
					{
						for ( int z = 0; z < dimensions.z; ++z )
						{
							bool door = abs( x - dimensions.x / 2 ) <= 3 && y < 9;
							bool windowX = ( x + 3 * column ) % 13 >= 4 && ( x + 3 * column ) % 13 <= 8;
							bool windowY = y % 11 >= 5 && y % 11 <= 8;
							bool window = windowX && windowY;

							VoxelCell& cell = cells[GetIndex( { x, y, z }, dimensions )];
							cell.filled = ( door || window ) ? 0 : 1;
							cell.material = static_cast<uint8_t>( ( column + 2 * row + x / 8 + y / 8 ) % 5 );
							uint32_t color = static_cast<uint32_t>( GetVoxelColor( cell.material ) );
							cell.r = static_cast<uint8_t>( color >> 16 );
							cell.g = static_cast<uint8_t>( color >> 8 );
							cell.b = static_cast<uint8_t>( color );
							cell.a = 255;
						}
					}
				}

				float centeredColumn = column - 0.5f * ( m_largeSceneColumns - 1 );
				b3Pos position = {
					centeredColumn * spacingX,
					0.5f * dimensions.y * kVoxelSize,
					row * spacingZ,
				};
				CreateVoxelBody( dimensions, std::move( cells ), position, b3Quat_identity, false, b3Vec3_zero, b3Vec3_zero );
			}
		}

		m_lastCutVoxelCount = 0;
		m_lastCreatedBodyCount = static_cast<int>( m_voxelBodies.size() );
		m_budgetRemovedBodyCount = 0;
		m_camera->SetView( 45.0f, 28.0f, 32.0f,
						   { 0.0f, 0.5f * dimensions.y * kVoxelSize, 0.5f * ( m_largeSceneRows - 1 ) * spacingZ } );
	}

	void SaveFirstVoxelBody()
	{
		if ( m_voxelBodies.empty() )
		{
			m_serializationStatus = "nothing to save";
			return;
		}

		const VoxelBody& body = *m_voxelBodies.front();
		std::string error;
		if ( SaveVoxelMap( "voxel_wall.b3vox", body.m_dimensions, body.m_cells, &error ) )
		{
			m_serializationStatus = "saved voxel_wall.b3vox";
		}
		else
		{
			m_serializationStatus = error;
		}
	}

	void SaveFirstVoxelBodyAsUnityAsset()
	{
		if ( m_voxelBodies.empty() || m_unityExportPath[0] == '\0' )
		{
			m_serializationStatus = "nothing to export or Unity asset path is empty";
			return;
		}

		const VoxelBody& body = *m_voxelBodies.front();
		std::string error;
		if ( SaveUnitySerializedVoxelMapAsset(
				 m_unityExportPath, body.m_dimensions, body.m_cells, &error ) )
		{
			m_serializationStatus = "saved Unity SerializedVoxelMap asset";
		}
		else
		{
			m_serializationStatus = error;
		}
	}

	void LoadVoxelBody()
	{
		Int3 dimensions;
		std::vector<VoxelCell> cells;
		std::string error;
		if ( LoadVoxelMap( "voxel_wall.b3vox", &dimensions, &cells, &error ) == false )
		{
			m_serializationStatus = error;
			return;
		}

		DestroyWreckingBalls();
		m_sceneGeneration += 1;
		m_bodyUpdates.CancelAll();
		m_replacements.CancelAll();
		m_pendingConnectedComponentBodies.clear();
		m_voxelBodies.clear();
		m_voxelBudgetCleanupPending = false;
		b3Pos position = { 0.0f, 0.5f * dimensions.y * kVoxelSize, 0.0f };
		InitializeSingleVoxelBody(
			dimensions, std::move( cells ), position, m_initializationPhysics, m_immediateInitialization,
			"loaded voxel_wall.b3vox" );
	}

	void LoadUnityVoxelBody()
	{
		if ( m_unityAssetPath[0] == '\0' )
		{
			m_serializationStatus = "enter a Unity SerializedVoxelMap .asset path";
			return;
		}

		Int3 dimensions;
		std::vector<VoxelCell> cells;
		std::string error;
		if ( LoadUnitySerializedVoxelMapAsset( m_unityAssetPath, &dimensions, &cells, &error ) == false )
		{
			m_serializationStatus = error;
			return;
		}

		DestroyWreckingBalls();
		m_sceneGeneration += 1;
		m_bodyUpdates.CancelAll();
		m_replacements.CancelAll();
		m_pendingConnectedComponentBodies.clear();
		m_voxelBodies.clear();
		m_voxelBudgetCleanupPending = false;
		b3Pos position = { 0.0f, 0.5f * dimensions.y * kVoxelSize, 0.0f };
		InitializeSingleVoxelBody(
			dimensions, std::move( cells ), position, m_initializationPhysics, m_immediateInitialization,
			"loaded Unity SerializedVoxelMap asset" );
		m_resizeDimensions[0] = dimensions.x;
		m_resizeDimensions[1] = dimensions.y;
		m_resizeDimensions[2] = dimensions.z;
	}

	void ImportObjVoxelBody()
	{
		if ( m_objImportInProgress || m_objPath[0] == '\0' || m_objScale <= 0.0f || m_objScanVoxelSize <= 0.0f )
		{
			return;
		}

		m_objImportInProgress = true;
		m_objImportStatus = "importing OBJ...";
		std::string path = m_objPath;
		float scale = m_objScale;
		float scanVoxelSize = m_objScanVoxelSize;
		bool zUp = m_objZUp;
		uint64_t generation = m_sceneGeneration;
		auto result = std::make_shared<ObjImportResult>();

		m_jobs.Submit(
			[result, path = std::move( path ), scale, scanVoxelSize, zUp]
			{
				VoxelTriangleMesh triangleMesh;
				if ( LoadObjVoxelTriangleMesh( path.c_str(), scale, zUp, &triangleMesh, &result->error ) == false )
				{
					return;
				}

				VoxelizedTriangleMesh voxelized = VoxelizeTriangleMesh( triangleMesh, scanVoxelSize );
				if ( voxelized.cells.empty() )
				{
					result->error = "OBJ voxelization produced an empty or oversized map";
					return;
				}

				result->piece.dimensions = voxelized.dimensions;
				result->piece.cells = std::move( voxelized.cells );
				result->piece.position = { 0.0f, 0.5f * voxelized.dimensions.y * kVoxelSize, 0.0f };
				result->piece.rotation = b3Quat_identity;
				result->piece.linearVelocity = b3Vec3_zero;
				result->piece.angularVelocity = b3Vec3_zero;
				result->piece.dynamic = false;
				result->piece.colliderRuns = BuildVoxelBoxRuns( result->piece.cells, result->piece.dimensions );
				result->piece.meshBuilds = BuildVoxelChunkMeshes( result->piece.dimensions, result->piece.cells );
				result->valid = true;
			},
			[this, result, generation]
			{
				m_objImportInProgress = false;
				if ( generation != m_sceneGeneration )
				{
					m_objImportStatus = "OBJ import discarded after scene reset";
					return;
				}
				if ( result->valid == false )
				{
					m_objImportStatus = result->error;
					return;
				}

				DestroyWreckingBalls();
				m_bodyUpdates.CancelAll();
				m_replacements.CancelAll();
				m_pendingConnectedComponentBodies.clear();
				m_voxelBodies.clear();
				m_voxelBudgetCleanupPending = false;
				m_sceneGeneration += 1;
				VoxelPieceBuild& piece = result->piece;
				CreateVoxelBody( piece.dimensions, std::move( piece.cells ), piece.position, piece.rotation, false,
								 b3Vec3_zero, b3Vec3_zero, std::move( piece.colliderRuns ), std::move( piece.meshBuilds ) );
				m_resizeDimensions[0] = piece.dimensions.x;
				m_resizeDimensions[1] = piece.dimensions.y;
				m_resizeDimensions[2] = piece.dimensions.z;
				m_objImportStatus = "OBJ voxelization loaded";
			} );
	}

	void LaunchWreckingBall( b3Vec2 mousePosition )
	{
		if ( m_wreckingBalls.size() == 16 )
		{
			b3BodyId oldestBodyId = m_wreckingBalls.front().bodyId;
			if ( b3Body_IsValid( oldestBodyId ) )
			{
				b3DestroyBody( oldestBodyId );
			}
			m_wreckingBalls.erase( m_wreckingBalls.begin() );
		}

		PickRay pickRay = m_camera->BuildPickRay( mousePosition.x, mousePosition.y );
		b3Vec3 direction = b3Normalize( pickRay.translation );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.isBullet = true;
		bodyDef.position = pickRay.origin + 2.0f * direction;
		bodyDef.linearVelocity = ( m_projectileSpeed * m_launchSpeedScale ) * direction;
		bodyDef.name = "voxel wrecking ball";
		b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.density *= 4.0f;
		shapeDef.enableHitEvents = true;
		b3Sphere sphere = { b3Vec3_zero, m_projectileRadius };
		b3CreateSphereShape( bodyId, &shapeDef, &sphere );
		ApplyProjectileMass( bodyId );
		TrackWreckingBall( bodyId );
	}

	void LaunchAutomatedWreckingBall( float x = 0.0f, float y = 4.7f )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.isBullet = true;
		bodyDef.position = { x, y, 8.0f };
		bodyDef.linearVelocity = { 0.0f, 0.0f, -m_projectileSpeed };
		bodyDef.name = "automated voxel wrecking ball";
		b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.density *= 4.0f;
		shapeDef.enableHitEvents = true;
		b3Sphere sphere = { b3Vec3_zero, m_projectileRadius };
		b3CreateSphereShape( bodyId, &shapeDef, &sphere );
		ApplyProjectileMass( bodyId );
		TrackWreckingBall( bodyId );
	}

	void ApplyProjectileMass( b3BodyId bodyId )
	{
		if ( m_projectileMass <= 0.0f )
		{
			return;
		}
		b3MassData massData = b3Body_GetMassData( bodyId );
		if ( massData.mass <= 0.0f )
		{
			return;
		}
		float scale = m_projectileMass / massData.mass;
		massData.mass = m_projectileMass;
		massData.inertia.cx = Scale( massData.inertia.cx, scale );
		massData.inertia.cy = Scale( massData.inertia.cy, scale );
		massData.inertia.cz = Scale( massData.inertia.cz, scale );
		b3Body_SetMassData( bodyId, massData );
	}

	void ApplyWeaponProfile( int profile )
	{
		m_activeWeaponProfile = std::clamp( profile, 0, 2 );
		if ( m_activeWeaponProfile == 0 )
		{
			m_projectileRadius = 0.25f;
			m_projectileSpeed = 20.0f;
			m_projectileMass = 0.0f;
			m_fractureRadius = 9;
			m_seedSpawnWorldRadius = 0.0f;
			m_seedCount = 16;
			m_minFragmentVoxels = 4;
			m_minVoxelsForWreckingBallFracture = 200;
			m_wreckingBallVelocityAfterFracture = 0.8f;
			m_wreckingBallLifetimeSeconds = 5.0f;
			m_destroyFragmentEdges = true;
			m_immediateFractureComputation = false;
		}
		else if ( m_activeWeaponProfile == 1 )
		{
			m_projectileRadius = 0.5f;
			m_projectileSpeed = 40.0f;
			m_projectileMass = 100.0f;
			m_fractureRadius = 10;
			m_seedSpawnWorldRadius = 1.0f;
			m_seedCount = 30;
			m_minFragmentVoxels = 5;
			m_minVoxelsForWreckingBallFracture = 500;
			m_wreckingBallVelocityAfterFracture = 0.9f;
			m_wreckingBallLifetimeSeconds = 3.0f;
			m_destroyFragmentEdges = false;
			m_immediateFractureComputation = false;
		}
		else
		{
			m_projectileRadius = 0.5f;
			m_projectileSpeed = 50.0f;
			m_projectileMass = 300.0f;
			m_fractureRadius = 30;
			m_seedSpawnWorldRadius = 2.0f;
			m_seedCount = 40;
			m_minFragmentVoxels = 5;
			m_minVoxelsForWreckingBallFracture = 500;
			m_wreckingBallVelocityAfterFracture = 0.9f;
			m_wreckingBallLifetimeSeconds = 3.0f;
			m_destroyFragmentEdges = false;
			m_immediateFractureComputation = true;
		}
	}

	void TrackWreckingBall( b3BodyId bodyId )
	{
		WreckingBall ball;
		ball.bodyId = bodyId;
		ball.lastVelocity = b3Body_GetLinearVelocity( bodyId );
		ball.startSpeed = Length( ball.lastVelocity );
		ball.lastPosition = b3Body_GetPosition( bodyId );
		ball.lastRotation = b3Body_GetRotation( bodyId );
		ball.birthStep = m_stepCount;
		m_wreckingBalls.push_back( ball );
	}

	void UpdateWreckingBallHistory()
	{
		for ( WreckingBall& ball : m_wreckingBalls )
		{
			if ( ball.fracturing || b3Body_IsValid( ball.bodyId ) == false )
			{
				continue;
			}
			ball.lastVelocity = b3Body_GetLinearVelocity( ball.bodyId );
			ball.lastPosition = b3Body_GetPosition( ball.bodyId );
			ball.lastRotation = b3Body_GetRotation( ball.bodyId );
		}
	}

	void ResumeWreckingBall( b3BodyId bodyId )
	{
		for ( WreckingBall& ball : m_wreckingBalls )
		{
			if ( B3_ID_EQUALS( ball.bodyId, bodyId ) == false )
			{
				continue;
			}
			if ( b3Body_IsValid( ball.bodyId ) )
			{
				b3Body_SetType( ball.bodyId, b3_dynamicBody );
				b3Body_SetLinearVelocity( ball.bodyId, Scale( ball.lastVelocity, m_wreckingBallVelocityAfterFracture ) );
			}
			ball.fracturing = false;
			return;
		}
	}

	void DestroyExpiredWreckingBalls()
	{
		float hertz = std::max( 1.0f, m_context->hertz );
		int lifetimeSteps = std::max( 1, static_cast<int>( std::ceil( m_wreckingBallLifetimeSeconds * hertz ) ) );
		for ( auto iterator = m_wreckingBalls.begin(); iterator != m_wreckingBalls.end(); )
		{
			if ( m_stepCount - iterator->birthStep < lifetimeSteps )
			{
				++iterator;
				continue;
			}
			if ( b3Body_IsValid( iterator->bodyId ) )
			{
				b3DestroyBody( iterator->bodyId );
			}
			iterator = m_wreckingBalls.erase( iterator );
		}
	}

	void CarveCenterVoxelBody()
	{
		if ( m_voxelBodies.empty() )
		{
			return;
		}

		VoxelBody& body = *m_voxelBodies.front();
		if ( body.m_updating )
		{
			return;
		}

		Int3 center = { body.m_dimensions.x / 2, body.m_dimensions.y / 2, body.m_dimensions.z / 2 };
		constexpr int radius = 3;
		int editedCount = 0;
		for ( int x = center.x - radius; x <= center.x + radius; ++x )
		{
			for ( int y = center.y - radius; y <= center.y + radius; ++y )
			{
				for ( int z = center.z - radius; z <= center.z + radius; ++z )
				{
					int dx = x - center.x;
					int dy = y - center.y;
					int dz = z - center.z;
					if ( dx * dx + dy * dy + dz * dz > radius * radius )
					{
						continue;
					}
					editedCount += body.SetVoxelFilled( { x, y, z }, false );
				}
			}
		}

		if ( editedCount == 0 )
		{
			return;
		}

		m_inPlaceUpdateStartedCount += 1;
		m_bodyUpdates.RequestUpdate(
			body,
			[this]( VoxelBody* updatedBody, bool applied ) { OnVoxelMapUpdateCompleted( updatedBody, applied ); } );
	}

	void HollowFirstVoxelBody()
	{
		if ( m_voxelBodies.empty() )
		{
			return;
		}

		VoxelBody& body = *m_voxelBodies.front();
		if ( body.m_updating )
		{
			return;
		}

		std::vector<VoxelCell> hollow = HollowVoxelMap( body.m_cells, body.m_dimensions, m_hollowWallThickness );
		if ( hollow.empty() )
		{
			return;
		}

		int editedCount = 0;
		for ( int index = 0; index < static_cast<int>( hollow.size() ); ++index )
		{
			bool filled = hollow[index].filled != 0;
			if ( filled != ( body.m_cells[index].filled != 0 ) )
			{
				editedCount += body.SetVoxelFilled( GetCoordinates( index, body.m_dimensions ), filled );
			}
		}
		if ( editedCount == 0 )
		{
			return;
		}

		m_inPlaceUpdateStartedCount += 1;
		m_bodyUpdates.RequestUpdate(
			body,
			[this]( VoxelBody* updatedBody, bool applied ) { OnVoxelMapUpdateCompleted( updatedBody, applied ); } );
	}

	void SplitFirstVoxelBodyForCclTest()
	{
		if ( m_voxelBodies.empty() )
		{
			return;
		}

		VoxelBody& body = *m_voxelBodies.front();
		if ( body.m_updating )
		{
			return;
		}

		int splitX = body.m_dimensions.x / 2;
		int editedCount = 0;
		for ( int y = 0; y < body.m_dimensions.y; ++y )
		{
			for ( int z = 0; z < body.m_dimensions.z; ++z )
			{
				editedCount += body.SetVoxelFilled( { splitX, y, z }, false );
			}
		}
		if ( editedCount == 0 )
		{
			return;
		}

		m_inPlaceUpdateStartedCount += 1;
		m_bodyUpdates.RequestUpdate(
			body,
			[this]( VoxelBody* updatedBody, bool applied ) { OnVoxelMapUpdateCompleted( updatedBody, applied ); } );
	}

	void OnVoxelMapUpdateCompleted( VoxelBody* body, bool applied )
	{
		if ( applied == false )
		{
			return;
		}

		m_inPlaceUpdateFinishedCount += 1;
		if ( m_connectedComponentsOnVoxelUpdate == false || body == nullptr )
		{
			return;
		}

		if ( std::find( m_pendingConnectedComponentBodies.begin(), m_pendingConnectedComponentBodies.end(), body->m_id ) ==
			 m_pendingConnectedComponentBodies.end() )
		{
			m_pendingConnectedComponentBodies.push_back( body->m_id );
		}
	}

	void StartConnectedComponentExtraction( VoxelBody& target )
	{
		if ( target.m_updating )
		{
			return;
		}

		target.m_updating = true;
		uint64_t targetId = target.m_id;
		uint64_t targetVersion = target.m_version;
		VoxelSourceSnapshot source = {
			target.m_dimensions,
			target.m_cells,
			b3Body_GetPosition( target.m_bodyId ),
			b3Body_GetRotation( target.m_bodyId ),
			target.m_dynamic ? b3Body_GetLinearVelocity( target.m_bodyId ) : b3Vec3_zero,
			target.m_dynamic ? b3Body_GetAngularVelocity( target.m_bodyId ) : b3Vec3_zero,
			target.m_dynamic,
		};
		bool keepGrounded = m_keepGroundedComponent;
		bool newPhysicsEnabled = m_newComponentPhysicsEnabled;
		int minimumVoxels = m_minConnectedComponentVoxels;
		auto result = std::make_shared<ConnectedComponentComputation>();
		m_cclStartedCount += 1;

		auto work = [result, source = std::move( source ), keepGrounded, newPhysicsEnabled, minimumVoxels]() mutable
		{
			std::vector<std::vector<int>> components = LabelVoxelComponents( source.cells, source.dimensions );
			result->componentCount = static_cast<int>( components.size() );
			if ( components.size() <= 1 )
			{
				return;
			}

			int groundedIndex = keepGrounded ? FindMostGroundedComponent( components, source.dimensions ) : 0;
			for ( int componentIndex = 0; componentIndex < static_cast<int>( components.size() ); ++componentIndex )
			{
				const std::vector<int>& component = components[componentIndex];
				bool retainedOriginalComponent = componentIndex == groundedIndex;
				if ( retainedOriginalComponent == false && static_cast<int>( component.size() ) < minimumVoxels )
				{
					continue;
				}

				bool dynamic = retainedOriginalComponent ? source.dynamic : newPhysicsEnabled;
				result->pieces.push_back( BuildVoxelPiece( source, component, dynamic, b3Vec3_zero ) );
			}
		};

		auto completion = [this, result, targetId, targetVersion]
		{
			VoxelBody* current = nullptr;
			for ( const std::unique_ptr<VoxelBody>& body : m_voxelBodies )
			{
				if ( body->m_id == targetId )
				{
					current = body.get();
					break;
				}
			}
			if ( current == nullptr )
			{
				return;
			}
			if ( current->m_version != targetVersion )
			{
				current->m_updating = false;
				m_cclFinishedCount += 1;
				return;
			}

			if ( result->componentCount <= 1 || result->pieces.empty() )
			{
				current->m_updating = false;
				m_cclFinishedCount += 1;
				return;
			}

			m_replacements.Replace(
				current, std::move( result->pieces ), m_replaceMode,
				[this]
				{
					m_cclFinishedCount += 1;
					m_voxelBudgetCleanupPending = true;
				} );
		};

		if ( m_immediateConnectedComponents )
		{
			work();
			completion();
		}
		else
		{
			m_jobs.Submit( std::move( work ), std::move( completion ) );
		}
	}

	void ProcessPendingConnectedComponents()
	{
		std::vector<uint64_t> pending = std::move( m_pendingConnectedComponentBodies );
		m_pendingConnectedComponentBodies.clear();
		for ( uint64_t id : pending )
		{
			for ( const std::unique_ptr<VoxelBody>& body : m_voxelBodies )
			{
				if ( body->m_id == id )
				{
					StartConnectedComponentExtraction( *body );
					break;
				}
			}
		}
	}

	void ResizeFirstVoxelBody()
	{
		if ( m_voxelBodies.empty() )
		{
			return;
		}

		VoxelBody& body = *m_voxelBodies.front();
		Int3 targetDimensions = { m_resizeDimensions[0], m_resizeDimensions[1], m_resizeDimensions[2] };
		int64_t targetVoxelCount =
			static_cast<int64_t>( targetDimensions.x ) * targetDimensions.y * targetDimensions.z;
		if ( body.m_updating || targetDimensions.x <= 0 || targetDimensions.y <= 0 || targetDimensions.z <= 0 ||
			 targetVoxelCount > 4 * 1024 * 1024 )
		{
			return;
		}

		body.m_updating = true;
		uint64_t bodyId = body.m_id;
		uint64_t version = body.m_version;
		Int3 sourceDimensions = body.m_dimensions;
		std::vector<VoxelCell> sourceCells = body.m_cells;
		b3Pos position = b3Body_GetPosition( body.m_bodyId );
		b3Quat rotation = b3Body_GetRotation( body.m_bodyId );
		b3Vec3 linearVelocity = body.m_dynamic ? b3Body_GetLinearVelocity( body.m_bodyId ) : b3Vec3_zero;
		b3Vec3 angularVelocity = body.m_dynamic ? b3Body_GetAngularVelocity( body.m_bodyId ) : b3Vec3_zero;
		bool dynamic = body.m_dynamic;
		auto piece = std::make_shared<VoxelPieceBuild>();
		m_editorOperationStartedCount += 1;

		m_jobs.Submit(
			[piece, sourceDimensions, sourceCells = std::move( sourceCells ), targetDimensions, position, rotation,
			 linearVelocity, angularVelocity, dynamic]() mutable
			{
				piece->dimensions = targetDimensions;
				piece->cells = ResizeVoxelMapNearest( sourceCells, sourceDimensions, targetDimensions );
				piece->position = position;
				piece->rotation = rotation;
				piece->linearVelocity = linearVelocity;
				piece->angularVelocity = angularVelocity;
				piece->dynamic = dynamic;
				piece->colliderRuns = BuildVoxelBoxRuns( piece->cells, targetDimensions );
				piece->meshBuilds = BuildVoxelChunkMeshes( targetDimensions, piece->cells );
			},
			[this, piece, bodyId, version]
			{
				VoxelBody* current = nullptr;
				for ( const std::unique_ptr<VoxelBody>& candidate : m_voxelBodies )
				{
					if ( candidate->m_id == bodyId )
					{
						current = candidate.get();
						break;
					}
				}
				if ( current == nullptr )
				{
					return;
				}
				if ( current->m_version != version )
				{
					current->m_updating = false;
					return;
				}

				std::vector<VoxelPieceBuild> pieces;
				pieces.push_back( std::move( *piece ) );
				m_replacements.Replace(
					current, std::move( pieces ), m_replaceMode,
					[this]
					{
						m_editorOperationFinishedCount += 1;
						m_voxelBudgetCleanupPending = true;
					} );
			} );
	}

	void DestroyWreckingBalls()
	{
		for ( WreckingBall& ball : m_wreckingBalls )
		{
			if ( b3Body_IsValid( ball.bodyId ) )
			{
				b3DestroyBody( ball.bodyId );
			}
		}
		m_wreckingBalls.clear();
	}

	VoxelBody* CreateVoxelBody( Int3 dimensions, std::vector<VoxelCell> cells, b3Pos position, b3Quat rotation, bool dynamic,
								b3Vec3 linearVelocity, b3Vec3 angularVelocity,
								std::vector<VoxelBoxRun> colliderRuns = {}, VoxelChunkMeshBuildBatch meshBuilds = {} )
	{
		auto body = std::make_unique<VoxelBody>( m_worldId, dimensions, std::move( cells ), position, rotation, dynamic,
											   linearVelocity, angularVelocity, std::move( colliderRuns ),
											   std::move( meshBuilds ) );
		VoxelBody* result = body.get();
		m_voxelBodies.push_back( std::move( body ) );
		return result;
	}

	void Fracture( VoxelBody* target, b3Pos worldPoint, float approachSpeed, int fractureVoxelRadius = -1,
				   float seedSpawnWorldRadius = -1.0f, std::function<void( bool )> completion = {} )
	{
		if ( target == nullptr || target->m_updating || target->GetFilledCount() < 2 * m_minFragmentVoxels )
		{
			if ( completion )
			{
				completion( false );
			}
			return;
		}

		b3Vec3 impactLocal = b3Body_GetLocalPoint( target->m_bodyId, worldPoint );
		int effectiveVoxelRadius = fractureVoxelRadius > 0 ? fractureVoxelRadius : m_fractureRadius;
		float radius = effectiveVoxelRadius * kVoxelSize;
		float effectiveSeedRadius = seedSpawnWorldRadius > 0.0f ? seedSpawnWorldRadius : 0.8f * radius;
		std::uniform_real_distribution<float> distribution( -1.0f, 1.0f );
		std::vector<b3Vec3> seeds;
		seeds.reserve( m_seedCount );
		for ( int i = 0; i < m_seedCount; ++i )
		{
			b3Vec3 offset;
			do
			{
				offset = { distribution( m_random ), distribution( m_random ), distribution( m_random ) };
			} while ( offset.x * offset.x + offset.y * offset.y + offset.z * offset.z > 1.0f );

			seeds.push_back( Add( impactLocal, Scale( offset, effectiveSeedRadius ) ) );
		}

		target->m_updating = true;
		uint64_t targetId = target->m_id;
		uint64_t targetVersion = target->m_version;
		Int3 dimensions = target->m_dimensions;
		VoxelSourceSnapshot source = {
			target->m_dimensions,
			target->m_cells,
			b3Body_GetPosition( target->m_bodyId ),
			b3Body_GetRotation( target->m_bodyId ),
			target->m_dynamic ? b3Body_GetLinearVelocity( target->m_bodyId ) : b3Vec3_zero,
			target->m_dynamic ? b3Body_GetAngularVelocity( target->m_bodyId ) : b3Vec3_zero,
			target->m_dynamic,
		};
		bool destroyFragmentEdges = m_destroyFragmentEdges;
		bool destroyOuterLayer = m_destroyOuterLayer;
		bool keepGroundedComponent = m_keepGroundedComponent;
		int minimumFragmentVoxels = m_minFragmentVoxels;
		auto result = std::make_shared<FractureComputation>();
		m_updateStartedCount += 1;

		auto fractureWork =
			[result, dimensions, source = std::move( source ), seeds = std::move( seeds ), impactLocal, radius,
			 destroyFragmentEdges, destroyOuterLayer, keepGroundedComponent, minimumFragmentVoxels, approachSpeed]() mutable
			{
				result->cutIndices = CutVoxelSphere( source.cells, dimensions, impactLocal, radius );

				if ( result->cutIndices.size() < 2 )
				{
					return;
				}

				result->voronoiGroups = GenerateVoxelVoronoiGroups( result->cutIndices, dimensions, seeds );
				std::vector<VoxelCell> remainingCells = source.cells;
				for ( int index : result->cutIndices )
				{
					remainingCells[index].filled = 0;
				}

				if ( destroyFragmentEdges )
				{
					for ( std::vector<int>& group : result->voronoiGroups )
					{
						group = DestroyVoxelFragmentEdges( group, dimensions );
					}
				}

				if ( destroyOuterLayer )
				{
					for ( std::vector<int>& group : result->voronoiGroups )
					{
						group = DestroyVoxelOuterLayer( group, dimensions );
					}
				}

				result->connectedComponents = LabelVoxelComponents( remainingCells, dimensions );
				float kickSpeed = std::min( 9.0f, 0.35f * approachSpeed );
				int staticComponentIndex =
					keepGroundedComponent ? FindMostGroundedComponent( result->connectedComponents, dimensions ) : 0;

				for ( int componentIndex = 0; componentIndex < static_cast<int>( result->connectedComponents.size() );
					  ++componentIndex )
				{
					const std::vector<int>& component = result->connectedComponents[componentIndex];
					if ( static_cast<int>( component.size() ) < minimumFragmentVoxels )
					{
						continue;
					}

					bool keepStatic = source.dynamic == false && componentIndex == staticComponentIndex;
					b3Vec3 kick = b3Vec3_zero;
					if ( keepStatic == false )
					{
						Int3 p = GetCoordinates( component.front(), dimensions );
						b3Vec3 center = GetCellCenter( p, dimensions );
						kick = Scale(
							NormalizeOrUp(
								{ center.x - impactLocal.x, center.y - impactLocal.y + 0.2f, center.z - impactLocal.z } ),
							0.45f * kickSpeed );
					}
					result->pieces.push_back( BuildVoxelPiece( source, component, keepStatic == false, kick ) );
				}

				for ( const std::vector<int>& group : result->voronoiGroups )
				{
					if ( static_cast<int>( group.size() ) < minimumFragmentVoxels )
					{
						continue;
					}

					b3Vec3 centroid = b3Vec3_zero;
					for ( int index : group )
					{
						centroid = Add( centroid, GetCellCenter( GetCoordinates( index, dimensions ), dimensions ) );
					}
					centroid = Scale( centroid, 1.0f / static_cast<float>( group.size() ) );
					b3Vec3 direction = NormalizeOrUp(
						{ centroid.x - impactLocal.x, centroid.y - impactLocal.y + 0.25f, centroid.z - impactLocal.z } );
					result->pieces.push_back( BuildVoxelPiece( source, group, true, Scale( direction, kickSpeed ) ) );
				}
				result->valid = true;
			};
		auto fractureCompletion =
			[this, result, targetId, targetVersion, completion = std::move( completion )]() mutable
			{
				VoxelBody* currentTarget = nullptr;
				for ( const std::unique_ptr<VoxelBody>& body : m_voxelBodies )
				{
					if ( body->m_id == targetId )
					{
						currentTarget = body.get();
						break;
					}
				}

				if ( currentTarget == nullptr || currentTarget->m_version != targetVersion )
				{
					if ( currentTarget != nullptr )
					{
						currentTarget->m_updating = false;
						m_updateFinishedCount += 1;
					}
					if ( completion )
					{
						completion( false );
					}
					return;
				}

				if ( result->valid )
				{
					ApplyFracture( currentTarget, *result, std::move( completion ) );
				}
				else
				{
					currentTarget->m_updating = false;
					m_updateFinishedCount += 1;
					if ( completion )
					{
						completion( false );
					}
				}
			};

		if ( m_immediateFractureComputation )
		{
			fractureWork();
			fractureCompletion();
		}
		else
		{
			m_jobs.Submit( std::move( fractureWork ), std::move( fractureCompletion ) );
		}
	}

	void ApplyFracture( VoxelBody* target, FractureComputation& result, std::function<void( bool )> completion = {} )
	{
		m_lastCutVoxelCount = static_cast<int>( result.cutIndices.size() );
		m_lastCreatedBodyCount = static_cast<int>( result.pieces.size() );
		m_impactCooldown = 8;
		m_replacements.Replace(
			target, std::move( result.pieces ), m_replaceMode,
			[this, completion = std::move( completion )]() mutable
			{
				m_updateFinishedCount += 1;
				m_voxelBudgetCleanupPending = true;
				if ( completion )
				{
					completion( true );
				}
			} );
	}

	bool IsBodyInCameraView( const VoxelBody& body ) const
	{
		if ( m_camera->m_width <= 0 || m_camera->m_height <= 0 || b3Body_IsValid( body.m_bodyId ) == false )
		{
			return true;
		}

		b3AABB bounds = b3Body_ComputeAABB( body.m_bodyId );
		if ( b3AABB_Overlaps( bounds, m_camera->DrawBounds() ) == false )
		{
			return false;
		}

		b3Pos drawOrigin = m_camera->DrawOrigin();
		if ( drawOrigin.x >= bounds.lowerBound.x && drawOrigin.x <= bounds.upperBound.x &&
			 drawOrigin.y >= bounds.lowerBound.y && drawOrigin.y <= bounds.upperBound.y &&
			 drawOrigin.z >= bounds.lowerBound.z && drawOrigin.z <= bounds.upperBound.z )
		{
			return true;
		}

		const float coordinates[3][2] = {
			{ bounds.lowerBound.x, bounds.upperBound.x },
			{ bounds.lowerBound.y, bounds.upperBound.y },
			{ bounds.lowerBound.z, bounds.upperBound.z },
		};
		bool outsideLeft = true;
		bool outsideRight = true;
		bool outsideBottom = true;
		bool outsideTop = true;
		bool behindCamera = true;
		for ( int x = 0; x < 2; ++x )
		{
			for ( int y = 0; y < 2; ++y )
			{
				for ( int z = 0; z < 2; ++z )
				{
					b3Vec3 relativePoint = b3SubPos(
						b3ToPos( { coordinates[0][x], coordinates[1][y], coordinates[2][z] } ), drawOrigin );
					Vec4 viewPoint = MulMV4( m_camera->View(), MakeVec4( relativePoint.x, relativePoint.y, relativePoint.z, 1.0f ) );
					Vec4 clipPoint = MulMV4( m_camera->Proj(), viewPoint );
					outsideLeft = outsideLeft && clipPoint.x < -clipPoint.w;
					outsideRight = outsideRight && clipPoint.x > clipPoint.w;
					outsideBottom = outsideBottom && clipPoint.y < -clipPoint.w;
					outsideTop = outsideTop && clipPoint.y > clipPoint.w;
					behindCamera = behindCamera && clipPoint.w <= 0.0f;
				}
			}
		}
		return outsideLeft == false && outsideRight == false && outsideBottom == false && outsideTop == false &&
			   behindCamera == false;
	}

	void CleanupOffscreenFragments()
	{
		m_offscreenCleanupSmallBodyCount = 0;
		m_offscreenCleanupProtectedBodyCount = 0;
		m_offscreenCleanupEligibleBodyCount = 0;
		m_offscreenCleanupOutsideBodyCount = 0;
		m_offscreenCleanupWaitingBodyCount = 0;
		if ( m_cleanupOffscreenFragments == false || m_context->minimized || m_context->pause )
		{
			return;
		}

		m_voxelBodies.erase(
			std::remove_if(
				m_voxelBodies.begin(), m_voxelBodies.end(),
				[this]( const std::unique_ptr<VoxelBody>& body )
				{
					if ( body->GetFilledCount() > m_offscreenCleanupMaxVoxelCount )
					{
						body->m_offscreenFrames = 0;
						return false;
					}
					m_offscreenCleanupSmallBodyCount += 1;

					bool protectedBody = body->m_dynamic == false || body->m_updating ||
										 b3Body_IsValid( body->m_bodyId ) == false ||
										 b3Body_IsEnabled( body->m_bodyId ) == false ||
										 m_replacements.OwnsBody( body->m_id );
					if ( protectedBody )
					{
						m_offscreenCleanupProtectedBodyCount += 1;
						body->m_offscreenFrames = 0;
						return false;
					}
					m_offscreenCleanupEligibleBodyCount += 1;

					if ( IsBodyInCameraView( *body ) )
					{
						body->m_offscreenFrames = 0;
						return false;
					}
					m_offscreenCleanupOutsideBodyCount += 1;

					body->m_offscreenFrames += 1;
					if ( body->m_offscreenFrames < m_offscreenCleanupGraceFrames || m_bodyDeletionBudgetRemaining <= 0 )
					{
						m_offscreenCleanupWaitingBodyCount += 1;
						return false;
					}

					m_bodyDeletionBudgetRemaining -= 1;
					m_offscreenCleanupRemovedBodyCount += 1;
					if ( body->m_id == m_offscreenCleanupAutoTestBodyId )
					{
						m_offscreenCleanupAutoTestFinished = true;
						std::fprintf( stderr, "offscreen cleanup autotest: removed body %llu\n",
									  static_cast<unsigned long long>( body->m_id ) );
					}
					return true;
				} ),
			m_voxelBodies.end() );
	}

	void ManageVoxelObjectBudget()
	{
		int totalBoxCount = 0;
		for ( const std::unique_ptr<VoxelBody>& body : m_voxelBodies )
		{
			totalBoxCount += body->m_boxCount;
		}

		if ( totalBoxCount > m_maxManagedBoxCount )
		{
			m_voxelBudgetCleanupPending = true;
		}
		if ( m_voxelBudgetCleanupPending == false )
		{
			return;
		}

		int targetBoxCount = std::max( 0, m_maxManagedBoxCount - m_colliderBudgetHysteresis );
		if ( totalBoxCount <= targetBoxCount )
		{
			m_voxelBudgetCleanupPending = false;
			return;
		}

		std::vector<VoxelBody*> candidates;
		candidates.reserve( m_voxelBodies.size() );
		for ( const std::unique_ptr<VoxelBody>& body : m_voxelBodies )
		{
			if ( body->m_dynamic && body->m_updating == false && b3Body_IsValid( body->m_bodyId ) &&
				 b3Body_IsEnabled( body->m_bodyId ) && m_replacements.OwnsBody( body->m_id ) == false &&
				 body->GetFilledCount() < m_manageFragmentBelowVoxelCount )
			{
				candidates.push_back( body.get() );
			}
		}
		std::sort( candidates.begin(), candidates.end(),
				   []( const VoxelBody* a, const VoxelBody* b ) { return a->m_id < b->m_id; } );

		std::vector<uint64_t> removeIds;
		for ( VoxelBody* candidate : candidates )
		{
			if ( totalBoxCount <= targetBoxCount || m_bodyDeletionBudgetRemaining <= 0 )
			{
				break;
			}
			totalBoxCount -= candidate->m_boxCount;
			removeIds.push_back( candidate->m_id );
			m_bodyDeletionBudgetRemaining -= 1;
			m_budgetRemovedBodyCount += 1;
		}
		m_voxelBudgetCleanupPending = totalBoxCount > targetBoxCount;
		if ( removeIds.empty() )
		{
			return;
		}
		m_voxelBodies.erase(
			std::remove_if( m_voxelBodies.begin(), m_voxelBodies.end(),
							[&removeIds]( const std::unique_ptr<VoxelBody>& body )
							{
								return std::binary_search( removeIds.begin(), removeIds.end(), body->m_id );
							} ),
			m_voxelBodies.end() );
	}

	void ProcessHitEvents()
	{
		if ( m_impactCooldown > 0 )
		{
			m_impactCooldown -= 1;
			return;
		}

		if ( m_wreckingBalls.empty() )
		{
			return;
		}

		b3ContactEvents events = b3World_GetContactEvents( m_worldId );
		for ( int i = 0; i < events.hitCount; ++i )
		{
			const b3ContactHitEvent& event = events.hitEvents[i];
			if ( b3Shape_IsValid( event.shapeIdA ) == false || b3Shape_IsValid( event.shapeIdB ) == false )
			{
				continue;
			}

			b3BodyId bodyA = b3Shape_GetBody( event.shapeIdA );
			b3BodyId bodyB = b3Shape_GetBody( event.shapeIdB );
			VoxelBody* target = nullptr;
			WreckingBall* wreckingBall = nullptr;

			for ( WreckingBall& candidate : m_wreckingBalls )
			{
				if ( candidate.fracturing )
				{
					continue;
				}

				if ( B3_ID_EQUALS( bodyA, candidate.bodyId ) )
				{
					wreckingBall = &candidate;
					target = static_cast<VoxelBody*>( b3Body_GetUserData( bodyB ) );
					break;
				}

				if ( B3_ID_EQUALS( bodyB, candidate.bodyId ) )
				{
					wreckingBall = &candidate;
					target = static_cast<VoxelBody*>( b3Body_GetUserData( bodyA ) );
					break;
				}
			}

			if ( wreckingBall != nullptr && target != nullptr && target->m_updating == false &&
				 target->GetFilledCount() >= m_minVoxelsForWreckingBallFracture )
			{
				float remainingSpeed = wreckingBall->startSpeed > 1.0e-5f ? Length( wreckingBall->lastVelocity ) /
																		  wreckingBall->startSpeed
																		: 0.0f;
				int destructionVoxelRadius = static_cast<int>( m_fractureRadius * remainingSpeed );
				if ( destructionVoxelRadius < m_minDestructionVoxelRadius )
				{
					continue;
				}

				b3BodyId ballId = wreckingBall->bodyId;
				wreckingBall->fracturing = true;
				b3Body_SetType( ballId, b3_kinematicBody );
				b3Body_SetTransform( ballId, wreckingBall->lastPosition, wreckingBall->lastRotation );
				b3Body_SetLinearVelocity( ballId, b3Vec3_zero );
				float baseSeedRadius =
					m_seedSpawnWorldRadius > 0.0f ? m_seedSpawnWorldRadius : 0.8f * m_fractureRadius * kVoxelSize;
				float seedRadius = baseSeedRadius * remainingSpeed;
				Fracture(
					target, event.point, event.approachSpeed, destructionVoxelRadius, seedRadius,
					[this, ballId]( bool )
					{
						ResumeWreckingBall( ballId );
					} );
				break;
			}
		}
	}

	void Step() override
	{
		g_voxelAssertPhase = "physics step";
		Sample::Step();
		m_bodyDeletionBudgetRemaining = m_maxBodyDeletesPerFrame;
		if ( m_weaponProfileAutoTest && m_weaponProfileAutoTestFinished == false && m_stepCount >= 2 )
		{
			ApplyWeaponProfile( 2 );
			LaunchAutomatedWreckingBall();
			assert( m_wreckingBalls.empty() == false );
			b3MassData massData = b3Body_GetMassData( m_wreckingBalls.back().bodyId );
			assert( fabsf( massData.mass - 300.0f ) < 0.01f );
			assert( m_fractureRadius == 30 && m_seedCount == 40 && m_immediateFractureComputation );
			DestroyWreckingBalls();
			ApplyWeaponProfile( 1 );
			LaunchAutomatedWreckingBall();
			m_weaponProfileAutoTestFinished = true;
		}
		if ( m_initializerAutoTest && m_initializerAutoTestTriggered == false && m_stepCount >= 2 )
		{
			m_initializerAutoTestTriggered = true;
			m_immediateInitialization = false;
			ResetDemo();
		}
		if ( m_physicsToggleAutoTest && m_physicsToggleAutoTestFinished == false && m_stepCount >= 2 &&
			 m_voxelBodies.empty() == false )
		{
			VoxelBody& body = *m_voxelBodies.front();
			body.SetPhysicsEnabled( true );
			assert( body.m_dynamic );
			assert( body.m_boxCount > 0 );
			b3Vec3 expectedLinearVelocity = { 1.25f, 2.5f, -3.75f };
			b3Vec3 expectedAngularVelocity = { -0.5f, 0.75f, 1.0f };
			b3Body_SetLinearVelocity( body.m_bodyId, expectedLinearVelocity );
			b3Body_SetAngularVelocity( body.m_bodyId, expectedAngularVelocity );
			body.SetKinematic( true );
			assert( body.IsKinematic() );
			body.SetKinematic( false );
			assert( body.IsKinematic() == false );
			b3Vec3 restoredLinearVelocity = b3Body_GetLinearVelocity( body.m_bodyId );
			b3Vec3 restoredAngularVelocity = b3Body_GetAngularVelocity( body.m_bodyId );
			assert( Length( restoredLinearVelocity - expectedLinearVelocity ) < 1.0e-5f );
			assert( Length( restoredAngularVelocity - expectedAngularVelocity ) < 1.0e-5f );
			body.SetPhysicsEnabled( false );
			assert( body.m_dynamic == false );
			assert( body.m_collisionMeshes.empty() == false );
			m_physicsToggleAutoTestFinished = true;
		}
		if ( m_updateAutoTest && m_updateAutoTestTriggered == false && m_stepCount >= 2 )
		{
			m_updateAutoTestTriggered = true;
			CarveCenterVoxelBody();
			if ( m_voxelBodies.empty() == false )
			{
				VoxelBody& body = *m_voxelBodies.front();
				Int3 secondEdit = { body.m_dimensions.x / 2 + 4, body.m_dimensions.y / 2, body.m_dimensions.z / 2 };
				if ( body.SetVoxelFilled( secondEdit, false ) )
				{
					m_inPlaceUpdateStartedCount += 1;
					m_bodyUpdates.RequestUpdate(
						body,
						[this]( VoxelBody* updatedBody, bool applied )
						{ OnVoxelMapUpdateCompleted( updatedBody, applied ); } );
				}
			}
		}
		if ( m_ballAutoTest && m_ballAutoTestTriggered == false && m_stepCount >= 2 )
		{
			m_ballAutoTestTriggered = true;
			LaunchAutomatedWreckingBall();
		}
		if ( m_ballStressTest && m_ballStressLaunchedCount < 36 && m_stepCount >= 2 && m_stepCount % 8 == 0 )
		{
			int column = m_ballStressLaunchedCount % 9;
			int row = ( m_ballStressLaunchedCount / 9 ) % 4;
			float x = -4.4f + 1.1f * column;
			float y = 1.8f + 1.5f * row;
			LaunchAutomatedWreckingBall( x, y );
			m_ballStressLaunchedCount += 1;
		}
		if ( m_ballStressTest && m_stepCount == 360 )
		{
			m_camera->SetView( 30.0f, 18.0f, 18.0f, { 1000.0f, 4.0f, 1000.0f } );
		}
		if ( m_resizeAutoTest && m_resizeAutoTestTriggered == false && m_stepCount >= 2 )
		{
			m_resizeAutoTestTriggered = true;
			m_resizeDimensions[0] = 28;
			m_resizeDimensions[1] = 21;
			m_resizeDimensions[2] = 7;
			ResizeFirstVoxelBody();
		}
		if ( m_objAutoTest && m_objAutoTestTriggered == false && m_stepCount >= 2 )
		{
			m_objAutoTestTriggered = true;
			ImportObjVoxelBody();
		}
		if ( m_stressTest && m_stressTriggeredCount < 6 && m_stepCount >= 5 &&
			 ( m_stepCount - 5 ) % 50 == 0 )
		{
			auto targetIterator = std::max_element(
				m_voxelBodies.begin(), m_voxelBodies.end(),
				[]( const std::unique_ptr<VoxelBody>& a, const std::unique_ptr<VoxelBody>& b )
				{
					int aCount = a->m_updating ? -1 : a->GetFilledCount();
					int bCount = b->m_updating ? -1 : b->GetFilledCount();
					return aCount < bCount;
				} );
			if ( targetIterator != m_voxelBodies.end() && ( *targetIterator )->m_updating == false &&
				 ( *targetIterator )->GetFilledCount() >= 2 * m_minFragmentVoxels )
			{
				VoxelBody* target = targetIterator->get();
				b3Pos point = b3Body_GetWorldPoint(
					target->m_bodyId, { 0.0f, 0.0f, 0.5f * target->m_dimensions.z * kVoxelSize } );
				Fracture( target, point, 20.0f );
				m_stressTriggeredCount += 1;
			}
		}
		if ( m_cclUpdateAutoTest && m_cclUpdateAutoTestTriggered == false && m_stepCount >= 2 )
		{
			m_cclUpdateAutoTestTriggered = true;
			SplitFirstVoxelBodyForCclTest();
		}
		if ( m_largeSceneAutoTest && m_largeSceneAutoTestTriggered == false && m_stepCount >= 2 )
		{
			m_largeSceneAutoTestTriggered = true;
			BuildLargeVoxelScene();
		}
		if ( m_unityAssetAutoTest && m_unityAssetAutoTestTriggered == false && m_stepCount >= 2 )
		{
			m_unityAssetAutoTestTriggered = true;
			LoadUnityVoxelBody();
		}
		if ( m_autoTest && m_autoTestTriggered == false && m_stepCount >= 5 && m_voxelBodies.empty() == false )
		{
			VoxelBody* target = m_voxelBodies.front().get();
			if ( target->m_updating == false )
			{
				m_autoTestTriggered = true;
				b3Pos point =
					b3Body_GetWorldPoint( target->m_bodyId, { 0.0f, 0.5f, 0.5f * target->m_dimensions.z * kVoxelSize } );
				Fracture( target, point, 20.0f * m_launchSpeedScale );
			}
		}
		g_voxelAssertPhase = "job completions";
		m_jobs.PumpCompletions();
		g_voxelAssertPhase = "connected components";
		ProcessPendingConnectedComponents();
		g_voxelAssertPhase = "body replacement";
		m_replacements.Update();
		g_voxelAssertPhase = "contact hit events";
		ProcessHitEvents();
		g_voxelAssertPhase = "wrecking ball maintenance";
		UpdateWreckingBallHistory();
		DestroyExpiredWreckingBalls();
		g_voxelAssertPhase = "collider budget cleanup";
		ManageVoxelObjectBudget();
		g_voxelAssertPhase = "offscreen cleanup";
		CleanupOffscreenFragments();
		g_voxelAssertPhase = "voxel statistics";

		int voxelCount = 0;
		int boxCount = 0;
		int chunkCount = 0;
		int meshCount = 0;
		int triangleCount = 0;
		for ( const std::unique_ptr<VoxelBody>& body : m_voxelBodies )
		{
			voxelCount += body->GetFilledCount();
			boxCount += body->m_boxCount;
			chunkCount += body->m_meshCache->GetChunkCount();
			meshCount += body->m_meshCache->GetMeshCount();
			triangleCount += body->m_meshCache->GetTriangleCount();
		}

		DrawTextLine( "wrecking ball: Shift + right click to launch, X to fracture center, R to restart" );
		DrawTextLine( "voxel bodies / visible voxels / collider boxes = %d / %d / %d", (int)m_voxelBodies.size(), voxelCount,
					  boxCount );
		DrawTextLine( "cached chunks / meshes / triangles = %d / %d / %d", chunkCount, meshCount, triangleCount );
		DrawTextLine( "async jobs / pending pieces / updates started / finished = %d / %d / %d / %d",
					  m_jobs.GetPendingJobCount(), m_replacements.GetPendingPieceCount(), m_updateStartedCount,
					  m_updateFinishedCount );
		DrawTextLine( "voxel map updates: active / started / finished = %d / %d / %d",
					  m_bodyUpdates.GetUpdatingBodyCount(), m_inPlaceUpdateStartedCount, m_inPlaceUpdateFinishedCount );
		DrawTextLine( "automatic CCL: started / finished = %d / %d", m_cclStartedCount, m_cclFinishedCount );
		DrawTextLine( "editor operations: started / finished = %d / %d", m_editorOperationStartedCount,
					  m_editorOperationFinishedCount );
		DrawTextLine( "last fracture: cut voxels / created bodies = %d / %d", m_lastCutVoxelCount, m_lastCreatedBodyCount );
		DrawTextLine( "collider budget / removed fragment bodies = %d / %d", m_maxManagedBoxCount, m_budgetRemovedBodyCount );
		DrawTextLine( "offscreen debris: small / protected / eligible = %d / %d / %d",
					  m_offscreenCleanupSmallBodyCount, m_offscreenCleanupProtectedBodyCount,
					  m_offscreenCleanupEligibleBodyCount );
		DrawTextLine( "offscreen debris: outside / waiting / removed = %d / %d / %d",
					  m_offscreenCleanupOutsideBodyCount, m_offscreenCleanupWaitingBodyCount,
					  m_offscreenCleanupRemovedBodyCount );
		DrawTextLine( "cleanup limits: max voxels / grace frames / body deletes per frame = %d / %d / %d",
					  m_offscreenCleanupMaxVoxelCount, m_offscreenCleanupGraceFrames, m_maxBodyDeletesPerFrame );
	}

	void Render() override
	{
		g_voxelAssertPhase = "voxel rendering";
		for ( const std::unique_ptr<VoxelBody>& body : m_voxelBodies )
		{
			body->Draw();
		}
	}

	void Keyboard( int key, int action, int modifiers ) override
	{
		(void)modifiers;
		if ( action != ACTION_PRESS )
		{
			return;
		}

		if ( key == SAPP_KEYCODE_X && m_voxelBodies.empty() == false )
		{
			VoxelBody* target = m_voxelBodies.front().get();
			b3Pos point = b3Body_GetWorldPoint( target->m_bodyId, { 0.0f, 0.5f, 0.5f * target->m_dimensions.z * kVoxelSize } );
			Fracture( target, point, 20.0f * m_launchSpeedScale );
		}
		else if ( key == SAPP_KEYCODE_C )
		{
			CarveCenterVoxelBody();
		}
		else if ( key == SAPP_KEYCODE_H )
		{
			HollowFirstVoxelBody();
		}
		else if ( key == SAPP_KEYCODE_Q )
		{
			ApplyWeaponProfile( ( m_activeWeaponProfile + 1 ) % 3 );
		}
	}

	void MouseDown( b3Vec2 p, int button, int modifiers ) override
	{
		if ( button == MOUSE_RIGHT && modifiers == MOD_SHIFT )
		{
			LaunchWreckingBall( p );
			return;
		}

		Sample::MouseDown( p, button, modifiers );
	}

	bool DrawControls() override
	{
		ImGui::Checkbox( "Immediate voxel initialization", &m_immediateInitialization );
		ImGui::Checkbox( "Enable physics on initialization", &m_initializationPhysics );
		if ( ImGui::Button( "Reset voxel wall" ) )
		{
			ResetDemo();
		}
		if ( m_initializationStatus.empty() == false )
		{
			ImGui::TextWrapped( "%s", m_initializationStatus.c_str() );
		}
		ImGui::SliderInt( "Large scene columns", &m_largeSceneColumns, 1, 4 );
		ImGui::SliderInt( "Large scene rows", &m_largeSceneRows, 1, 3 );
		if ( ImGui::Button( "Build large destructible scene" ) )
		{
			BuildLargeVoxelScene();
		}
		if ( ImGui::Button( "Save voxel_wall.b3vox" ) )
		{
			SaveFirstVoxelBody();
		}
		if ( ImGui::Button( "Load voxel_wall.b3vox" ) )
		{
			LoadVoxelBody();
		}
		ImGui::InputText( "Unity voxel .asset path", m_unityAssetPath, sizeof( m_unityAssetPath ) );
		if ( ImGui::Button( "Load Unity SerializedVoxelMap" ) )
		{
			LoadUnityVoxelBody();
		}
		ImGui::InputText( "Unity export .asset path", m_unityExportPath, sizeof( m_unityExportPath ) );
		if ( ImGui::Button( "Save Unity SerializedVoxelMap" ) )
		{
			SaveFirstVoxelBodyAsUnityAsset();
		}
		if ( ImGui::Button( "Carve center (async update)" ) )
		{
			CarveCenterVoxelBody();
		}
		ImGui::SliderInt( "Hollow wall thickness", &m_hollowWallThickness, 1, 8 );
		if ( ImGui::Button( "Hollow first voxel object" ) )
		{
			HollowFirstVoxelBody();
		}
		if ( m_voxelBodies.empty() == false )
		{
			VoxelBody& firstBody = *m_voxelBodies.front();
			if ( ImGui::Button( firstBody.m_dynamic ? "Disable first body physics" : "Enable first body physics" ) )
			{
				if ( firstBody.m_updating == false )
				{
					firstBody.SetPhysicsEnabled( firstBody.m_dynamic == false );
				}
			}
			if ( firstBody.m_dynamic &&
				 ImGui::Button( firstBody.IsKinematic() ? "Resume first body dynamics" : "Make first body kinematic" ) )
			{
				firstBody.SetKinematic( firstBody.IsKinematic() == false );
			}
		}
		ImGui::InputInt3( "Resize dimensions", m_resizeDimensions );
		if ( ImGui::Button( "Resize first voxel object" ) )
		{
			ResizeFirstVoxelBody();
		}
		if ( m_serializationStatus.empty() == false )
		{
			ImGui::TextWrapped( "%s", m_serializationStatus.c_str() );
		}
		ImGui::InputText( "OBJ path", m_objPath, sizeof( m_objPath ) );
		ImGui::InputFloat( "OBJ scale", &m_objScale, 0.1f, 1.0f, "%.2f" );
		ImGui::InputFloat( "OBJ scan voxel size", &m_objScanVoxelSize, 0.05f, 0.2f, "%.2f" );
		ImGui::Checkbox( "OBJ is Z-up", &m_objZUp );
		if ( ImGui::Button( "Import and voxelize OBJ" ) )
		{
			ImportObjVoxelBody();
		}
		if ( m_objImportStatus.empty() == false )
		{
			ImGui::TextWrapped( "%s", m_objImportStatus.c_str() );
		}

		const char* weaponProfiles[] = { "Box3D sample ball", "Unity small ball", "Unity big ball" };
		int weaponProfile = m_activeWeaponProfile;
		if ( ImGui::Combo( "Weapon profile", &weaponProfile, weaponProfiles, 3 ) )
		{
			ApplyWeaponProfile( weaponProfile );
		}
		ImGui::SliderFloat( "Projectile radius", &m_projectileRadius, 0.1f, 1.0f, "%.2f" );
		ImGui::SliderFloat( "Projectile speed", &m_projectileSpeed, 1.0f, 80.0f, "%.1f" );
		ImGui::SliderFloat( "Projectile mass override", &m_projectileMass, 0.0f, 500.0f, "%.1f" );
		ImGui::SliderInt( "Fracture radius", &m_fractureRadius, 4, 40 );
		ImGui::SliderFloat( "Seed spawn world radius", &m_seedSpawnWorldRadius, 0.0f, 8.0f, "%.2f" );
		ImGui::SliderInt( "Voronoi seeds", &m_seedCount, 4, 64 );
		ImGui::SliderInt( "Minimum fragment", &m_minFragmentVoxels, 1, 48 );
		ImGui::SliderInt( "Ball minimum destruction radius", &m_minDestructionVoxelRadius, 1, 16 );
		ImGui::SliderInt( "Ball minimum target voxels", &m_minVoxelsForWreckingBallFracture, 1, 1000 );
		ImGui::SliderFloat( "Ball velocity after fracture", &m_wreckingBallVelocityAfterFracture, 0.0f, 1.0f, "%.2f" );
		ImGui::SliderFloat( "Ball lifetime", &m_wreckingBallLifetimeSeconds, 1.0f, 30.0f, "%.1f s" );
		ImGui::SliderFloat( "Launch speed scale", &m_launchSpeedScale, 0.5f, 3.0f, "%.1f" );
		ImGui::Checkbox( "Keep grounded component", &m_keepGroundedComponent );
		ImGui::Checkbox( "CCL after voxel update", &m_connectedComponentsOnVoxelUpdate );
		ImGui::Checkbox( "Immediate update CCL", &m_immediateConnectedComponents );
		ImGui::Checkbox( "New component physics", &m_newComponentPhysicsEnabled );
		ImGui::SliderInt( "Minimum CCL component", &m_minConnectedComponentVoxels, 1, 64 );
		ImGui::Checkbox( "Destroy fragment edges", &m_destroyFragmentEdges );
		ImGui::Checkbox( "Destroy fragment outer layer", &m_destroyOuterLayer );
		ImGui::Checkbox( "Immediate fracture computation", &m_immediateFractureComputation );
		const char* replaceModes[] = { "Immediate", "Standard", "Anti-flicker" };
		int replaceMode = static_cast<int>( m_replaceMode );
		if ( ImGui::Combo( "Replace updater", &replaceMode, replaceModes, 3 ) )
		{
			m_replaceMode = static_cast<VoxelReplaceMode>( replaceMode );
		}
		if ( ImGui::SliderInt( "Bodies built per frame", &m_bodiesBuiltPerFrame, 1, 16 ) )
		{
			m_replacements.SetBodiesPerFrame( m_bodiesBuiltPerFrame );
		}
		ImGui::SliderInt( "Collider box budget", &m_maxManagedBoxCount, 256, 10000 );
		ImGui::SliderInt( "Managed fragment max voxels", &m_manageFragmentBelowVoxelCount, 16, 1000 );
		ImGui::Checkbox( "Delete small offscreen debris", &m_cleanupOffscreenFragments );
		ImGui::SliderInt( "Offscreen cleanup max voxels", &m_offscreenCleanupMaxVoxelCount, 1, 5000 );
		ImGui::SliderInt( "Offscreen cleanup grace frames", &m_offscreenCleanupGraceFrames, 15, 600 );
		ImGui::SliderInt( "Max body deletes/frame", &m_maxBodyDeletesPerFrame, 1, 64 );
		return true;
	}

	static Sample* Create( SampleContext* context )
	{
		return new VoxelDestructionDemo( context );
	}

	VoxelJobSystem m_jobs;
	std::vector<std::unique_ptr<VoxelBody>> m_voxelBodies;
	VoxelBodyUpdateManager m_bodyUpdates;
	VoxelReplaceUpdater m_replacements;
	std::vector<WreckingBall> m_wreckingBalls;
	std::mt19937 m_random;
	int m_activeWeaponProfile = 0;
	float m_projectileRadius = 0.25f;
	float m_projectileSpeed = 20.0f;
	float m_projectileMass = 0.0f;
	int m_fractureRadius = 9;
	float m_seedSpawnWorldRadius = 0.0f;
	int m_seedCount = 16;
	int m_minFragmentVoxels = 4;
	int m_minDestructionVoxelRadius = 5;
	int m_minVoxelsForWreckingBallFracture = 200;
	bool m_cleanupOffscreenFragments = true;
	int m_offscreenCleanupMaxVoxelCount = 500;
	int m_offscreenCleanupGraceFrames = 60;
	int m_maxBodyDeletesPerFrame = 8;
	int m_bodyDeletionBudgetRemaining = 8;
	int m_offscreenCleanupSmallBodyCount = 0;
	int m_offscreenCleanupProtectedBodyCount = 0;
	int m_offscreenCleanupEligibleBodyCount = 0;
	int m_offscreenCleanupOutsideBodyCount = 0;
	int m_offscreenCleanupWaitingBodyCount = 0;
	int m_offscreenCleanupRemovedBodyCount = 0;
	float m_wreckingBallVelocityAfterFracture = 0.8f;
	float m_wreckingBallLifetimeSeconds = 5.0f;
	bool m_keepGroundedComponent = true;
	bool m_connectedComponentsOnVoxelUpdate = true;
	bool m_immediateConnectedComponents = false;
	bool m_newComponentPhysicsEnabled = true;
	int m_minConnectedComponentVoxels = 10;
	bool m_destroyFragmentEdges = true;
	bool m_destroyOuterLayer = false;
	bool m_immediateFractureComputation = false;
	VoxelReplaceMode m_replaceMode = VoxelReplaceMode::antiFlicker;
	int m_bodiesBuiltPerFrame = 2;
	int m_hollowWallThickness = 1;
	int m_resizeDimensions[3] = { 56, 42, 7 };
	int m_largeSceneColumns = 3;
	int m_largeSceneRows = 2;
	int m_lastCutVoxelCount = 0;
	int m_lastCreatedBodyCount = 0;
	int m_impactCooldown = 0;
	int m_updateStartedCount = 0;
	int m_updateFinishedCount = 0;
	int m_maxManagedBoxCount = 3500;
	int m_colliderBudgetHysteresis = 128;
	int m_manageFragmentBelowVoxelCount = 300;
	int m_budgetRemovedBodyCount = 0;
	bool m_voxelBudgetCleanupPending = false;
	bool m_autoTest = false;
	bool m_autoTestTriggered = false;
	bool m_budgetAutoTest = false;
	bool m_updateAutoTest = false;
	bool m_updateAutoTestTriggered = false;
	bool m_ballAutoTest = false;
	bool m_ballAutoTestTriggered = false;
	bool m_ballStressTest = false;
	int m_ballStressLaunchedCount = 0;
	bool m_resizeAutoTest = false;
	bool m_resizeAutoTestTriggered = false;
	bool m_objAutoTest = false;
	bool m_objAutoTestTriggered = false;
	bool m_stressTest = false;
	int m_stressTriggeredCount = 0;
	bool m_cclUpdateAutoTest = false;
	bool m_cclUpdateAutoTestTriggered = false;
	bool m_largeSceneAutoTest = false;
	bool m_largeSceneAutoTestTriggered = false;
	bool m_unityAssetAutoTest = false;
	bool m_unityAssetAutoTestTriggered = false;
	bool m_physicsToggleAutoTest = false;
	bool m_physicsToggleAutoTestFinished = false;
	bool m_initializerAutoTest = false;
	bool m_initializerAutoTestTriggered = false;
	bool m_initializerAutoTestFinished = false;
	bool m_weaponProfileAutoTest = false;
	bool m_weaponProfileAutoTestFinished = false;
	bool m_offscreenCleanupAutoTest = false;
	bool m_offscreenCleanupAutoTestFinished = false;
	uint64_t m_offscreenCleanupAutoTestBodyId = 0;
	bool m_immediateInitialization = false;
	bool m_initializationPhysics = false;
	bool m_initializationInProgress = false;
	int m_inPlaceUpdateStartedCount = 0;
	int m_inPlaceUpdateFinishedCount = 0;
	std::vector<uint64_t> m_pendingConnectedComponentBodies;
	int m_cclStartedCount = 0;
	int m_cclFinishedCount = 0;
	int m_editorOperationStartedCount = 0;
	int m_editorOperationFinishedCount = 0;
	std::string m_serializationStatus;
	std::string m_initializationStatus;
	char m_unityAssetPath[512] = {};
	char m_unityExportPath[512] = "voxel_wall_unity.asset";
	char m_objPath[512] = {};
	float m_objScale = 1.0f;
	float m_objScanVoxelSize = 0.2f;
	bool m_objZUp = true;
	bool m_objImportInProgress = false;
	std::string m_objImportStatus;
	uint64_t m_sceneGeneration = 0;
};

static int sampleVoxelDestruction = RegisterSample( "Voxel", "Destructible Wall", VoxelDestructionDemo::Create );

} // namespace
