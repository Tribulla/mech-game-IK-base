#pragma once

#include "gfx/draw_overlay.h"
#include "gfx/utility.h"
#include "sokol_gfx.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define MAX_GEOM_XP_INSTANCES_GLOBAL ( 16 * 1024 )

typedef struct MeshHandle
{
	int index;
	uint32_t hash;
} MeshHandle;

static inline MeshHandle InvalidMeshHandle( void )
{
	MeshHandle h = { -1, 0u };
	return h;
}

static inline bool IsMeshHandleValid( MeshHandle h )
{
	return h.index >= 0;
}

typedef struct MeshVertex
{
	float position[3];
	float normal[3];
} MeshVertex;

typedef struct EdgeVertex
{
	float position[3];
	float flag;
} EdgeVertex;

typedef struct MeshDrawSpan
{
	sg_buffer vbo;
	sg_buffer ibo;
	int indexCount;
	int firstInstance;
	int instanceCount;
} MeshDrawSpan;

typedef enum MeshKind
{
	MESH_KIND_UNKNOWN = 0,
	MESH_KIND_HULL,
	MESH_KIND_MESH,
	MESH_KIND_HEIGHTFIELD,
} MeshKind;

typedef struct MeshEdgeBatch
{
	MeshKind kind;
	sg_view edgeStorageView;
	int edgeCount;
	int firstInstance;
	int instanceCount;
	bool isTransparent;
} MeshEdgeBatch;

typedef struct MeshXpInstance
{
	sg_buffer vbo;
	sg_buffer ibo;
	int indexCount;
	int firstInstance;
	Vec3 origin;
	float shadowCast;
} MeshXpInstance;

void CreateMeshRegistry( void );
void DestroyMeshRegistry( void );

MeshHandle FindMesh( uint32_t hash );

MeshHandle RegisterMesh( uint32_t hash, const MeshVertex* vertices, int vertexCount, const uint32_t* indices,
								int indexCount, const char* debugLabel );

void RegisterMeshEdges( MeshHandle h, const EdgeVertex* edges, int edgeCount, const char* debugLabel );

bool HasEdges( MeshHandle h );
int GetEdgeCount( MeshHandle h );
sg_view GetEdgeStorageView( MeshHandle h );

void SetMeshKind( MeshHandle h, MeshKind kind );
MeshKind GetMeshKind( MeshHandle h );

typedef struct MeshBuffers
{
	sg_buffer vbo;
	sg_buffer ibo;
	int indexCount;
} MeshBuffers;

bool GetMeshBuffers( MeshHandle h, MeshBuffers* out );

void AddMeshReference( MeshHandle h );
void ReleaseMeshReference( MeshHandle h );

void ResetFrameForMeshes( void );

typedef enum MeshMaterialMode
{
	MESH_MATERIAL_MODE_SOLID = 0,
	MESH_MATERIAL_MODE_GROUND_GRID = 1,
} MeshMaterialMode;

void AppendMesh( MeshHandle h, Transform transform, Vec3 scale, Vec4 baseColor, float metallic, float roughness,
					MeshMaterialMode materialMode, float gridCellSize, TransparentShadowCast shadowCast );

int UploadMeshInstances( void );

int GetMeshDrawSpanCount( void );

MeshDrawSpan GetMeshDrawSpan( int i );

int GetMeshMirrorDrawSpanCount( void );

MeshDrawSpan GetMeshMirrorDrawSpan( int i );

sg_view GetMeshInstanceView( void );

int GetTransparentMeshInstanceCount( void );
MeshXpInstance GetTransparentMeshInstance( int i );
sg_view GetTransparentMeshInstanceView( void );

int GetMeshEdgeBatchCount( void );
MeshEdgeBatch GetMeshEdgeBatch( int i );

int GetMeshEntryCount( void );

#ifdef __cplusplus
} 
#endif
