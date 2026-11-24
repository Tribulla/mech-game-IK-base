#include "gfx/geometry_registry.h"

#include "geom.glsl.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_REGISTRY_ENTRIES 4096
#define MAX_GEOM_INSTANCES_GLOBAL 65536
#define INITIAL_PER_ENTRY_CAPACITY 8

typedef struct Entry
{
	uint32_t hash;
	int refCount;
	sg_buffer vbo;
	sg_buffer ibo;
	int indexCount;
	MeshKind kind;

	sg_buffer edgeBuf;
	sg_view edgeStorageView;
	int edgeCount;

	int xpFirstInstance;

	geom_instance_t* instances;
	int count;
	int capacity;

	int firstInstance;

	geom_instance_t* xpInstances;
	int xpCount;
	int xpCapacity;
} Entry;

static struct
{
	Entry entries[MAX_REGISTRY_ENTRIES];
	int entryCount;

	sg_buffer instBuf;
	sg_view instView;

	geom_instance_t* uploadScratch;
	int uploadScratchCap;

	MeshDrawSpan drawSpans[MAX_REGISTRY_ENTRIES];
	int drawSpanCount;
	MeshDrawSpan mirrorDrawSpans[MAX_REGISTRY_ENTRIES];
	int mirrorDrawSpanCount;

	sg_buffer xpInstBuf;
	sg_view xpInstView;
	geom_instance_t* xpUploadScratch;
	int xpUploadScratchCap;
	MeshXpInstance xpDraws[MAX_GEOM_XP_INSTANCES_GLOBAL];
	int xpDrawCount;

	MeshEdgeBatch edgeBatches[2 * MAX_REGISTRY_ENTRIES];
	int edgeBatchCount;
} s_geom;

void CreateMeshRegistry( void )
{
	sg_buffer_desc bdesc = { 0 };
	bdesc.usage.storage_buffer = true;
	bdesc.usage.stream_update = true;
	bdesc.size = (size_t)MAX_GEOM_INSTANCES_GLOBAL * sizeof( geom_instance_t );
	bdesc.label = "geom_instances";
	s_geom.instBuf = sg_make_buffer( &bdesc );

	sg_view_desc vdesc = { 0 };
	vdesc.storage_buffer.buffer = s_geom.instBuf;
	vdesc.label = "geom_instances_view";
	s_geom.instView = sg_make_view( &vdesc );

	sg_buffer_desc xpbdesc = { 0 };
	xpbdesc.usage.storage_buffer = true;
	xpbdesc.usage.stream_update = true;
	xpbdesc.size = (size_t)MAX_GEOM_XP_INSTANCES_GLOBAL * sizeof( geom_instance_t );
	xpbdesc.label = "geom_instances_xp";
	s_geom.xpInstBuf = sg_make_buffer( &xpbdesc );

	sg_view_desc xpvdesc = { 0 };
	xpvdesc.storage_buffer.buffer = s_geom.xpInstBuf;
	xpvdesc.label = "geom_instances_xp_view";
	s_geom.xpInstView = sg_make_view( &xpvdesc );

	s_geom.uploadScratch = NULL;
	s_geom.uploadScratchCap = 0;
	s_geom.xpUploadScratch = NULL;
	s_geom.xpUploadScratchCap = 0;
	s_geom.entryCount = 0;
	s_geom.drawSpanCount = 0;
	s_geom.mirrorDrawSpanCount = 0;
	s_geom.xpDrawCount = 0;
	s_geom.edgeBatchCount = 0;
}

void DestroyMeshRegistry( void )
{
	for ( int i = 0; i < s_geom.entryCount; ++i )
	{
		Entry* e = &s_geom.entries[i];
		if ( e->refCount > 0 )
		{
			sg_destroy_buffer( e->ibo );
			sg_destroy_buffer( e->vbo );
			if ( e->edgeCount > 0 )
			{
				sg_destroy_view( e->edgeStorageView );
				sg_destroy_buffer( e->edgeBuf );
			}
		}
		free( e->instances );
		e->instances = NULL;
		e->capacity = 0;
		free( e->xpInstances );
		e->xpInstances = NULL;
		e->xpCapacity = 0;
	}
	s_geom.entryCount = 0;

	free( s_geom.uploadScratch );
	s_geom.uploadScratch = NULL;
	s_geom.uploadScratchCap = 0;
	free( s_geom.xpUploadScratch );
	s_geom.xpUploadScratch = NULL;
	s_geom.xpUploadScratchCap = 0;

	sg_destroy_view( s_geom.xpInstView );
	sg_destroy_buffer( s_geom.xpInstBuf );
	sg_destroy_view( s_geom.instView );
	sg_destroy_buffer( s_geom.instBuf );
}

MeshHandle FindMesh( uint32_t hash )
{
	if ( hash == 0u )
	{
		return InvalidMeshHandle();
	}

	for ( int i = 0; i < s_geom.entryCount; ++i )
	{
		const Entry* e = &s_geom.entries[i];
		if ( e->refCount > 0 && e->hash == hash )
		{
			MeshHandle h = { i, hash };
			return h;
		}
	}

	return InvalidMeshHandle();
}

static int FindFreeSlot( void )
{
	for ( int i = 0; i < s_geom.entryCount; ++i )
	{
		if ( s_geom.entries[i].refCount == 0 )
		{
			return i;
		}
	}
	if ( s_geom.entryCount >= MAX_REGISTRY_ENTRIES )
	{
		return -1;
	}
	return s_geom.entryCount++;
}

MeshHandle RegisterMesh( uint32_t hash, const MeshVertex* vertices, int vertexCount, const uint32_t* indices,
								int indexCount, const char* debugLabel )
{
	assert( hash != 0u );
	assert( vertexCount > 0 );
	assert( indexCount > 0 );

	const int slot = FindFreeSlot();
	if ( slot < 0 )
	{
		fprintf( stderr, "error: registry full (cap=%d), dropping geometry\n", MAX_REGISTRY_ENTRIES );
		return InvalidMeshHandle();
	}

	Entry* e = &s_geom.entries[slot];

	sg_buffer_desc vbDesc = { 0 };
	vbDesc.usage.vertex_buffer = true;
	vbDesc.usage.immutable = true;
	vbDesc.data.ptr = vertices;
	vbDesc.data.size = (size_t)vertexCount * sizeof( MeshVertex );
	vbDesc.label = debugLabel ? debugLabel : "geom_vbo";
	sg_buffer vbo = sg_make_buffer( &vbDesc );

	sg_buffer_desc ibDesc = { 0 };
	ibDesc.usage.index_buffer = true;
	ibDesc.usage.immutable = true;
	ibDesc.data.ptr = indices;
	ibDesc.data.size = (size_t)indexCount * sizeof( uint32_t );
	ibDesc.label = debugLabel ? debugLabel : "geom_ibo";
	sg_buffer ibo = sg_make_buffer( &ibDesc );

	if ( sg_query_buffer_state( vbo ) != SG_RESOURCESTATE_VALID || sg_query_buffer_state( ibo ) != SG_RESOURCESTATE_VALID )
	{
		sg_destroy_buffer( ibo );
		sg_destroy_buffer( vbo );
		return InvalidMeshHandle();
	}

	e->vbo = vbo;
	e->ibo = ibo;
	e->hash = hash;
	e->refCount = 1;
	e->indexCount = indexCount;
	e->kind = MESH_KIND_UNKNOWN;
	e->edgeBuf.id = SG_INVALID_ID;
	e->edgeStorageView.id = SG_INVALID_ID;
	e->edgeCount = 0;
	e->xpFirstInstance = 0;
	e->count = 0;
	e->xpCount = 0;

	MeshHandle h = { slot, hash };
	return h;
}

void RegisterMeshEdges( MeshHandle h, const EdgeVertex* edges, int edgeCount, const char* debugLabel )
{
	if ( !IsMeshHandleValid( h ) )
	{
		return;
	}
	if ( edges == NULL || edgeCount <= 0 )
	{
		return;
	}
	Entry* e = &s_geom.entries[h.index];
	if ( e->refCount == 0 || e->hash != h.hash )
	{
		return;
	}
	assert( e->edgeCount == 0 && "RegisterMeshEdges already called for this entry" );

	sg_buffer_desc bdesc = { 0 };
	bdesc.usage.storage_buffer = true;
	bdesc.usage.immutable = true;
	bdesc.data.ptr = edges;
	bdesc.data.size = (size_t)( 2 * edgeCount ) * sizeof( EdgeVertex );
	bdesc.label = debugLabel ? debugLabel : "geom_edges";
	sg_buffer edgeBuf = sg_make_buffer( &bdesc );

	if ( sg_query_buffer_state( edgeBuf ) != SG_RESOURCESTATE_VALID )
	{
		sg_destroy_buffer( edgeBuf );
		return;
	}

	sg_view_desc vdesc = { 0 };
	vdesc.storage_buffer.buffer = edgeBuf;
	vdesc.label = debugLabel ? debugLabel : "geom_edges_view";
	sg_view edgeView = sg_make_view( &vdesc );
	if ( sg_query_view_state( edgeView ) != SG_RESOURCESTATE_VALID )
	{
		sg_destroy_view( edgeView );
		sg_destroy_buffer( edgeBuf );
		return;
	}

	e->edgeBuf = edgeBuf;
	e->edgeStorageView = edgeView;
	e->edgeCount = edgeCount;
}

bool HasEdges( MeshHandle h )
{
	if ( !IsMeshHandleValid( h ) )
	{
		return false;
	}
	const Entry* e = &s_geom.entries[h.index];
	if ( e->refCount == 0 || e->hash != h.hash )
	{
		return false;
	}
	return e->edgeCount > 0;
}

int GetEdgeCount( MeshHandle h )
{
	if ( !IsMeshHandleValid( h ) )
	{
		return 0;
	}
	const Entry* e = &s_geom.entries[h.index];
	if ( e->refCount == 0 || e->hash != h.hash )
	{
		return 0;
	}
	return e->edgeCount;
}

sg_view GetEdgeStorageView( MeshHandle h )
{
	sg_view invalid = { SG_INVALID_ID };
	if ( !IsMeshHandleValid( h ) )
	{
		return invalid;
	}
	const Entry* e = &s_geom.entries[h.index];
	if ( e->refCount == 0 || e->hash != h.hash || e->edgeCount == 0 )
	{
		return invalid;
	}
	return e->edgeStorageView;
}

void SetMeshKind( MeshHandle h, MeshKind kind )
{
	if ( !IsMeshHandleValid( h ) )
	{
		return;
	}
	Entry* e = &s_geom.entries[h.index];
	if ( e->refCount == 0 || e->hash != h.hash )
	{
		return;
	}
	e->kind = kind;
}

MeshKind GetMeshKind( MeshHandle h )
{
	if ( !IsMeshHandleValid( h ) )
	{
		return MESH_KIND_UNKNOWN;
	}
	const Entry* e = &s_geom.entries[h.index];
	if ( e->refCount == 0 || e->hash != h.hash )
	{
		return MESH_KIND_UNKNOWN;
	}
	return e->kind;
}

bool GetMeshBuffers( MeshHandle h, MeshBuffers* out )
{
	if ( !IsMeshHandleValid( h ) || !out )
	{
		return false;
	}
	const Entry* e = &s_geom.entries[h.index];
	if ( e->refCount == 0 || e->hash != h.hash )
	{
		return false;
	}
	out->vbo = e->vbo;
	out->ibo = e->ibo;
	out->indexCount = e->indexCount;
	return true;
}

void AddMeshReference( MeshHandle h )
{
	if ( !IsMeshHandleValid( h ) )
	{
		return;
	}
	Entry* e = &s_geom.entries[h.index];
	assert( e->refCount > 0 && e->hash == h.hash );
	e->refCount += 1;
}

void ReleaseMeshReference( MeshHandle h )
{
	if ( !IsMeshHandleValid( h ) )
	{
		return;
	}
	Entry* e = &s_geom.entries[h.index];
	assert( e->refCount > 0 && e->hash == h.hash );
	e->refCount -= 1;
	if ( e->refCount == 0 )
	{
		sg_destroy_buffer( e->ibo );
		sg_destroy_buffer( e->vbo );
		if ( e->edgeCount > 0 )
		{
			sg_destroy_view( e->edgeStorageView );
			sg_destroy_buffer( e->edgeBuf );
		}
		e->hash = 0u;
		e->indexCount = 0;
		e->edgeBuf.id = SG_INVALID_ID;
		e->edgeStorageView.id = SG_INVALID_ID;
		e->edgeCount = 0;
		e->count = 0;
		e->xpCount = 0;
	}
}

void ResetFrameForMeshes( void )
{
	for ( int i = 0; i < s_geom.entryCount; ++i )
	{
		s_geom.entries[i].count = 0;
		s_geom.entries[i].xpCount = 0;
	}
	s_geom.drawSpanCount = 0;
	s_geom.mirrorDrawSpanCount = 0;
	s_geom.xpDrawCount = 0;
	s_geom.edgeBatchCount = 0;
}

static void PackMat4ToInstance( const Mat4* transform, Vec3 scale, Vec4 baseColor, float metallic, float roughness,
								MeshMaterialMode materialMode, float gridCellSize, TransparentShadowCast shadowCast,
								geom_instance_t* inst )
{
	inst->xform_row0.x = transform->cx.x;
	inst->xform_row0.y = transform->cy.x;
	inst->xform_row0.z = transform->cz.x;
	inst->xform_row0.w = transform->cw.x;
	inst->xform_row1.x = transform->cx.y;
	inst->xform_row1.y = transform->cy.y;
	inst->xform_row1.z = transform->cz.y;
	inst->xform_row1.w = transform->cw.y;
	inst->xform_row2.x = transform->cx.z;
	inst->xform_row2.y = transform->cy.z;
	inst->xform_row2.z = transform->cz.z;
	inst->xform_row2.w = transform->cw.z;

	inst->base_color = baseColor;

	inst->scale.x = scale.x;
	inst->scale.y = scale.y;
	inst->scale.z = scale.z;
	inst->scale.w = 0.0f;

	const bool isGround = ( materialMode == MESH_MATERIAL_MODE_GROUND_GRID );
	inst->material.x = metallic;
	inst->material.y = roughness;
	inst->material.z = (float)materialMode;
	inst->material.w = isGround ? gridCellSize : ( ( shadowCast == TRANSPARENT_SHADOW_FULL ) ? 1.0f : 0.0f );
}

static Vec3 GetOriginFromInstance( const geom_instance_t* inst )
{
	return (Vec3){ inst->xform_row0.w, inst->xform_row1.w, inst->xform_row2.w };
}

void AppendMesh( MeshHandle h, Transform transform, Vec3 scale, Vec4 baseColor, float metallic, float roughness,
					MeshMaterialMode materialMode, float gridCellSize, TransparentShadowCast shadowCast )
{
	if ( !IsMeshHandleValid( h ) )
	{
		return;
	}
	Entry* e = &s_geom.entries[h.index];
	if ( e->refCount == 0 || e->hash != h.hash )
	{
		return;
	}

	const bool xp = baseColor.w < 1.0f;
	geom_instance_t** arr = xp ? &e->xpInstances : &e->instances;
	int* cnt = xp ? &e->xpCount : &e->count;
	int* cap = xp ? &e->xpCapacity : &e->capacity;

	if ( *cnt >= *cap )
	{
		const int newCap = *cap == 0 ? INITIAL_PER_ENTRY_CAPACITY : *cap * 2;
		geom_instance_t* grown = (geom_instance_t*)realloc( *arr, (size_t)newCap * sizeof( geom_instance_t ) );
		if ( !grown )
		{
			fprintf( stderr, "error: per-entry%s grow failed (newCap=%d)\n", xp ? "(xp)" : "", newCap );
			return;
		}
		*arr = grown;
		*cap = newCap;
	}
	Mat4 m = MakeMat4FromTransform( transform, Vec3_one );
	PackMat4ToInstance( &m, scale, baseColor, metallic, roughness, materialMode, gridCellSize, shadowCast,
						&( *arr )[( *cnt )++] );
}

int UploadMeshInstances( void )
{

	int total = 0;
	for ( int i = 0; i < s_geom.entryCount; ++i )
	{
		total += s_geom.entries[i].count;
	}
	s_geom.drawSpanCount = 0;
	if ( total > MAX_GEOM_INSTANCES_GLOBAL )
	{
		fprintf( stderr, "error: %d opaque instances exceeds global cap %d\n", total, MAX_GEOM_INSTANCES_GLOBAL );
		total = MAX_GEOM_INSTANCES_GLOBAL;
	}
	int opaqueCursor = 0;
	if ( total > 0 )
	{
		if ( total > s_geom.uploadScratchCap )
		{
			geom_instance_t* grown = (geom_instance_t*)realloc( s_geom.uploadScratch, (size_t)total * sizeof( geom_instance_t ) );
			if ( !grown )
			{
				fprintf( stderr, "error: upload scratch grow failed\n" );
				return 0;
			}
			s_geom.uploadScratch = grown;
			s_geom.uploadScratchCap = total;
		}

		for ( int i = 0; i < s_geom.entryCount && opaqueCursor < total; ++i )
		{
			Entry* e = &s_geom.entries[i];
			if ( e->refCount == 0 || e->count == 0 )
			{
				continue;
			}
			int take = e->count;
			if ( opaqueCursor + take > total )
			{
				take = total - opaqueCursor;
			}

			geom_instance_t* dst = &s_geom.uploadScratch[opaqueCursor];
			int posCount = 0;
			for ( int k = 0; k < take; ++k )
			{
				const geom_instance_t* inst = &e->instances[k];
				if ( inst->scale.x * inst->scale.y * inst->scale.z >= 0.0f )
				{
					dst[posCount++] = *inst;
				}
			}
			int negCount = 0;
			for ( int k = 0; k < take; ++k )
			{
				const geom_instance_t* inst = &e->instances[k];
				if ( inst->scale.x * inst->scale.y * inst->scale.z < 0.0f )
				{
					dst[posCount + negCount++] = *inst;
				}
			}
			e->firstInstance = opaqueCursor;

			if ( posCount > 0 )
			{
				MeshDrawSpan span;
				span.vbo = e->vbo;
				span.ibo = e->ibo;
				span.indexCount = e->indexCount;
				span.firstInstance = opaqueCursor;
				span.instanceCount = posCount;
				s_geom.drawSpans[s_geom.drawSpanCount++] = span;
			}
			if ( negCount > 0 )
			{
				MeshDrawSpan span;
				span.vbo = e->vbo;
				span.ibo = e->ibo;
				span.indexCount = e->indexCount;
				span.firstInstance = opaqueCursor + posCount;
				span.instanceCount = negCount;
				s_geom.mirrorDrawSpans[s_geom.mirrorDrawSpanCount++] = span;
			}

			if ( e->edgeCount > 0 )
			{
				MeshEdgeBatch eb;
				eb.kind = e->kind;
				eb.edgeStorageView = e->edgeStorageView;
				eb.edgeCount = e->edgeCount;
				eb.firstInstance = opaqueCursor;
				eb.instanceCount = take;
				eb.isTransparent = false;
				s_geom.edgeBatches[s_geom.edgeBatchCount++] = eb;
			}

			opaqueCursor += take;
		}

		sg_range r;
		r.ptr = s_geom.uploadScratch;
		r.size = (size_t)opaqueCursor * sizeof( geom_instance_t );
		sg_update_buffer( s_geom.instBuf, &r );
	}

	int xpTotal = 0;
	for ( int i = 0; i < s_geom.entryCount; ++i )
	{
		xpTotal += s_geom.entries[i].xpCount;
	}
	s_geom.xpDrawCount = 0;
	if ( xpTotal > MAX_GEOM_XP_INSTANCES_GLOBAL )
	{
		fprintf( stderr, "error: %d transparent instances exceeds cap %d\n", xpTotal, MAX_GEOM_XP_INSTANCES_GLOBAL );
		xpTotal = MAX_GEOM_XP_INSTANCES_GLOBAL;
	}
	int xpCursor = 0;
	if ( xpTotal > 0 )
	{
		if ( xpTotal > s_geom.xpUploadScratchCap )
		{
			geom_instance_t* grown =
				(geom_instance_t*)realloc( s_geom.xpUploadScratch, (size_t)xpTotal * sizeof( geom_instance_t ) );
			if ( !grown )
			{
				fprintf( stderr, "error: xp upload scratch grow failed\n" );
				return opaqueCursor;
			}
			s_geom.xpUploadScratch = grown;
			s_geom.xpUploadScratchCap = xpTotal;
		}

		for ( int i = 0; i < s_geom.entryCount && xpCursor < xpTotal; ++i )
		{
			Entry* e = &s_geom.entries[i];
			if ( e->refCount == 0 || e->xpCount == 0 )
			{
				continue;
			}
			int take = e->xpCount;
			if ( xpCursor + take > xpTotal )
			{
				take = xpTotal - xpCursor;
			}
			memcpy( &s_geom.xpUploadScratch[xpCursor], e->xpInstances, (size_t)take * sizeof( geom_instance_t ) );
			e->xpFirstInstance = xpCursor;
			for ( int k = 0; k < take && s_geom.xpDrawCount < MAX_GEOM_XP_INSTANCES_GLOBAL; ++k )
			{
				const geom_instance_t* src = &e->xpInstances[k];
				MeshXpInstance d;
				d.vbo = e->vbo;
				d.ibo = e->ibo;
				d.indexCount = e->indexCount;
				d.firstInstance = xpCursor + k;
				d.origin = GetOriginFromInstance( src );
				d.shadowCast = src->material.w;
				s_geom.xpDraws[s_geom.xpDrawCount++] = d;
			}

			if ( e->edgeCount > 0 )
			{
				MeshEdgeBatch eb;
				eb.kind = e->kind;
				eb.edgeStorageView = e->edgeStorageView;
				eb.edgeCount = e->edgeCount;
				eb.firstInstance = xpCursor;
				eb.instanceCount = take;
				eb.isTransparent = true;
				s_geom.edgeBatches[s_geom.edgeBatchCount++] = eb;
			}

			xpCursor += take;
		}

		sg_range xr;
		xr.ptr = s_geom.xpUploadScratch;
		xr.size = (size_t)xpCursor * sizeof( geom_instance_t );
		sg_update_buffer( s_geom.xpInstBuf, &xr );
	}

	return opaqueCursor + xpCursor;
}

int GetMeshDrawSpanCount( void )
{
	return s_geom.drawSpanCount;
}

MeshDrawSpan GetMeshDrawSpan( int i )
{
	assert( i >= 0 && i < s_geom.drawSpanCount );
	return s_geom.drawSpans[i];
}

int GetMeshMirrorDrawSpanCount( void )
{
	return s_geom.mirrorDrawSpanCount;
}

MeshDrawSpan GetMeshMirrorDrawSpan( int i )
{
	assert( i >= 0 && i < s_geom.mirrorDrawSpanCount );
	return s_geom.mirrorDrawSpans[i];
}

sg_view GetMeshInstanceView( void )
{
	return s_geom.instView;
}

int GetTransparentMeshInstanceCount( void )
{
	return s_geom.xpDrawCount;
}

MeshXpInstance GetTransparentMeshInstance( int i )
{
	assert( i >= 0 && i < s_geom.xpDrawCount );
	return s_geom.xpDraws[i];
}

sg_view GetTransparentMeshInstanceView( void )
{
	return s_geom.xpInstView;
}

int GetMeshEntryCount( void )
{
	return s_geom.entryCount;
}

int GetMeshEdgeBatchCount( void )
{
	return s_geom.edgeBatchCount;
}

MeshEdgeBatch GetMeshEdgeBatch( int i )
{
	assert( i >= 0 && i < s_geom.edgeBatchCount );
	return s_geom.edgeBatches[i];
}
