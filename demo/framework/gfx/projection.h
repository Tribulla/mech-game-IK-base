#pragma once

#include "gfx/utility.h"

#include <stdbool.h>

static inline bool ProjectWorldToScreen( Mat4 view, Mat4 proj, int viewportW, int viewportH, Vec3 worldPos, float* outScreenX,
										 float* outScreenY )
{
	Vec4 world = MakeVec4( worldPos.x, worldPos.y, worldPos.z, 1.0f );
	Vec4 viewP = MulMV4( view, world );
	Vec4 clip = MulMV4( proj, viewP );

	if ( clip.w <= 1e-6f )
	{
		return false;
	}

	const float invW = 1.0f / clip.w;
	const float ndcX = clip.x * invW;
	const float ndcY = clip.y * invW;

	if ( ndcX < -1.0f || ndcX > 1.0f )
	{
		return false;
	}

	if ( ndcY < -1.0f || ndcY > 1.0f )
	{
		return false;
	}

	*outScreenX = ( ndcX * 0.5f + 0.5f ) * (float)viewportW;
	*outScreenY = ( 1.0f - ( ndcY * 0.5f + 0.5f ) ) * (float)viewportH;

	return true;
}
