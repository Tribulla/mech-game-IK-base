#pragma once

#include "gfx/utility.h"
#include "sokol_gfx.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct HighlightParams
{
	bool enabled;
	float hoverThicknessPx;
	float selectThicknessPx;

	Vec4 hoverColor;
	Vec4 selectColor;
} HighlightParams;

HighlightParams GetDefaultHighlightParams( void );

void InitHighlightOutline( void );
void ShutdownHighlightOutline( void );

void SubmitHighlightOutline( sg_view maskSampleView, sg_pixel_format colorFormat, sg_pixel_format depthFormat,
							 const HighlightParams* params );

#ifdef __cplusplus
} 
#endif
