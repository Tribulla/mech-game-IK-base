#pragma once

#include "sokol_gfx.h"

#ifdef __cplusplus
extern "C"
{
#endif

void InitToneMap( void );
void ShutdownToneMap( void );

void SubmitToneMap( sg_view sceneColorView, sg_pixel_format outputColorFormat, sg_pixel_format outputDepthFormat, float ev,
					float saturation );

#ifdef __cplusplus
} 
#endif
