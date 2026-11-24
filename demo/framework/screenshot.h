#pragma once

#include "sokol_gfx.h"

void RequestScreenshot( const char* path );
bool IsScreenshotRequested();

bool CaptureScreenshotIfRequested( const sg_swapchain* swapchain, const char* sampleName );

bool WritePng( const char* path, const unsigned char* rgba, int width, int height );
