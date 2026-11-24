#pragma once

#include "sokol_app.h"
#include "sokol_gfx.h"

typedef enum GuiPanelAnchor
{
	GUI_ANCHOR_LEFT = 0,
	GUI_ANCHOR_RIGHT = 1,
} GuiPanelAnchor;

typedef void DrawUiFcn( void );

void InitUI( const sg_environment* env, DrawUiFcn* drawGuiFcn );
void ShutdownUI( void );

bool HandleEvent( const sapp_event* e );

void StartUIFrame( float dtSec );
void RenderUI( const sg_swapchain* swapchain );

bool BeginPanel( const char* name, GuiPanelAnchor anchor, float width );
void EndPanel( void );
