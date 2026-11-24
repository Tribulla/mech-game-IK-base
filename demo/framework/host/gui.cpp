#include "gui.h"

#include "gfx/draw.h"
#include "gfx/projection.h"
#include "gfx/renderer.h"
#include "gfx/text.h"
#include "imgui.h"
#include "implot.h"
#include "sokol_app.h"
#include "sokol_imgui.h"
#include "sokol_log.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>

static bool s_uiInitialized = false;

static float s_uiScale = 1.0f;

static DrawUiFcn* s_uiCallback;

static void guiApplyStyle( void )
{
	ImGuiStyle& style = ImGui::GetStyle();

	style.WindowPadding = ImVec2( 10.0f, 10.0f );
	style.FramePadding = ImVec2( 8.0f, 4.0f );
	style.CellPadding = ImVec2( 6.0f, 4.0f );
	style.ItemSpacing = ImVec2( 8.0f, 7.0f );
	style.ItemInnerSpacing = ImVec2( 7.0f, 4.0f );
	style.IndentSpacing = 18.0f;
	style.ScrollbarSize = 12.0f;
	style.GrabMinSize = 10.0f;

	style.WindowBorderSize = 1.0f;
	style.FrameBorderSize = 0.0f;
	style.PopupBorderSize = 1.0f;
	style.TabBorderSize = 0.0f;
	style.SeparatorTextBorderSize = 1.0f;

	style.WindowRounding = 4.0f;
	style.ChildRounding = 4.0f;
	style.PopupRounding = 4.0f;
	style.FrameRounding = 3.0f;
	style.GrabRounding = 3.0f;
	style.ScrollbarRounding = 3.0f;
	style.TabRounding = 3.0f;

	style.WindowTitleAlign = ImVec2( 0.0f, 0.5f );

	const ImVec4 accent = ImVec4( 0.28f, 0.48f, 0.66f, 1.00f );
	const ImVec4 accentHi = ImVec4( 0.38f, 0.60f, 0.80f, 1.00f );
	const ImVec4 accentLo = ImVec4( 0.22f, 0.36f, 0.50f, 1.00f );

	ImVec4* c = style.Colors;
	c[ImGuiCol_Text] = ImVec4( 0.90f, 0.91f, 0.93f, 1.00f );
	c[ImGuiCol_TextDisabled] = ImVec4( 0.49f, 0.51f, 0.55f, 1.00f );
	c[ImGuiCol_WindowBg] = ImVec4( 0.110f, 0.115f, 0.125f, 0.97f );
	c[ImGuiCol_ChildBg] = ImVec4( 0.00f, 0.00f, 0.00f, 0.00f );
	c[ImGuiCol_PopupBg] = ImVec4( 0.100f, 0.105f, 0.115f, 0.98f );
	c[ImGuiCol_Border] = ImVec4( 0.00f, 0.00f, 0.00f, 0.45f );
	c[ImGuiCol_BorderShadow] = ImVec4( 0.00f, 0.00f, 0.00f, 0.00f );
	c[ImGuiCol_FrameBg] = ImVec4( 0.18f, 0.19f, 0.21f, 1.00f );
	c[ImGuiCol_FrameBgHovered] = ImVec4( 0.24f, 0.26f, 0.29f, 1.00f );
	c[ImGuiCol_FrameBgActive] = ImVec4( 0.29f, 0.32f, 0.36f, 1.00f );
	c[ImGuiCol_TitleBg] = ImVec4( 0.090f, 0.095f, 0.105f, 1.00f );
	c[ImGuiCol_TitleBgActive] = ImVec4( 0.14f, 0.16f, 0.19f, 1.00f );
	c[ImGuiCol_TitleBgCollapsed] = ImVec4( 0.090f, 0.095f, 0.105f, 0.75f );
	c[ImGuiCol_MenuBarBg] = ImVec4( 0.13f, 0.14f, 0.16f, 1.00f );
	c[ImGuiCol_ScrollbarBg] = ImVec4( 0.06f, 0.06f, 0.07f, 0.55f );
	c[ImGuiCol_ScrollbarGrab] = ImVec4( 0.28f, 0.30f, 0.33f, 1.00f );
	c[ImGuiCol_ScrollbarGrabHovered] = ImVec4( 0.36f, 0.39f, 0.43f, 1.00f );
	c[ImGuiCol_ScrollbarGrabActive] = accent;
	c[ImGuiCol_CheckMark] = accentHi;
	c[ImGuiCol_SliderGrab] = accent;
	c[ImGuiCol_SliderGrabActive] = accentHi;
	c[ImGuiCol_Button] = ImVec4( 0.22f, 0.24f, 0.27f, 1.00f );
	c[ImGuiCol_ButtonHovered] = accentLo;
	c[ImGuiCol_ButtonActive] = accent;
	c[ImGuiCol_Header] = ImVec4( 0.19f, 0.21f, 0.24f, 1.00f );
	c[ImGuiCol_HeaderHovered] = accentLo;
	c[ImGuiCol_HeaderActive] = accent;
	c[ImGuiCol_Separator] = ImVec4( 1.00f, 1.00f, 1.00f, 0.09f );
	c[ImGuiCol_SeparatorHovered] = accentLo;
	c[ImGuiCol_SeparatorActive] = accent;
	c[ImGuiCol_ResizeGrip] = ImVec4( 1.00f, 1.00f, 1.00f, 0.06f );
	c[ImGuiCol_ResizeGripHovered] = accentLo;
	c[ImGuiCol_ResizeGripActive] = accent;
	c[ImGuiCol_Tab] = ImVec4( 0.15f, 0.16f, 0.18f, 1.00f );
	c[ImGuiCol_TabHovered] = accentLo;
	c[ImGuiCol_TabSelected] = accent;
	c[ImGuiCol_TabSelectedOverline] = accentHi;
	c[ImGuiCol_TabDimmed] = ImVec4( 0.12f, 0.13f, 0.14f, 1.00f );
	c[ImGuiCol_TabDimmedSelected] = accentLo;
	c[ImGuiCol_TextSelectedBg] = ImVec4( accent.x, accent.y, accent.z, 0.40f );
	c[ImGuiCol_DragDropTarget] = accentHi;
	c[ImGuiCol_NavCursor] = accentHi;
	c[ImGuiCol_PlotLines] = ImVec4( 0.70f, 0.72f, 0.75f, 1.00f );
	c[ImGuiCol_PlotLinesHovered] = accentHi;
	c[ImGuiCol_PlotHistogram] = accent;
	c[ImGuiCol_PlotHistogramHovered] = accentHi;
}

void InitUI( const sg_environment* env, DrawUiFcn* drawGuiFcn )
{
	s_uiCallback = drawGuiFcn;

	simgui_desc_t desc = {};
	desc.color_format = env->defaults.color_format;
	desc.depth_format = env->defaults.depth_format;
	desc.sample_count = env->defaults.sample_count;
	desc.ini_filename = "imgui.ini";
	desc.logger.func = slog_func;
	desc.no_default_font = true;
	simgui_setup( &desc );

	ImPlot::CreateContext();

	s_uiScale = sapp_dpi_scale();

	ImGuiIO& io = ImGui::GetIO();
	io.Fonts->Clear();

	io.ConfigWindowsResizeFromEdges = true;

	guiApplyStyle();

	io.Fonts->AddFontDefaultVector();

	ImGuiStyle& style = ImGui::GetStyle();
	if ( s_uiScale != 1.0f )
	{
		style.ScaleAllSizes( s_uiScale );
	}

	style.FontSizeBase = floorf( 13.0f * s_uiScale );

	s_uiInitialized = true;
}

void ShutdownUI( void )
{
	if ( !s_uiInitialized )
		return;

	ImPlot::DestroyContext();
	simgui_shutdown();
	s_uiInitialized = false;
}

bool HandleEvent( const sapp_event* e )
{
	if ( !s_uiInitialized )
	{
		return false;
	}

	simgui_handle_event( e );
	const ImGuiIO& io = ImGui::GetIO();

	if ( e->type == SAPP_EVENTTYPE_KEY_DOWN || e->type == SAPP_EVENTTYPE_KEY_UP || e->type == SAPP_EVENTTYPE_CHAR )
	{
		return io.WantTextInput;
	}

	if ( e->type == SAPP_EVENTTYPE_MOUSE_DOWN || e->type == SAPP_EVENTTYPE_MOUSE_UP || e->type == SAPP_EVENTTYPE_MOUSE_MOVE ||
		 e->type == SAPP_EVENTTYPE_MOUSE_SCROLL )
	{
		return io.WantCaptureMouse;
	}

	return false;
}

static const float PANEL_GAP = 5.0f;

static float s_panelCursorY[2];
static GuiPanelAnchor s_panelOpenSide;
static bool s_panelOpen = false;

static void ResetUIPanelLayout( void )
{
	const ImGuiViewport* vp = ImGui::GetMainViewport();
	const float top = vp->WorkPos.y + ImGui::GetFrameHeight() + PANEL_GAP * s_uiScale;
	s_panelCursorY[GUI_ANCHOR_LEFT] = top;
	s_panelCursorY[GUI_ANCHOR_RIGHT] = top;
	s_panelOpen = false;
}

bool BeginPanel( const char* name, GuiPanelAnchor anchor, float width )
{
	ImGuiViewport* vp = ImGui::GetMainViewport();
	float gap = PANEL_GAP * s_uiScale;
	float w = width * s_uiScale;
	float cursorY = s_panelCursorY[anchor];

	float anchorX;
	ImVec2 pivot;
	if ( anchor == GUI_ANCHOR_RIGHT )
	{
		anchorX = vp->WorkPos.x + vp->WorkSize.x - gap;
		pivot = ImVec2( 1.0f, 0.0f );
	}
	else
	{
		anchorX = vp->WorkPos.x + gap;
		pivot = ImVec2( 0.0f, 0.0f );
	}
	ImGui::SetNextWindowPos( ImVec2( anchorX, cursorY ), ImGuiCond_Always, pivot );

	float maxH = vp->WorkPos.y + vp->WorkSize.y - gap - cursorY;
	if ( maxH < ImGui::GetFrameHeight() )
		maxH = ImGui::GetFrameHeight();
	ImGui::SetNextWindowSizeConstraints( ImVec2( w, 0.0f ), ImVec2( w, maxH ) );
	ImGui::SetNextWindowSize( ImVec2( w, 0.0f ), ImGuiCond_Always );

	s_panelOpenSide = anchor;
	s_panelOpen = true;
	return ImGui::Begin( name, nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize );
}

void EndPanel( void )
{
	if ( s_panelOpen )
	{
		s_panelCursorY[s_panelOpenSide] += ImGui::GetWindowSize().y + PANEL_GAP * s_uiScale;
		s_panelOpen = false;
	}
	ImGui::End();
}

static void RenderText()
{
	const int n = GetTextCount();
	if ( n <= 0 )
		return;

	const CameraState cam = GetCameraState();
	if ( cam.viewportW <= 0 || cam.viewportH <= 0 )
		return;

	ImGuiViewport* vp = ImGui::GetMainViewport();
	const int vpW = (int)vp->Size.x;
	const int vpH = (int)vp->Size.y;
	if ( vpW <= 0 || vpH <= 0 )
		return;
	ImDrawList* dl = ImGui::GetBackgroundDrawList();
	const ImVec2 origin = vp->Pos;

	for ( int i = 0; i < n; ++i )
	{
		const TextEntry* e = GetTextAt( i );
		float sx, sy;
		if ( !ResolveTextScreenPos( e, cam.view, cam.proj, vpW, vpH, &sx, &sy ) )
		{
			continue;
		}
		const auto byte = []( float v ) -> uint32_t {
			if ( v < 0.0f )
				v = 0.0f;
			if ( v > 1.0f )
				v = 1.0f;
			return (uint32_t)( v * 255.0f + 0.5f );
		};
		const uint32_t col = IM_COL32( byte( e->color.x ), byte( e->color.y ), byte( e->color.z ), byte( e->color.w ) );
		dl->AddText( nullptr, 0.0f, ImVec2( origin.x + sx, origin.y + sy ), col, e->text );
	}
}

void StartUIFrame( float dtSec )
{
	if ( !s_uiInitialized )
	{
		return;
	}

	simgui_frame_desc_t fd = {};
	fd.width = sapp_width();
	fd.height = sapp_height();

	const float kMinDt = 1.0f / 1000.0f;
	fd.delta_time = ( dtSec >= kMinDt ) ? (double)dtSec : (double)kMinDt;

	fd.dpi_scale = 1.0f;
	simgui_new_frame( &fd );

	ResetUIPanelLayout();

	if ( s_uiCallback != nullptr )
	{
		s_uiCallback();
	}

	RenderText();
}

void RenderUI( const sg_swapchain* swapchain )
{
	if ( !s_uiInitialized )
	{
		return;
	}

	sg_pass pass = {};
	pass.action.colors[0].load_action = SG_LOADACTION_LOAD;
	pass.action.depth.load_action = SG_LOADACTION_DONTCARE;
	pass.action.stencil.load_action = SG_LOADACTION_DONTCARE;
	pass.swapchain = *swapchain;
	pass.label = "imgui";
	sg_begin_pass( &pass );
	simgui_render();
	sg_end_pass();
}
