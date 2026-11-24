#pragma once

typedef enum OverlayOcclusionMode
{
	OVERLAY_OCCLUSION_HIDE = 0,

	OVERLAY_OCCLUSION_DIM = 1,

	OVERLAY_OCCLUSION_DASHED = 2,
} OverlayOcclusionMode;

typedef enum OverlayThicknessUnit
{
	OVERLAY_THICKNESS_PIXELS = 0,

	OVERLAY_THICKNESS_WORLD = 1,
} OverlayThicknessUnit;

typedef enum TransparentShadowCast
{
	TRANSPARENT_SHADOW_FULL = 0,

	TRANSPARENT_SHADOW_NONE = 1,
} TransparentShadowCast;
