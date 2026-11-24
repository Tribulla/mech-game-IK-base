#pragma sokol @module gtao_prefilter_depth

#pragma sokol @ctype vec2 Vec2
#pragma sokol @ctype vec4 Vec4
#pragma sokol @ctype mat4 Mat4

#pragma sokol @cs cs

layout(binding = 0) uniform ub_gtao
{
	ivec2 ViewportSize;
	float EffectRadius;
	float EffectFalloffRange;
	float RadiusMultiplier;
} gtao;

layout(binding = 0, r32f) uniform writeonly image2D out_depth0;
layout(binding = 1, r32f) uniform writeonly image2D out_depth1;
layout(binding = 2, r32f) uniform writeonly image2D out_depth2;
layout(binding = 3, r32f) uniform writeonly image2D out_depth3;
layout(binding = 4, r32f) uniform writeonly image2D out_depth4;

layout(binding = 5) uniform texture2D tex_src_depth;
layout(binding = 0) uniform sampler smp_src_depth;

float XeGTAO_ClampDepth(float depth) {
	return clamp(depth, 0.0, 3.402823466e+38);
}

float XeGTAO_DepthMIPFilter(float depth0, float depth1, float depth2, float depth3) {
	float maxDepth = max(max(depth0, depth1), max(depth2, depth3));

const float depthRangeScaleFactor = 0.75; 
float effectRadius = depthRangeScaleFactor * gtao.EffectRadius * gtao.RadiusMultiplier;
float falloffRange = gtao.EffectFalloffRange * effectRadius;
float falloffFrom = effectRadius * (1.0 - gtao.EffectFalloffRange);
float falloffMul = -1.0 / falloffRange;
float falloffAdd = falloffFrom / falloffRange + 1.0;

float weight0 = clamp((maxDepth - depth0) * falloffMul + falloffAdd, 0.0, 1.0);
float weight1 = clamp((maxDepth - depth1) * falloffMul + falloffAdd, 0.0, 1.0);
float weight2 = clamp((maxDepth - depth2) * falloffMul + falloffAdd, 0.0, 1.0);
float weight3 = clamp((maxDepth - depth3) * falloffMul + falloffAdd, 0.0, 1.0);

float weightSum = weight0 + weight1 + weight2 + weight3;
return (weight0 * depth0 + weight1 * depth1 +
weight2 * depth2 + weight3 * depth3) / weightSum;
}

shared float g_scratchDepths[8][8];

layout(local_size_x = 8, local_size_y = 8, local_size_z = 1)in;

void main() {
ivec2 dispatchThreadID = ivec2(gl_GlobalInvocationID.xy);
ivec2 groupThreadID = ivec2(gl_LocalInvocationID.xy);
ivec2 viewportMax = ivec2(gtao.ViewportSize) - 1;

ivec2 pixCoord = dispatchThreadID * 2;
ivec2 p0 = clamp(pixCoord + ivec2(0, 0), ivec2(0), viewportMax);
ivec2 p1 = clamp(pixCoord + ivec2(1, 0), ivec2(0), viewportMax);
ivec2 p2 = clamp(pixCoord + ivec2(0, 1), ivec2(0), viewportMax);
ivec2 p3 = clamp(pixCoord + ivec2(1, 1), ivec2(0), viewportMax);
float depth0 = XeGTAO_ClampDepth(texelFetch(sampler2D(tex_src_depth, smp_src_depth), p0, 0).r);
float depth1 = XeGTAO_ClampDepth(texelFetch(sampler2D(tex_src_depth, smp_src_depth), p1, 0).r);
float depth2 = XeGTAO_ClampDepth(texelFetch(sampler2D(tex_src_depth, smp_src_depth), p2, 0).r);
float depth3 = XeGTAO_ClampDepth(texelFetch(sampler2D(tex_src_depth, smp_src_depth), p3, 0).r);

if (all(lessThan(pixCoord + ivec2(0, 0), ivec2(gtao.ViewportSize)))) {
	imageStore(out_depth0, pixCoord + ivec2(0, 0), vec4(depth0, 0.0, 0.0, 0.0));
}
if (all(lessThan(pixCoord + ivec2(1, 0), ivec2(gtao.ViewportSize)))) {
	imageStore(out_depth0, pixCoord + ivec2(1, 0), vec4(depth1, 0.0, 0.0, 0.0));
}
if (all(lessThan(pixCoord + ivec2(0, 1), ivec2(gtao.ViewportSize)))) {
	imageStore(out_depth0, pixCoord + ivec2(0, 1), vec4(depth2, 0.0, 0.0, 0.0));
}
if (all(lessThan(pixCoord + ivec2(1, 1), ivec2(gtao.ViewportSize)))) {
	imageStore(out_depth0, pixCoord + ivec2(1, 1), vec4(depth3, 0.0, 0.0, 0.0));
}

float dm1 = XeGTAO_DepthMIPFilter(depth0, depth1, depth2, depth3);
if (all(lessThan(dispatchThreadID, ivec2(gtao.ViewportSize)))) {
	imageStore(out_depth1, dispatchThreadID, vec4(dm1, 0.0, 0.0, 0.0));
}
g_scratchDepths[groupThreadID.x][groupThreadID.y] = dm1;

barrier();

float dm2 = 0.0;
bool isMip2Producer = all(equal(groupThreadID & ivec2(1), ivec2(0)));
if (isMip2Producer) {
	float inTL = g_scratchDepths[groupThreadID.x + 0][groupThreadID.y + 0];
	float inTR = g_scratchDepths[groupThreadID.x + 1][groupThreadID.y + 0];
	float inBL = g_scratchDepths[groupThreadID.x + 0][groupThreadID.y + 1];
	float inBR = g_scratchDepths[groupThreadID.x + 1][groupThreadID.y + 1];

	dm2 = XeGTAO_DepthMIPFilter(inTL, inTR, inBL, inBR);

	ivec2 dispatchID2 = ivec2(uvec2(dispatchThreadID) >> 1u);
	if (all(lessThan(dispatchID2, ivec2(gtao.ViewportSize)))) {
		imageStore(out_depth2, dispatchID2, vec4(dm2, 0.0, 0.0, 0.0));
	}
	g_scratchDepths[groupThreadID.x][groupThreadID.y] = dm2;
}

barrier();

float dm3 = 0.0;
bool isMip3Producer = all(equal(groupThreadID & ivec2(3), ivec2(0)));
if (isMip3Producer) {
	float inTL = g_scratchDepths[groupThreadID.x + 0][groupThreadID.y + 0];
	float inTR = g_scratchDepths[groupThreadID.x + 2][groupThreadID.y + 0];
	float inBL = g_scratchDepths[groupThreadID.x + 0][groupThreadID.y + 2];
	float inBR = g_scratchDepths[groupThreadID.x + 2][groupThreadID.y + 2];

	dm3 = XeGTAO_DepthMIPFilter(inTL, inTR, inBL, inBR);
	ivec2 dispatchID4 = ivec2(uvec2(dispatchThreadID) >> 2u);
	if (all(lessThan(dispatchID4, ivec2(gtao.ViewportSize)))) {
		imageStore(out_depth3, dispatchID4, vec4(dm3, 0.0, 0.0, 0.0));
	}
	g_scratchDepths[groupThreadID.x][groupThreadID.y] = dm3;
}

barrier();

if (all(equal(groupThreadID, ivec2(0)))) {
	float inTL = g_scratchDepths[0][0];
	float inTR = g_scratchDepths[4][0];
	float inBL = g_scratchDepths[0][4];
	float inBR = g_scratchDepths[4][4];

	float dm4 = XeGTAO_DepthMIPFilter(inTL, inTR, inBL, inBR);
	ivec2 dispatchID8 = ivec2(uvec2(dispatchThreadID) >> 3u);
	if (all(lessThan(dispatchID8, ivec2(gtao.ViewportSize)))) {
		imageStore(out_depth4, dispatchID8, vec4(dm4, 0.0, 0.0, 0.0));
	}
}
}
#pragma sokol @end

#pragma sokol @program prog cs
