const float XE_GTAO_PI = 3.1415926535897932384626433832795;
const float XE_GTAO_PI_HALF = 1.5707963267948966192313216916398;

const float XE_GTAO_OCCLUSION_TERM_SCALE = 1.5;

const float XE_GTAO_DEPTH_MIP_LEVELS = 5.0;

const uint XE_HILBERT_WIDTH = 64u;

struct GTAOConsts
{
	vec2 ViewportPixelSize;
	vec2 NDCToViewMul;
	vec2 NDCToViewAdd;
	vec2 NDCToViewMul_x_PixelSize;
	float EffectRadius;
	float EffectFalloffRange;
	float RadiusMultiplier;
	float FinalValuePower;
	float SampleDistributionPower;
	float ThinOccluderCompensation;
	float DepthMIPSamplingOffset;
	float SliceCount;
	float StepsPerSlice;
};

float XeGTAO_FastSqrt(float x)
{
	return intBitsToFloat(0x1fbd1df5 + (floatBitsToInt(x) >> 1));
}

float XeGTAO_FastACos(float inX)
{
	float x = abs(inX);
	float res = -0.156583 * x + XE_GTAO_PI_HALF;
	res *= XeGTAO_FastSqrt(1.0 - x);
	return (inX >= 0.0) ? res : (XE_GTAO_PI - res);
}

float XeGTAO_ClampDepth(float depth)
{
	return clamp(depth, 0.0, 3.402823466e+38);
}

float XeGTAO_ScreenSpaceToViewSpaceDepth(float screenDepth, GTAOConsts consts)
{
	return screenDepth;
}

vec3 XeGTAO_ComputeViewspacePosition(vec2 screenPos, float viewspaceDepth, GTAOConsts consts) {
	vec3 ret;
	ret.xy = (consts.NDCToViewMul * screenPos + consts.NDCToViewAdd) * viewspaceDepth;
	ret.z = -viewspaceDepth;
	return ret;
}

vec4 XeGTAO_CalculateEdges(float centerZ, float leftZ, float rightZ, float topZ, float bottomZ)
{
	vec4 edgesLRTB = vec4(leftZ, rightZ, topZ, bottomZ) - centerZ;

	float slopeLR = (edgesLRTB.y - edgesLRTB.x) * 0.5;
	float slopeTB = (edgesLRTB.w - edgesLRTB.z) * 0.5;
	vec4 edgesLRTBSlopeAdjusted = edgesLRTB + vec4(slopeLR, - slopeLR, slopeTB, - slopeTB);
	edgesLRTB = min(abs(edgesLRTB), abs(edgesLRTBSlopeAdjusted));
	return vec4(clamp((1.25 - edgesLRTB / (centerZ * 0.011)), 0.0, 1.0));
}

float XeGTAO_PackEdges(vec4 edgesLRTB)
{
	edgesLRTB = round(clamp(edgesLRTB, 0.0, 1.0) * 2.9);
	return dot(edgesLRTB, vec4(64.0 / 255.0, 16.0 / 255.0, 4.0 / 255.0, 1.0 / 255.0));
}

vec4 XeGTAO_UnpackEdges(float packed_val)
{
	uint p = uint(packed_val * 255.5);
	vec4 edgesLRTB;
	edgesLRTB.x = float((p >> 6)& 0x03u) / 3.0;
	edgesLRTB.y = float((p >> 4)& 0x03u) / 3.0;
	edgesLRTB.z = float((p >> 2)& 0x03u) / 3.0;
	edgesLRTB.w = float((p >> 0)& 0x03u) / 3.0;
	return clamp(edgesLRTB, 0.0, 1.0);
}

vec3 XeGTAO_CalculateNormal(vec4 edgesLRTB, vec3 pixCenterPos,
	vec3 pixLPos, vec3 pixRPos, vec3 pixTPos, vec3 pixBPos)
{
	vec4 acceptedNormals = clamp(vec4(
			edgesLRTB.x * edgesLRTB.z,
			edgesLRTB.z * edgesLRTB.y,
			edgesLRTB.y * edgesLRTB.w,
		edgesLRTB.w * edgesLRTB.x) + 0.01, 0.0, 1.0);

	pixLPos = normalize(pixLPos - pixCenterPos);
	pixRPos = normalize(pixRPos - pixCenterPos);
	pixTPos = normalize(pixTPos - pixCenterPos);
	pixBPos = normalize(pixBPos - pixCenterPos);

	vec3 pixelNormal = acceptedNormals.x * cross(pixTPos, pixLPos)
	+ acceptedNormals.y * cross(pixRPos, pixTPos)
	+ acceptedNormals.z * cross(pixBPos, pixRPos)
	+ acceptedNormals.w * cross(pixLPos, pixBPos);

	return normalize(pixelNormal);
}

uint HilbertIndex(uint posX, uint posY)
{
	uint index = 0u;
	for (uint curLevel = XE_HILBERT_WIDTH / 2u; curLevel > 0u; curLevel /= 2u)
	{
		uint regionX = (posX & curLevel) > 0u ? 1u : 0u;
		uint regionY = (posY & curLevel) > 0u ? 1u : 0u;
		index += curLevel * curLevel * ((3u * regionX)^ regionY);
		if (regionY == 0u)
		{
			if (regionX == 1u)
			{
				posX = (XE_HILBERT_WIDTH - 1u) - posX;
				posY = (XE_HILBERT_WIDTH - 1u) - posY;
			}
			uint temp = posX;
			posX = posY;
			posY = temp;
		}
	}
	return index;
}
