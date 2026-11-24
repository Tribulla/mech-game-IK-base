#pragma sokol @module gtao_main_pass

#pragma sokol @ctype vec2 Vec2
#pragma sokol @ctype vec4 Vec4
#pragma sokol @ctype mat4 Mat4

#pragma sokol @cs cs

layout( binding = 0 ) uniform ub_gtao
{
	ivec2 ViewportSize;
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
}
gtao;

layout( binding = 0, r32f ) uniform writeonly image2D out_noisy_ao;
layout( binding = 1, r32f ) uniform writeonly image2D out_edges;

layout( binding = 2 ) uniform texture2D tex_raw_depth;
layout( binding = 3 ) uniform texture2D tex_prefilter_depth;
layout( binding = 0 ) uniform sampler smp_point_clamp;

#pragma sokol @include ../common/xe_gtao_common.glsl

vec2 SpatioTemporalNoise( uvec2 pixCoord, int temporalIndex )
{
	uint index = HilbertIndex( pixCoord.x, pixCoord.y );
	index += 288u * ( uint( temporalIndex ) % 64u );

	return fract( 0.5 + float( index ) * vec2( 0.75487766624669276005, 0.5698402909980532659114 ) );
}

vec3 XeGTAO_ComputeViewspaceNormal( ivec2 pixCoord, GTAOConsts consts )
{
	vec2 normalizedScreenPos = ( vec2( pixCoord ) + 0.5 ) * consts.ViewportPixelSize;

	ivec2 vmax = gtao.ViewportSize - 1;
	ivec2 cC = clamp( pixCoord, ivec2( 0 ), vmax );
	ivec2 cL = clamp( pixCoord + ivec2( -1, 0 ), ivec2( 0 ), vmax );
	ivec2 cR = clamp( pixCoord + ivec2( 1, 0 ), ivec2( 0 ), vmax );
	ivec2 cT = clamp( pixCoord + ivec2( 0, -1 ), ivec2( 0 ), vmax );
	ivec2 cB = clamp( pixCoord + ivec2( 0, 1 ), ivec2( 0 ), vmax );

	float viewspaceZ =
		XeGTAO_ScreenSpaceToViewSpaceDepth( texelFetch( sampler2D( tex_raw_depth, smp_point_clamp ), cC, 0 ).r, consts );
	float pixLZ =
		XeGTAO_ScreenSpaceToViewSpaceDepth( texelFetch( sampler2D( tex_raw_depth, smp_point_clamp ), cL, 0 ).r, consts );
	float pixRZ =
		XeGTAO_ScreenSpaceToViewSpaceDepth( texelFetch( sampler2D( tex_raw_depth, smp_point_clamp ), cR, 0 ).r, consts );
	float pixTZ =
		XeGTAO_ScreenSpaceToViewSpaceDepth( texelFetch( sampler2D( tex_raw_depth, smp_point_clamp ), cT, 0 ).r, consts );
	float pixBZ =
		XeGTAO_ScreenSpaceToViewSpaceDepth( texelFetch( sampler2D( tex_raw_depth, smp_point_clamp ), cB, 0 ).r, consts );

	vec4 edgesLRTB = XeGTAO_CalculateEdges( viewspaceZ, pixLZ, pixRZ, pixTZ, pixBZ );

	vec3 CENTER = XeGTAO_ComputeViewspacePosition( normalizedScreenPos, viewspaceZ, consts );
	vec3 LEFT =
		XeGTAO_ComputeViewspacePosition( normalizedScreenPos + vec2( -1.0, 0.0 ) * consts.ViewportPixelSize, pixLZ, consts );
	vec3 RIGHT =
		XeGTAO_ComputeViewspacePosition( normalizedScreenPos + vec2( 1.0, 0.0 ) * consts.ViewportPixelSize, pixRZ, consts );
	vec3 TOP =
		XeGTAO_ComputeViewspacePosition( normalizedScreenPos + vec2( 0.0, -1.0 ) * consts.ViewportPixelSize, pixTZ, consts );
	vec3 BOTTOM =
		XeGTAO_ComputeViewspacePosition( normalizedScreenPos + vec2( 0.0, 1.0 ) * consts.ViewportPixelSize, pixBZ, consts );
	return XeGTAO_CalculateNormal( edgesLRTB, CENTER, LEFT, RIGHT, TOP, BOTTOM );
}

float XeGTAO_OutputWorkingTerm( ivec2 pixCoord, float visibility )
{
	visibility = clamp( visibility / XE_GTAO_OCCLUSION_TERM_SCALE, 0.0, 1.0 );
	imageStore( out_noisy_ao, pixCoord, vec4( visibility, 0.0, 0.0, 0.0 ) );
	return visibility;
}

float XeGTAO_MainPass( ivec2 pixCoord, vec2 localNoise, vec3 viewspaceNormal,
					   float centerViewspaceZ, 
					   vec4 edgesLRTB,		   
					   GTAOConsts consts )
{
	vec2 normalizedScreenPos = ( vec2( pixCoord ) + 0.5 ) * consts.ViewportPixelSize;
	float viewspaceZ = centerViewspaceZ;

	imageStore( out_edges, pixCoord, vec4( XeGTAO_PackEdges( edgesLRTB ), 0.0, 0.0, 0.0 ) );

	viewspaceZ *= 0.99999;

	vec3 pixCenterPos = XeGTAO_ComputeViewspacePosition( normalizedScreenPos, viewspaceZ, consts );
	vec3 viewVec = normalize( -pixCenterPos );

	float sliceCount = consts.SliceCount;
	float stepsPerSlice = consts.StepsPerSlice;
	const float effectRadius = consts.EffectRadius * consts.RadiusMultiplier;
	const float falloffRange = consts.EffectFalloffRange * effectRadius;
	const float falloffFrom = effectRadius * ( 1.0 - consts.EffectFalloffRange );
	const float falloffMul = -1.0 / falloffRange;
	const float falloffAdd = falloffFrom / falloffRange + 1.0;
	const float sampleDistributionPower = consts.SampleDistributionPower;
	const float thinOccluderCompensation = consts.ThinOccluderCompensation;

	float visibility = 0.0;

	{
		const float noiseSlice = localNoise.x;
		const float noiseSample = localNoise.y;
		const float pixelTooCloseThreshold = 1.3;

		vec2 pixelDirRBViewspaceSizeAtCenterZ = vec2( viewspaceZ, viewspaceZ ) * consts.NDCToViewMul_x_PixelSize;
		float screenspaceRadius = effectRadius / pixelDirRBViewspaceSizeAtCenterZ.x;

		visibility += clamp( ( 10.0 - screenspaceRadius ) / 100.0, 0.0, 1.0 ) * 0.5;

		float minS = pixelTooCloseThreshold / screenspaceRadius;

		for ( float slice = 0.0; slice < sliceCount; slice += 1.0 )
		{
			float sliceK = ( slice + noiseSlice ) / sliceCount;
			float phi = sliceK * XE_GTAO_PI;
			float cosPhi = cos( phi );
			float sinPhi = sin( phi );
			vec2 omega = vec2( cosPhi, -sinPhi );
			omega *= screenspaceRadius;

			vec3 directionVec = vec3( cosPhi, sinPhi, 0.0 );
			vec3 orthoDirectionVec = directionVec - ( dot( directionVec, viewVec ) * viewVec );
			vec3 axisVec = normalize( cross( orthoDirectionVec, viewVec ) );

			vec3 projectedNormalVec = viewspaceNormal - axisVec * dot( viewspaceNormal, axisVec );

			float signNorm = sign( dot( orthoDirectionVec, projectedNormalVec ) );
			float projectedNormalVecLength = length( projectedNormalVec );
			float cosNorm = clamp( dot( projectedNormalVec, viewVec ) / projectedNormalVecLength, 0.0, 1.0 );

			float n = signNorm * XeGTAO_FastACos( cosNorm );

			const float lowHorizonCos0 = cos( n + XE_GTAO_PI_HALF );
			const float lowHorizonCos1 = cos( n - XE_GTAO_PI_HALF );

			float horizonCos0 = lowHorizonCos0;
			float horizonCos1 = lowHorizonCos1;

			for ( float step_ = 0.0; step_ < stepsPerSlice; step_ += 1.0 )
			{

				float stepBaseNoise = ( slice + step_ * stepsPerSlice ) * 0.6180339887498948482;
				float stepNoise = fract( noiseSample + stepBaseNoise );

				float s = ( step_ + stepNoise ) / stepsPerSlice;

				s = pow( max( 0.0, s ), sampleDistributionPower );
				s += minS;

				vec2 sampleOffset = s * omega;
				float sampleOffsetLength = length( sampleOffset );

				float mipLevel =
					clamp( log2( sampleOffsetLength ) - consts.DepthMIPSamplingOffset, 0.0, XE_GTAO_DEPTH_MIP_LEVELS );

				sampleOffset = round( sampleOffset ) * consts.ViewportPixelSize;

				vec2 sampleScreenPos0 = normalizedScreenPos + sampleOffset;
				float SZ0 = textureLod( sampler2D( tex_prefilter_depth, smp_point_clamp ), sampleScreenPos0, mipLevel ).x;
				vec3 samplePos0 = XeGTAO_ComputeViewspacePosition( sampleScreenPos0, SZ0, consts );

				vec2 sampleScreenPos1 = normalizedScreenPos - sampleOffset;
				float SZ1 = textureLod( sampler2D( tex_prefilter_depth, smp_point_clamp ), sampleScreenPos1, mipLevel ).x;
				vec3 samplePos1 = XeGTAO_ComputeViewspacePosition( sampleScreenPos1, SZ1, consts );

				vec3 sampleDelta0 = samplePos0 - pixCenterPos;
				vec3 sampleDelta1 = samplePos1 - pixCenterPos;
				float sampleDist0 = length( sampleDelta0 );
				float sampleDist1 = length( sampleDelta1 );

				vec3 sampleHorizonVec0 = sampleDelta0 / sampleDist0;
				vec3 sampleHorizonVec1 = sampleDelta1 / sampleDist1;

				float falloffBase0 =
					length( vec3( sampleDelta0.x, sampleDelta0.y, sampleDelta0.z * ( 1.0 + thinOccluderCompensation ) ) );
				float falloffBase1 =
					length( vec3( sampleDelta1.x, sampleDelta1.y, sampleDelta1.z * ( 1.0 + thinOccluderCompensation ) ) );
				float weight0 = clamp( falloffBase0 * falloffMul + falloffAdd, 0.0, 1.0 );
				float weight1 = clamp( falloffBase1 * falloffMul + falloffAdd, 0.0, 1.0 );

				float shc0 = dot( sampleHorizonVec0, viewVec );
				float shc1 = dot( sampleHorizonVec1, viewVec );

				shc0 = mix( lowHorizonCos0, shc0, weight0 );
				shc1 = mix( lowHorizonCos1, shc1, weight1 );

				horizonCos0 = max( horizonCos0, shc0 );
				horizonCos1 = max( horizonCos1, shc1 );
			}

			projectedNormalVecLength = mix( projectedNormalVecLength, 1.0, 0.05 );

			float h0 = -XeGTAO_FastACos( horizonCos1 );
			float h1 = XeGTAO_FastACos( horizonCos0 );
			float iarc0 = ( cosNorm + 2.0 * h0 * sin( n ) - cos( 2.0 * h0 - n ) ) * 0.25;
			float iarc1 = ( cosNorm + 2.0 * h1 * sin( n ) - cos( 2.0 * h1 - n ) ) * 0.25;
			float localVisibility = projectedNormalVecLength * ( iarc0 + iarc1 );
			visibility += localVisibility;
		}

		visibility /= sliceCount;

		visibility = pow( max( 0.0, visibility ), consts.FinalValuePower );

		visibility = max( 0.03, visibility );
	}

	return XeGTAO_OutputWorkingTerm( pixCoord, visibility );
}

layout( local_size_x = 8, local_size_y = 8, local_size_z = 1 ) in;

void main()
{
	ivec2 pixCoord = ivec2( gl_GlobalInvocationID.xy );
	if ( any( greaterThanEqual( pixCoord, gtao.ViewportSize ) ) )
	{
		return;
	}

	GTAOConsts consts;
	consts.ViewportPixelSize = gtao.ViewportPixelSize;
	consts.NDCToViewMul = gtao.NDCToViewMul;
	consts.NDCToViewAdd = gtao.NDCToViewAdd;
	consts.NDCToViewMul_x_PixelSize = gtao.NDCToViewMul_x_PixelSize;
	consts.EffectRadius = gtao.EffectRadius;
	consts.EffectFalloffRange = gtao.EffectFalloffRange;
	consts.RadiusMultiplier = gtao.RadiusMultiplier;
	consts.FinalValuePower = gtao.FinalValuePower;
	consts.SampleDistributionPower = gtao.SampleDistributionPower;
	consts.ThinOccluderCompensation = gtao.ThinOccluderCompensation;
	consts.DepthMIPSamplingOffset = gtao.DepthMIPSamplingOffset;
	consts.SliceCount = gtao.SliceCount;
	consts.StepsPerSlice = gtao.StepsPerSlice;

	vec3 viewspaceNormal = XeGTAO_ComputeViewspaceNormal( pixCoord, consts );

	ivec2 vmax = gtao.ViewportSize - 1;
	float centerZ = XeGTAO_ScreenSpaceToViewSpaceDepth(
		texelFetch( sampler2D( tex_raw_depth, smp_point_clamp ), clamp( pixCoord, ivec2( 0 ), vmax ), 0 ).r, consts );
	float leftZ = XeGTAO_ScreenSpaceToViewSpaceDepth(
		texelFetch( sampler2D( tex_raw_depth, smp_point_clamp ), clamp( pixCoord + ivec2( -1, 0 ), ivec2( 0 ), vmax ), 0 ).r,
		consts );
	float rightZ = XeGTAO_ScreenSpaceToViewSpaceDepth(
		texelFetch( sampler2D( tex_raw_depth, smp_point_clamp ), clamp( pixCoord + ivec2( 1, 0 ), ivec2( 0 ), vmax ), 0 ).r,
		consts );
	float topZ = XeGTAO_ScreenSpaceToViewSpaceDepth(
		texelFetch( sampler2D( tex_raw_depth, smp_point_clamp ), clamp( pixCoord + ivec2( 0, -1 ), ivec2( 0 ), vmax ), 0 ).r,
		consts );
	float botZ = XeGTAO_ScreenSpaceToViewSpaceDepth(
		texelFetch( sampler2D( tex_raw_depth, smp_point_clamp ), clamp( pixCoord + ivec2( 0, 1 ), ivec2( 0 ), vmax ), 0 ).r,
		consts );
	vec4 edgesLRTB = XeGTAO_CalculateEdges( centerZ, leftZ, rightZ, topZ, botZ );

	vec2 noise = SpatioTemporalNoise( uvec2( pixCoord ), 0 );

	XeGTAO_MainPass( pixCoord, noise, viewspaceNormal, centerZ, edgesLRTB, consts );
}
#pragma sokol @end

#pragma sokol @program prog cs
