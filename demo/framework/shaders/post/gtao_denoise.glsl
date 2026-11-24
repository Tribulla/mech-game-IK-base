#pragma sokol @module gtao_denoise

#pragma sokol @ctype vec2 Vec2
#pragma sokol @ctype vec4 Vec4
#pragma sokol @ctype mat4 Mat4

#pragma sokol @cs cs

layout( binding = 0 ) uniform ub_gtao
{
	ivec2 ViewportSize;
	float DenoiseBlurBeta;
}
gtao;

layout( binding = 1 ) uniform ub_denoise
{
	int FinalApply;
	int _Pad0;
	int _Pad1;
	int _Pad2;
};

layout( binding = 0, r32f ) uniform writeonly image2D out_ao;

layout( binding = 1 ) uniform texture2D tex_edges;
layout( binding = 2 ) uniform texture2D tex_noisy_ao;
layout( binding = 0 ) uniform sampler smp_point_clamp;

#pragma sokol @include ../common/xe_gtao_common.glsl

void XeGTAO_AddSample( float ssaoValue, float edgeValue, inout float sum, inout float sumWeight )
{
	float weight = edgeValue;
	sum += weight * ssaoValue;
	sumWeight += weight;
}

float fetchEdge( ivec2 coord )
{
	ivec2 vmax = gtao.ViewportSize - 1;
	ivec2 c = clamp( coord, ivec2( 0 ), vmax );
	return texelFetch( sampler2D( tex_edges, smp_point_clamp ), c, 0 ).r;
}

float fetchAO( ivec2 coord )
{
	ivec2 vmax = gtao.ViewportSize - 1;
	ivec2 c = clamp( coord, ivec2( 0 ), vmax );
	return texelFetch( sampler2D( tex_noisy_ao, smp_point_clamp ), c, 0 ).r;
}

void XeGTAO_Output( ivec2 pixCoord, float outputValue, bool finalApply )
{
	float scale = finalApply ? XE_GTAO_OCCLUSION_TERM_SCALE : 1.0;
	imageStore( out_ao, pixCoord, vec4( outputValue * scale, 0.0, 0.0, 0.0 ) );
}

layout( local_size_x = 8, local_size_y = 8, local_size_z = 1 ) in;

void main()
{
	ivec2 dispatchThreadID = ivec2( gl_GlobalInvocationID.xy );

	ivec2 pixCoordBase = ivec2( dispatchThreadID.x * 2, dispatchThreadID.y );
	if ( any( greaterThanEqual( pixCoordBase, gtao.ViewportSize ) ) )
	{
		return;
	}

	bool finalApply = ( FinalApply != 0 );
	float blurAmount = finalApply ? gtao.DenoiseBlurBeta : ( gtao.DenoiseBlurBeta / 5.0 );
	const float diagWeight = 0.85 * 0.5;

	float edgesN[4][3];
	float ao[4][3];
	for ( int i = 0; i < 4; ++i )
	{
		for ( int j = 0; j < 3; ++j )
		{
			ivec2 c = pixCoordBase + ivec2( i - 1, j - 1 );
			edgesN[i][j] = fetchEdge( c );
			ao[i][j] = fetchAO( c );
		}
	}

	for ( int side = 0; side < 2; ++side )
	{
		ivec2 pixCoord = pixCoordBase + ivec2( side, 0 );
		if ( pixCoord.x >= gtao.ViewportSize.x )
		{
			break;
		}

		int cC = 1 + side;
		int cL = cC - 1;
		int cR = cC + 1;

		vec4 edgesC_LRTB = XeGTAO_UnpackEdges( edgesN[cC][1] );
		vec4 edgesL_LRTB = XeGTAO_UnpackEdges( edgesN[cL][1] );
		vec4 edgesR_LRTB = XeGTAO_UnpackEdges( edgesN[cR][1] );
		vec4 edgesT_LRTB = XeGTAO_UnpackEdges( edgesN[cC][0] );
		vec4 edgesB_LRTB = XeGTAO_UnpackEdges( edgesN[cC][2] );

		edgesC_LRTB *= vec4( edgesL_LRTB.y, edgesR_LRTB.x, edgesT_LRTB.w, edgesB_LRTB.z );

		const float leak_threshold = 2.5;
		const float leak_strength = 0.5;
		float edginess =
			( clamp( 4.0 - leak_threshold - dot( edgesC_LRTB, vec4( 1.0 ) ), 0.0, 1.0 ) / ( 4.0 - leak_threshold ) ) *
			leak_strength;
		edgesC_LRTB = clamp( edgesC_LRTB + edginess, 0.0, 1.0 );

		float weightTL = diagWeight * ( edgesC_LRTB.x * edgesL_LRTB.z + edgesC_LRTB.z * edgesT_LRTB.x );
		float weightTR = diagWeight * ( edgesC_LRTB.z * edgesT_LRTB.y + edgesC_LRTB.y * edgesR_LRTB.z );
		float weightBL = diagWeight * ( edgesC_LRTB.w * edgesB_LRTB.x + edgesC_LRTB.x * edgesL_LRTB.w );
		float weightBR = diagWeight * ( edgesC_LRTB.y * edgesR_LRTB.w + edgesC_LRTB.w * edgesB_LRTB.y );

		float ssaoValue = ao[cC][1];
		float ssaoValueL = ao[cL][1];
		float ssaoValueR = ao[cR][1];
		float ssaoValueT = ao[cC][0];
		float ssaoValueB = ao[cC][2];
		float ssaoValueTL = ao[cL][0];
		float ssaoValueTR = ao[cR][0];
		float ssaoValueBL = ao[cL][2];
		float ssaoValueBR = ao[cR][2];

		float sumWeight = blurAmount;
		float sum = ssaoValue * sumWeight;

		XeGTAO_AddSample( ssaoValueL, edgesC_LRTB.x, sum, sumWeight );
		XeGTAO_AddSample( ssaoValueR, edgesC_LRTB.y, sum, sumWeight );
		XeGTAO_AddSample( ssaoValueT, edgesC_LRTB.z, sum, sumWeight );
		XeGTAO_AddSample( ssaoValueB, edgesC_LRTB.w, sum, sumWeight );

		XeGTAO_AddSample( ssaoValueTL, weightTL, sum, sumWeight );
		XeGTAO_AddSample( ssaoValueTR, weightTR, sum, sumWeight );
		XeGTAO_AddSample( ssaoValueBL, weightBL, sum, sumWeight );
		XeGTAO_AddSample( ssaoValueBR, weightBR, sum, sumWeight );

		float aoTerm = sum / sumWeight;
		XeGTAO_Output( pixCoord, aoTerm, finalApply );
	}
}
#pragma sokol @end

#pragma sokol @program prog cs
