#pragma sokol @module highlight_outline

#pragma sokol @ctype vec4 Vec4

#pragma sokol @vs vs

void main()
{
	vec2 p = vec2( ( gl_VertexIndex == 1 ) ? 3.0 : -1.0, ( gl_VertexIndex == 2 ) ? 3.0 : -1.0 );
	gl_Position = vec4( p, 0.0, 1.0 );
}
#pragma sokol @end

#pragma sokol @fs fs

layout( binding = 0 ) uniform ub_pass
{
	vec4 hover_color;  
	vec4 select_color; 
	vec4 thickness_px; 
};

layout( binding = 0 ) uniform texture2D tex_mask;
layout( binding = 0 ) uniform sampler smp_mask;

layout( location = 0 ) out vec4 out_color;

const float KIND_EMPTY_THRESHOLD = 0.25;
const float KIND_SELECT_THRESHOLD = 0.75;

void main()
{
	ivec2 px = ivec2( gl_FragCoord.xy );
	ivec2 sz = textureSize( sampler2D( tex_mask, smp_mask ), 0 );

	float center = texelFetch( sampler2D( tex_mask, smp_mask ), px, 0 ).x;
	if ( center > KIND_EMPTY_THRESHOLD )
	{
		out_color = vec4( 0.0 );
		return;
	}

	int hoverR = clamp( int( thickness_px.x + 0.5 ), 1, 4 );
	int selectR = clamp( int( thickness_px.y + 0.5 ), 1, 4 );
	int maxR = max( hoverR, selectR );

	bool sawSelectInSelectR = false;
	bool sawHoverInHoverR = false;
	bool sawHoverInSelectR = false;

	for ( int r = 1; r <= maxR; ++r )
	{
		for ( int i = 0; i < 4; ++i )
		{
			ivec2 off;
			if ( i == 0 )
				off = ivec2( r, 0 );
			else if ( i == 1 )
				off = ivec2( -r, 0 );
			else if ( i == 2 )
				off = ivec2( 0, r );
			else
				off = ivec2( 0, -r );

			ivec2 q = px + off;
			if ( q.x < 0 || q.y < 0 || q.x >= sz.x || q.y >= sz.y )
				continue;
			float k = texelFetch( sampler2D( tex_mask, smp_mask ), q, 0 ).x;
			if ( k <= KIND_EMPTY_THRESHOLD )
				continue;

			if ( k >= KIND_SELECT_THRESHOLD )
			{
				if ( r <= selectR )
					sawSelectInSelectR = true;
			}
			else
			{
				if ( r <= hoverR )
					sawHoverInHoverR = true;
				if ( r <= selectR )
					sawHoverInSelectR = true;
			}
		}
	}

	if ( sawSelectInSelectR )
	{
		out_color = select_color;
		return;
	}
	if ( sawHoverInHoverR )
	{
		out_color = hover_color;
		return;
	}
	if ( sawHoverInSelectR )
	{
		out_color = hover_color;
		return;
	}
	out_color = vec4( 0.0 );
}
#pragma sokol @end

#pragma sokol @program prog vs fs
