#pragma sokol @module mask_sphere

#pragma sokol @ctype mat4 Mat4
#pragma sokol @ctype vec4 Vec4
#pragma sokol @ctype vec3 Vec3

#pragma sokol @vs vs

layout( binding = 0 ) uniform ub_frame
{
	mat4 view_proj;
};

struct instance
{
	vec4 center_radius; 
	vec4 kind;			
};

layout( binding = 0 ) readonly buffer instances
{
	instance items[];
};

in vec3 in_pos; 

sample out vec3 v_world_pos_proxy;
flat out vec3 v_center;
flat out float v_radius;
flat out float v_kind;

void main()
{
	instance inst = items[gl_InstanceIndex];
	vec3 center = inst.center_radius.xyz;
	float radius = inst.center_radius.w;

	vec3 world_pos = center + ( 2.0 * radius ) * in_pos;

	v_world_pos_proxy = world_pos;
	v_center = center;
	v_radius = radius;
	v_kind = inst.kind.x;

	gl_Position = view_proj * vec4( world_pos, 1.0 );
}
#pragma sokol @end

#pragma sokol @fs fs

layout( binding = 1 ) uniform ub_pass
{
	mat4 view_proj;	 
	vec4 camera_pos; 
};

sample in vec3 v_world_pos_proxy;
flat in vec3 v_center;
flat in float v_radius;
flat in float v_kind;

layout( location = 0 ) out vec4 out_kind;

void main()
{
	vec3 ro = camera_pos.xyz;
	vec3 rd = normalize( v_world_pos_proxy - ro );

	vec3 oc = ro - v_center;
	float b = dot( oc, rd );
	float c2 = dot( oc, oc ) - v_radius * v_radius;
	float h = b * b - c2;
	if ( h < 0.0 )
		discard;
	float t = -b - sqrt( h );
	if ( t < 0.0 )
		discard;

	vec3 hit_world = ro + t * rd;
	vec4 hit_clip = view_proj * vec4( hit_world, 1.0 );
	gl_FragDepth = hit_clip.z / hit_clip.w;

	out_kind = vec4( v_kind * 0.5, 0.0, 0.0, 1.0 );
}
#pragma sokol @end

#pragma sokol @program prog vs fs
