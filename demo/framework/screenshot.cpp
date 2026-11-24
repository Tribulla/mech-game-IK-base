#if defined( _MSC_VER ) && !defined( _CRT_SECURE_NO_WARNINGS )
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "screenshot.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#if defined( _MSC_VER )
#pragma warning( push, 0 )
#endif
#include "stb_image_write.h"
#if defined( _MSC_VER )
#pragma warning( pop )
#endif

#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>

#if defined( _WIN32 )
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <d3d11.h>
#include <direct.h>
#else
#include <sys/stat.h>
#endif

static bool s_requested = false;
static char s_path[512];

void RequestScreenshot( const char* path )
{
	s_requested = true;
	if ( path != nullptr )
	{
		snprintf( s_path, sizeof( s_path ), "%s", path );
	}
	else
	{
		s_path[0] = 0;
	}
}

bool IsScreenshotRequested()
{
	return s_requested;
}

bool WritePng( const char* path, const unsigned char* rgba, int width, int height )
{
	stbi_write_png_compression_level = 6;
	return stbi_write_png( path, width, height, 4, rgba, width * 4 ) != 0;
}

static void MakeAutoPath( char* out, size_t size, const char* sampleName )
{
#if defined( _WIN32 )
	_mkdir( "screenshots" );
#else
	mkdir( "screenshots", 0755 );
#endif
	char clean[128];
	int n = 0;
	for ( const char* s = sampleName ? sampleName : "sample"; *s && n < 120; ++s )
	{
		char c = *s;
		clean[n++] = isalnum( (unsigned char)c ) ? (char)tolower( (unsigned char)c ) : '_';
	}
	clean[n] = 0;
	for ( int i = 1; i < 10000; ++i )
	{
		snprintf( out, size, "screenshots/%s_%03d.png", clean, i );
		FILE* f = fopen( out, "rb" );
		if ( f == nullptr )
		{
			return;
		}
		fclose( f );
	}
}

bool CaptureScreenshotIfRequested( const sg_swapchain* swapchain, const char* sampleName )
{
	if ( s_requested == false )
	{
		return false;
	}
	s_requested = false;

	char path[512];
	if ( s_path[0] != 0 )
	{
		snprintf( path, sizeof( path ), "%s", s_path );
	}
	else
	{
		MakeAutoPath( path, sizeof( path ), sampleName );
	}

#if defined( _WIN32 )
	if ( sg_query_backend() != SG_BACKEND_D3D11 )
	{
		fprintf( stderr, "screenshot: only the D3D11 backend is supported\n" );
		return false;
	}

	ID3D11RenderTargetView* rtv = (ID3D11RenderTargetView*)swapchain->d3d11.render_view;
	ID3D11Device* device = (ID3D11Device*)sg_d3d11_device();
	ID3D11DeviceContext* context = (ID3D11DeviceContext*)sg_d3d11_device_context();
	if ( rtv == nullptr || device == nullptr || context == nullptr )
	{
		fprintf( stderr, "screenshot: no D3D11 render target\n" );
		return false;
	}

	ID3D11Resource* resource = nullptr;
	rtv->GetResource( &resource );
	ID3D11Texture2D* backBuffer = nullptr;
	HRESULT hr = resource->QueryInterface( __uuidof( ID3D11Texture2D ), (void**)&backBuffer );
	resource->Release();
	if ( FAILED( hr ) || backBuffer == nullptr )
	{
		return false;
	}

	D3D11_TEXTURE2D_DESC desc;
	backBuffer->GetDesc( &desc );
	desc.Usage = D3D11_USAGE_STAGING;
	desc.BindFlags = 0;
	desc.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
	desc.MiscFlags = 0;
	desc.SampleDesc.Count = 1;
	desc.SampleDesc.Quality = 0;

	ID3D11Texture2D* staging = nullptr;
	hr = device->CreateTexture2D( &desc, nullptr, &staging );
	if ( FAILED( hr ) )
	{
		backBuffer->Release();
		return false;
	}
	context->CopyResource( staging, backBuffer );

	D3D11_MAPPED_SUBRESOURCE mapped;
	hr = context->Map( staging, 0, D3D11_MAP_READ, 0, &mapped );
	bool ok = false;
	if ( SUCCEEDED( hr ) )
	{
		bool bgra = desc.Format == DXGI_FORMAT_B8G8R8A8_UNORM || desc.Format == DXGI_FORMAT_B8G8R8A8_UNORM_SRGB;
		std::vector<unsigned char> pixels( (size_t)desc.Width * desc.Height * 4 );
		for ( UINT y = 0; y < desc.Height; ++y )
		{
			const unsigned char* src = (const unsigned char*)mapped.pData + (size_t)y * mapped.RowPitch;
			unsigned char* dst = pixels.data() + (size_t)y * desc.Width * 4;
			for ( UINT x = 0; x < desc.Width; ++x )
			{
				dst[4 * x + 0] = bgra ? src[4 * x + 2] : src[4 * x + 0];
				dst[4 * x + 1] = src[4 * x + 1];
				dst[4 * x + 2] = bgra ? src[4 * x + 0] : src[4 * x + 2];
				dst[4 * x + 3] = 255;
			}
		}
		context->Unmap( staging, 0 );
		ok = WritePng( path, pixels.data(), (int)desc.Width, (int)desc.Height );
	}

	staging->Release();
	backBuffer->Release();

	if ( ok )
	{
		fprintf( stderr, "screenshot: saved %s\n", path );
	}
	else
	{
		fprintf( stderr, "screenshot: failed to write %s\n", path );
	}
	return ok;
#else
	(void)swapchain;
	fprintf( stderr, "screenshot: not implemented on this backend (%s)\n", path );
	return false;
#endif
}
