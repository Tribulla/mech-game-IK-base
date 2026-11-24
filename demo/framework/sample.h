#pragma once

#include "host/camera.h"

#include "math3d.h"

#include <stdint.h>

bool IsKeyDown( int key );
void SetKeyDown( int key, bool down );

struct SampleContext
{
	void Save();
	void Load();

	Camera camera;
	class Sample* sample = nullptr;
	bool minimized = false;

	float mouseX = 0.0f;
	float mouseY = 0.0f;
	float mouseDX = 0.0f;
	float mouseDY = 0.0f;

	int windowWidth = 1920;
	int windowHeight = 1080;

	float hertz = 60.0f;
	bool pause = false;
	int singleStep = 0;

	bool restart = false;

	bool requestRestart = false;

	bool showUI = true;
	bool showMetrics = false;
	bool openSamplePicker = false;
	bool showControls = false;

	bool enableShadows = true;
	bool enableGtao = true;
	bool viewZUp = false;
	int debugView = 0;
	float drawDistance = 200.0f;

	int sampleIndex = 0;

	bool newUser = true;
};

struct DragHandle
{
	Vec3* point;
	float radius;
	bool visible;
};

class Sample
{
public:
	explicit Sample( SampleContext* context );
	virtual ~Sample();

	virtual void Update( float dt )
	{
		(void)dt;
	}

	virtual void Render()
	{
	}

	virtual void DrawControls()
	{
	}

	virtual void DrawWindows()
	{
	}

	virtual void DrawMetricsTabs()
	{
	}

	virtual void DrawViewMenu()
	{
	}

	virtual void Keyboard( int key, int mods )
	{
		(void)key;
		(void)mods;
	}

	virtual void MouseDown( Vec2 p, int button, int mods )
	{
		(void)p;
		(void)button;
		(void)mods;
	}
	virtual void MouseUp( Vec2 p, int button )
	{
		(void)p;
		(void)button;
	}
	virtual void MouseMove( Vec2 p )
	{
		(void)p;
	}

	virtual bool GetBounds( AABB* bounds )
	{
		(void)bounds;
		return false;
	}

	void DrawTextLine( const char* format, ... );

	void DrawTextLineColored( uint32_t paletteHex, const char* format, ... );

	PickRay MouseRay() const;
	PickRay ScreenRay( Vec2 p ) const;

	static bool RayHitPlaneY( const PickRay& ray, float height, Vec3* hit );

	int AddHandle( Vec3* point, float radius = 0.2f, bool visible = true );

	int ActiveHandle() const
	{
		return m_activeHandle;
	}

	void FrameBounds( AABB bounds, float padding = 1.2f );

	void Step();
	void RenderFrame();
	void ResetText();
	void HandleMouseDown( Vec2 p, int button, int mods );
	void HandleMouseUp( Vec2 p, int button );
	void HandleMouseMove( Vec2 p );
	bool ComputeBounds( AABB* bounds );
	void DrawMetrics();

	SampleContext* m_context;
	Camera* m_camera;

	int m_stepCount;
	double m_time;

	int m_textLine;
	int m_textIncrement;

	static constexpr int m_maxHandles = 64;
	DragHandle m_handles[m_maxHandles];
	int m_handleCount;
	int m_activeHandle;
	int m_hoverHandle;
	float m_handleDepth; 
	Vec3 m_handleGrabOffset;

	static constexpr int m_frameHistory = 512;
	float m_frameMs[m_frameHistory];
	int m_frameWrite;
};

#define MAX_SAMPLES 256

typedef Sample* SampleCreateFcn( SampleContext* context );

struct SampleEntry
{
	const char* category;
	const char* name;
	SampleCreateFcn* createFcn;
	const char* description;
};

extern SampleEntry g_sampleEntries[MAX_SAMPLES];
extern int g_sampleCount;

int RegisterSample( const char* category, const char* name, SampleCreateFcn* fcn, const char* description = nullptr );

int FindSample( const char* nameOrPath );

void SelectSample( SampleContext* context, int selection, bool restart );

void DrawUI( SampleContext* context );
