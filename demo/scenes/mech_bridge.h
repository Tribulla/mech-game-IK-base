#pragma once

#if defined( MECH_HAVE_PYTHON )
#define PY_SSIZE_T_CLEAN
#if defined( _DEBUG )
#undef _DEBUG
#include <Python.h>
#define _DEBUG
#else
#include <Python.h>
#endif
#endif

#include "gaitcore.h"
#include "gfx/timer.h"
#include "mech_world.h"

#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>

namespace mech
{

enum Backend
{
	BackendC = 0,
	BackendPython = 1
};

inline int EnvInt( const char* name, int fallback )
{
#if defined( _MSC_VER )
	char* v = nullptr;
	size_t len = 0;
	if ( _dupenv_s( &v, &len, name ) == 0 && v != nullptr )
	{
		int r = atoi( v );
		free( v );
		return r;
	}
	return fallback;
#else
	const char* v = getenv( name );
	return v ? atoi( v ) : fallback;
#endif
}

inline std::string EnvStr( const char* name )
{
#if defined( _MSC_VER )
	char* v = nullptr;
	size_t len = 0;
	std::string out;
	if ( _dupenv_s( &v, &len, name ) == 0 && v != nullptr )
	{
		out = v;
		free( v );
	}
	return out;
#else
	const char* v = getenv( name );
	return v ? std::string( v ) : std::string();
#endif
}

constexpr int kInputStride = GC_INPUT_STRIDE;	
constexpr int kHeaderFloats = GC_HEADER_FLOATS; 
constexpr int kLegExtra = GC_LEG_EXTRA_FLOATS;	
constexpr int kEventFloats = GC_EVENT_FLOATS;	
constexpr int kDebugFloats = GC_DEBUG_LINE_FLOATS; 

enum InputField
{
	InFwd = 0,
	InStrafe,
	InTurn,
	InRun,
	InSlow,
	InCrouch,
	InJump,
	InDrift,
	InClimb,
	InImpact,
	InGait,
	InAim
};

enum PoseField
{
	PosePos = 0,
	PoseRight = 3,
	PoseUp = 6,
	PoseFwd = 9,
	PoseSpeed = 12,
	PoseState = 13,
	PoseFlags = 14,
	PosePlanted = 15,
	PoseVel = 16,
	PosePhase = 19,
	PoseDrift = 20,
	PoseGait = 21,
	PoseSupport = 22,
	PoseCycle = 23,
	PoseWire = 24,
	PoseAnchor = 25
};

enum PoseFlag
{
	FlagDrift = 1,
	FlagWall = 2,
	FlagCrouch = 4,
	FlagClimb = 8,
	FlagSupported = 16,
	FlagSlow = 32,
	FlagRun = 64,
	FlagWire = 128,
	FlagLeap = 256
};

enum PoseState
{
	StateGround = GC_STATE_GROUND,
	StateAir = GC_STATE_AIR,
	StateMantle = GC_STATE_MANTLE,
	StateCrouchJump = GC_STATE_CROUCHJUMP
};

enum EventType
{
	EvStep = GC_EV_STEP,
	EvSpark = GC_EV_SPARK,
	EvLand = GC_EV_LAND,
	EvBoost = GC_EV_BOOST,
	EvImpact = GC_EV_IMPACT,
	EvSkid = GC_EV_SKID,
	EvMantle = GC_EV_MANTLE
};

enum DebugRole
{
	DbgLine = GC_DBG_LINE,
	DbgLineDim = GC_DBG_LINE_DIM,
	DbgInfo = GC_DBG_INFO,
	DbgHighlight = GC_DBG_HIGHLIGHT,
	DbgGood = GC_DBG_GOOD,
	DbgWarning = GC_DBG_WARNING,
	DbgError = GC_DBG_ERROR,
	DbgForce = GC_DBG_FORCE,
	DbgNormal = GC_DBG_NORMAL,
	DbgVelocity = GC_DBG_VELOCITY,
	DbgAxisX = GC_DBG_AXIS_X,
	DbgAxisY = GC_DBG_AXIS_Y,
	DbgAxisZ = GC_DBG_AXIS_Z
};

inline Vec4 DebugRoleColor( int role )
{
	switch ( role )
	{
		case DbgLineDim:
			return Color( Palette::LineDim );
		case DbgInfo:
			return Color( Palette::Info );
		case DbgHighlight:
			return Color( Palette::Highlight );
		case DbgGood:
			return Color( Palette::Good );
		case DbgWarning:
			return Color( Palette::Warning );
		case DbgError:
			return Color( Palette::Error );
		case DbgForce:
			return Color( Palette::Force );
		case DbgNormal:
			return Color( Palette::Normal );
		case DbgVelocity:
			return Color( Palette::Velocity );
		case DbgAxisX:
			return Color( Palette::AxisX );
		case DbgAxisY:
			return Color( Palette::AxisY );
		case DbgAxisZ:
			return Color( Palette::AxisZ );
		default:
			return Color( Palette::Line );
	}
}

struct MechInfo
{
	std::string name;
	std::string blurb;
	int kind = 0;
	int legs = 0;
	int jointsPerLeg = 0;
	Vec3 half = { 0.5f, 0.3f, 0.5f };
	float widths[4] = { 0.2f, 0.2f, 0.2f, 0.2f };
	int widthCount = 1;
	float mass = 1.0f;
	bool climber = false;
	int faction = 0;
	float wireMax = 0.0f;
	float walkSpeed = 5.0f;
	float runSpeed = 10.0f;
	int floats = 0;

	int LegStride() const
	{
		return jointsPerLeg * 3 + kLegExtra;
	}
};

struct MechStats
{
	std::string mode, gait, state;
	bool gaitOverride = false;
	float duty = 0.0f, cycle = 0.0f, maxStretch = 0.0f, simMs = 0.0f;
	int rays = 0;
	bool climbEnabled = false;
	bool crouchToggle = false;
};

constexpr int kProfileHeader = GC_PROFILE_HEADER;
constexpr int kProfileKinds = GC_PROFILE_KINDS;
constexpr int kProfileKindStride = GC_PROFILE_KIND_STRIDE;
constexpr int kProfileFloats = GC_PROFILE_FLOATS;

struct NativeQueryStats
{
	int rayCalls = 0;
	int pushCalls = 0;
	int heightCalls = 0;
	float rayMs = 0.0f;
	float pushMs = 0.0f;
	float heightMs = 0.0f;
};

inline const World*& ActiveWorld()
{
	static const World* s_world = nullptr;
	return s_world;
}

namespace detail
{

inline int CRaycast( void*, double ox, double oy, double oz, double dx, double dy, double dz, double maxDist,
					 GcRayHit* outHit )
{
	const World* w = ActiveWorld();
	if ( w == nullptr )
	{
		return 0;
	}
	float t = 0.0f;
	Vec3 p, n;
	int sid = 0;
	if ( w->Raycast( { (float)ox, (float)oy, (float)oz }, { (float)dx, (float)dy, (float)dz }, (float)maxDist, &t, &p, &n, &sid ) == false )
	{
		return 0;
	}
	outHit->t = (double)t;
	outHit->px = (double)p.x;
	outHit->py = (double)p.y;
	outHit->pz = (double)p.z;
	outHit->nx = (double)n.x;
	outHit->ny = (double)n.y;
	outHit->nz = (double)n.z;
	outHit->surface = sid;
	return 1;
}

inline int CSpherePush( void*, double cx, double cy, double cz, double radius, double* outXyz, double* outNxyz )
{
	const World* w = ActiveWorld();
	if ( w == nullptr )
	{
		outXyz[0] = cx;
		outXyz[1] = cy;
		outXyz[2] = cz;
		outNxyz[0] = 0.0;
		outNxyz[1] = 1.0;
		outNxyz[2] = 0.0;
		return 0;
	}
	Vec3 c = { (float)cx, (float)cy, (float)cz };
	Vec3 n = { 0.0f, 1.0f, 0.0f };
	bool hit = w->SpherePush( &c, (float)radius, &n );
	outXyz[0] = (double)c.x;
	outXyz[1] = (double)c.y;
	outXyz[2] = (double)c.z;
	outNxyz[0] = (double)n.x;
	outNxyz[1] = (double)n.y;
	outNxyz[2] = (double)n.z;
	return hit ? 1 : 0;
}

inline double CTerrainHeight( void*, double x, double z )
{
	const World* w = ActiveWorld();
	return w ? (double)w->Height( (float)x, (float)z ) : 0.0;
}

inline GcWorldCallbacks MakeWorldCallbacks()
{
	GcWorldCallbacks cb = {};
	cb.raycast = CRaycast;
	cb.sphere_push = CSpherePush;
	cb.terrain_height = CTerrainHeight;
	cb.user = nullptr;
	return cb;
}

struct BridgeState
{
	Backend backend = BackendC;
	bool backendInited = false;

	GcContext* cCtx = nullptr;
	int cThreads = 0; 
	bool cReady = false;
	std::string cInitMsg;

	bool pyTried = false;
	bool pyReady = false;
	bool pyError = false;
	bool pyProfileNative = false;
	NativeQueryStats pyNativeStats;
	std::string pyInitMessage;
	std::string pyLastError;
	std::string ikDir;
#if defined( MECH_HAVE_PYTHON )
	PyObject* pyApi = nullptr;
#endif
};

inline BridgeState& State()
{
	static BridgeState s;
	if ( s.backendInited == false )
	{
		s.backendInited = true;
		std::string b = EnvStr( "MECH_BACKEND" );
		int t = EnvInt( "MECH_THREADS", 0 );
		s.cThreads = t > 0 ? ClampInt( t, 1, gc_hardware_threads() ) : gc_hardware_threads();
#if defined( MECH_HAVE_PYTHON )
		if ( b == "python" || b == "Python" || b == "py" || b == "PY" )
		{
			s.backend = BackendPython;
		}
		else
#endif
		{
			s.backend = BackendC;
		}
	}
	return s;
}

#if defined( MECH_HAVE_PYTHON )

inline PyObject* EngineRaycast( PyObject*, PyObject* args )
{
	float ox, oy, oz, dx, dy, dz, maxd;
	if ( !PyArg_ParseTuple( args, "fffffff", &ox, &oy, &oz, &dx, &dy, &dz, &maxd ) )
	{
		return nullptr;
	}
	BridgeState& st = State();
	bool prof = st.pyProfileNative;
	uint64_t t0 = prof ? GetTicks() : 0;
	const World* w = ActiveWorld();
	float t;
	Vec3 p, n;
	int sid;
	bool hit = w != nullptr && w->Raycast( { ox, oy, oz }, { dx, dy, dz }, maxd, &t, &p, &n, &sid );
	if ( prof )
	{
		st.pyNativeStats.rayCalls += 1;
		st.pyNativeStats.rayMs += GetMilliseconds( t0 );
	}
	if ( hit == false )
	{
		Py_RETURN_NONE;
	}
	return Py_BuildValue( "(ffffffff)", t, p.x, p.y, p.z, n.x, n.y, n.z, (float)sid );
}

inline PyObject* EngineSpherePush( PyObject*, PyObject* args )
{
	float x, y, z, r;
	if ( !PyArg_ParseTuple( args, "ffff", &x, &y, &z, &r ) )
	{
		return nullptr;
	}
	BridgeState& st = State();
	bool prof = st.pyProfileNative;
	uint64_t t0 = prof ? GetTicks() : 0;
	Vec3 c = { x, y, z }, n = { 0.0f, 1.0f, 0.0f };
	int hit = 0;
	if ( const World* w = ActiveWorld() )
	{
		hit = w->SpherePush( &c, r, &n ) ? 1 : 0;
	}
	if ( prof )
	{
		st.pyNativeStats.pushCalls += 1;
		st.pyNativeStats.pushMs += GetMilliseconds( t0 );
	}
	return Py_BuildValue( "(fffifff)", c.x, c.y, c.z, hit, n.x, n.y, n.z );
}

inline PyObject* EngineTerrainHeight( PyObject*, PyObject* args )
{
	float x, z;
	if ( !PyArg_ParseTuple( args, "ff", &x, &z ) )
	{
		return nullptr;
	}
	BridgeState& st = State();
	bool prof = st.pyProfileNative;
	uint64_t t0 = prof ? GetTicks() : 0;
	const World* w = ActiveWorld();
	double h = w ? w->Height( x, z ) : 0.0;
	if ( prof )
	{
		st.pyNativeStats.heightCalls += 1;
		st.pyNativeStats.heightMs += GetMilliseconds( t0 );
	}
	return PyFloat_FromDouble( h );
}

inline PyMethodDef* EngineMethods()
{
	static PyMethodDef methods[] = {
		{ "raycast", EngineRaycast, METH_VARARGS, "raycast(ox,oy,oz, dx,dy,dz, maxd) -> (t,px,py,pz,nx,ny,nz,surface) | None" },
		{ "sphere_push", EngineSpherePush, METH_VARARGS, "sphere_push(x,y,z,r) -> (x,y,z,hit,nx,ny,nz)" },
		{ "terrain_height", EngineTerrainHeight, METH_VARARGS, "heightfield height at x,z" },
		{ nullptr, nullptr, 0, nullptr } };
	return methods;
}

inline PyObject* InitEngineModule()
{
	static PyModuleDef def = { PyModuleDef_HEAD_INIT, "engine", "native world queries from the C++ host", -1, EngineMethods() };
	return PyModule_Create( &def );
}

inline void CapturePyError( const char* where )
{
	BridgeState& s = State();
	s.pyError = true;
	std::string text = std::string( where ) + ": ";
	if ( PyErr_Occurred() )
	{
#if PY_VERSION_HEX >= 0x030C0000
		PyObject* type = nullptr;
		PyObject* value = PyErr_GetRaisedException();
		PyObject* tb = nullptr;
		PyObject* mod = PyImport_ImportModule( "traceback" );
		PyObject* lines = mod && value ? PyObject_CallMethod( mod, "format_exception", "O", value ) : nullptr;
#else
		PyObject *type = nullptr, *value = nullptr, *tb = nullptr;
		PyErr_Fetch( &type, &value, &tb );
		PyErr_NormalizeException( &type, &value, &tb );
		PyObject* mod = PyImport_ImportModule( "traceback" );
		PyObject* lines = mod ? PyObject_CallMethod( mod, "format_exception", "OOO", type ? type : Py_None,
													 value ? value : Py_None, tb ? tb : Py_None )
							  : nullptr;
#endif
		if ( lines )
		{
			for ( Py_ssize_t i = 0; i < PyList_Size( lines ); ++i )
			{
				const char* l = PyUnicode_AsUTF8( PyList_GetItem( lines, i ) );
				text += l ? l : "";
			}
		}
		else
		{
			text += "(no traceback)";
		}
		PyErr_Clear();
		Py_XDECREF( lines );
		Py_XDECREF( mod );
		Py_XDECREF( type );
		Py_XDECREF( value );
		Py_XDECREF( tb );
	}
	s.pyLastError = text;
	fprintf( stderr, "[gaitcore] %s\n", text.c_str() );
}

inline int CopyPyBytes( PyObject* res, float* buf, int maxFloats, const char* where )
{
	if ( res == nullptr )
	{
		CapturePyError( where );
		return 0;
	}
	int n = 0;
	if ( PyBytes_Check( res ) )
	{
		Py_ssize_t count = PyBytes_GET_SIZE( res ) / 4;
		n = (int)( count < maxFloats ? count : maxFloats );
		memcpy( buf, PyBytes_AS_STRING( res ), (size_t)n * 4 );
	}
	Py_DECREF( res );
	return n;
}

inline bool FileExists( const std::string& path )
{
	std::ifstream f( path.c_str() );
	return f.good();
}

inline std::string FindIkDir()
{
	std::vector<std::string> candidates;
	if ( const char* env = getenv( "MECH_IK_DIR" ) )
	{
		candidates.push_back( env );
	}
#if defined( MECH_IK_DIR )
	candidates.push_back( MECH_IK_DIR );
#endif
	candidates.push_back( "IK/python" );
	candidates.push_back( "../IK/python" );
	candidates.push_back( "../../IK/python" );
	candidates.push_back( "../../../IK/python" );
	for ( const std::string& c : candidates )
	{
		if ( FileExists( c + "/gaitcore/api.py" ) )
		{
			return c;
		}
	}
	return std::string();
}

inline bool InitPython()
{
	BridgeState& s = State();
	if ( s.pyTried )
	{
		return s.pyReady;
	}
	s.pyTried = true;
	s.ikDir = FindIkDir();
	if ( s.ikDir.empty() )
	{
		s.pyInitMessage = "could not find IK/python/gaitcore/api.py";
		s.pyLastError = s.pyInitMessage;
		return false;
	}

	PyImport_AppendInittab( "engine", InitEngineModule );
	PyConfig cfg;
	PyConfig_InitPythonConfig( &cfg );
	cfg.parse_argv = 0;
	cfg.install_signal_handlers = 0;
	cfg.write_bytecode = 0;
#if defined( MECH_PY_HOME )
	PyConfig_SetBytesString( &cfg, &cfg.home, MECH_PY_HOME );
#endif
	PyStatus st = Py_InitializeFromConfig( &cfg );
	PyConfig_Clear( &cfg );
	if ( PyStatus_Exception( st ) )
	{
		s.pyInitMessage = std::string( "Python init failed: " ) + ( st.err_msg ? st.err_msg : "?" );
		s.pyLastError = s.pyInitMessage;
		return false;
	}
	PyObject* path = PySys_GetObject( "path" );
	PyObject* dir = PyUnicode_DecodeFSDefault( s.ikDir.c_str() );
	PyList_Insert( path, 0, dir );
	Py_DECREF( dir );

	s.pyApi = PyImport_ImportModule( "gaitcore.api" );
	if ( s.pyApi == nullptr )
	{
		CapturePyError( "import gaitcore.api" );
		s.pyInitMessage = "could not import gaitcore.api from " + s.ikDir;
		return false;
	}
	PyObject* r = PyObject_CallMethod( s.pyApi, "init", nullptr );
	if ( r == nullptr )
	{
		CapturePyError( "gaitcore.api.init" );
		return false;
	}
	const char* msg = PyUnicode_AsUTF8( r );
	s.pyInitMessage = msg ? msg : "ok";
	Py_DECREF( r );
	s.pyReady = true;
	return true;
}

#endif 

inline bool InitC()
{
	BridgeState& s = State();
	if ( s.cCtx == nullptr )
	{
		s.cCtx = gc_create( s.cThreads );
		if ( s.cCtx == nullptr )
		{
			s.cReady = false;
			s.cInitMsg = "Failed to allocate C gaitcore context";
			return false;
		}
	}
	GcWorldCallbacks cb = MakeWorldCallbacks();
	gc_set_world( s.cCtx, ActiveWorld() ? &cb : nullptr );
	char buf[128];
	snprintf( buf, sizeof( buf ), "gaitcore_c ready (%d worker threads, %d mech types)", gc_get_threads( s.cCtx ),
			  gc_kind_count() );
	s.cInitMsg = buf;
	s.cReady = true;
	return true;
}

} 

inline bool HasPythonBackend()
{
#if defined( MECH_HAVE_PYTHON )
	return true;
#else
	return false;
#endif
}

inline Backend ActiveBackend()
{
	return detail::State().backend;
}

inline const char* BackendName( Backend b )
{
	return b == BackendPython ? "Python" : "C (Native)";
}

inline void SetBackend( Backend b )
{
	detail::BridgeState& s = detail::State();
	if ( b == BackendPython && HasPythonBackend() == false )
	{
		s.backend = BackendC;
		return;
	}
	s.backend = b;
}

inline int HardwareThreads()
{
	return gc_hardware_threads();
}

inline int WorkerThreads()
{
	detail::BridgeState& s = detail::State();
	if ( s.cCtx )
	{
		return gc_get_threads( s.cCtx );
	}
	return s.cThreads > 0 ? s.cThreads : gc_hardware_threads();
}

inline void SetWorkerThreads( int n )
{
	detail::BridgeState& s = detail::State();
	int hw = gc_hardware_threads();
	int clamped = ClampInt( n, 1, hw );
	s.cThreads = clamped;
	if ( s.cCtx )
	{
		gc_set_threads( s.cCtx, clamped );
		char buf[128];
		snprintf( buf, sizeof( buf ), "gaitcore_c ready (%d worker threads, %d mech types)", gc_get_threads( s.cCtx ),
				  gc_kind_count() );
		s.cInitMsg = buf;
	}
}

inline float LastCpuMs()
{
	detail::BridgeState& s = detail::State();
	if ( s.backend == BackendC && s.cCtx )
	{
		return gc_last_cpu_ms( s.cCtx );
	}
	return 0.0f;
}

inline bool Init()
{
	if ( ActiveBackend() == BackendC )
	{
		return detail::InitC();
	}
#if defined( MECH_HAVE_PYTHON )
	return detail::InitPython();
#else
	return false;
#endif
}

inline bool Ready()
{
	const detail::BridgeState& s = detail::State();
	return s.backend == BackendC ? s.cReady : s.pyReady;
}

inline bool HasError()
{
	const detail::BridgeState& s = detail::State();
	if ( s.backend == BackendC )
	{
		return s.cReady == false;
	}
	return s.pyError || s.pyReady == false;
}

inline const std::string& LastError()
{
	const detail::BridgeState& s = detail::State();
	if ( s.backend == BackendC )
	{
		static std::string empty;
		return s.cReady ? empty : s.cInitMsg;
	}
	return s.pyLastError;
}

inline const std::string& InitMessage()
{
	const detail::BridgeState& s = detail::State();
	return s.backend == BackendC ? s.cInitMsg : s.pyInitMessage;
}

inline const std::string& IkDir()
{
	const detail::BridgeState& s = detail::State();
	if ( s.backend == BackendC )
	{
		static std::string cdir = "IK/c";
		return cdir;
	}
	return s.ikDir;
}

inline bool Reset( bool reloadCode )
{
	if ( Init() == false )
	{
		return false;
	}
	detail::BridgeState& s = detail::State();
	if ( s.backend == BackendC )
	{
		gc_reset( s.cCtx );
		GcWorldCallbacks cb = detail::MakeWorldCallbacks();
		gc_set_world( s.cCtx, ActiveWorld() ? &cb : nullptr );
		return true;
	}
#if defined( MECH_HAVE_PYTHON )
	PyObject* r = PyObject_CallMethod( s.pyApi, "reset", "i", reloadCode ? 1 : 0 );
	if ( r == nullptr )
	{
		detail::CapturePyError( "gaitcore.api.reset" );
		return false;
	}
	Py_DECREF( r );
	s.pyError = false;
	s.pyLastError.clear();
	return true;
#else
	(void)reloadCode;
	return false;
#endif
}

inline int KindCount()
{
	if ( Ready() == false )
	{
		return 0;
	}
	if ( ActiveBackend() == BackendC )
	{
		return gc_kind_count();
	}
#if defined( MECH_HAVE_PYTHON )
	PyObject* r = PyObject_CallMethod( detail::State().pyApi, "kinds", nullptr );
	if ( r == nullptr )
	{
		detail::CapturePyError( "kinds" );
		return 0;
	}
	int n = (int)PyList_Size( r );
	Py_DECREF( r );
	return n;
#else
	return 0;
#endif
}

inline std::vector<std::string> KindNames()
{
	std::vector<std::string> names;
	if ( Ready() == false )
	{
		return names;
	}
	if ( ActiveBackend() == BackendC )
	{
		int n = gc_kind_count();
		for ( int i = 0; i < n; ++i )
		{
			names.push_back( gc_kind_name( i ) );
		}
		return names;
	}
#if defined( MECH_HAVE_PYTHON )
	PyObject* r = PyObject_CallMethod( detail::State().pyApi, "kinds", nullptr );
	if ( r == nullptr )
	{
		detail::CapturePyError( "kinds" );
		return names;
	}
	for ( Py_ssize_t i = 0; i < PyList_Size( r ); ++i )
	{
		const char* s = PyUnicode_AsUTF8( PyList_GetItem( r, i ) );
		names.push_back( s ? s : "?" );
	}
	Py_DECREF( r );
#endif
	return names;
}

inline int Spawn( int kind, Vec3 p, float yaw )
{
	if ( Ready() == false )
	{
		return -1;
	}
	if ( ActiveBackend() == BackendC )
	{
		return gc_spawn( detail::State().cCtx, kind, p.x, p.y, p.z, yaw );
	}
#if defined( MECH_HAVE_PYTHON )
	PyObject* r = PyObject_CallMethod( detail::State().pyApi, "spawn", "iffff", kind, p.x, p.y, p.z, yaw );
	if ( r == nullptr )
	{
		detail::CapturePyError( "spawn" );
		return -1;
	}
	int h = (int)PyLong_AsLong( r );
	Py_DECREF( r );
	return h;
#else
	return -1;
#endif
}

inline bool Respawn( int h, int kind, Vec3 p, float yaw )
{
	if ( Ready() == false )
	{
		return false;
	}
	if ( ActiveBackend() == BackendC )
	{
		return gc_respawn( detail::State().cCtx, h, kind, p.x, p.y, p.z, yaw ) >= 0;
	}
#if defined( MECH_HAVE_PYTHON )
	PyObject* r = PyObject_CallMethod( detail::State().pyApi, "respawn", "iiffff", h, kind, p.x, p.y, p.z, yaw );
	if ( r == nullptr )
	{
		detail::CapturePyError( "respawn" );
		return false;
	}
	Py_DECREF( r );
	return true;
#else
	return false;
#endif
}

inline bool Info( int h, MechInfo* o )
{
	if ( Ready() == false )
	{
		return false;
	}
	if ( ActiveBackend() == BackendC )
	{
		GcMechInfo gi;
		if ( gc_info( detail::State().cCtx, h, &gi ) == 0 )
		{
			return false;
		}
		o->name = gi.name ? gi.name : "?";
		o->blurb = gi.blurb ? gi.blurb : "";
		o->kind = gi.kind;
		o->legs = gi.legs;
		o->jointsPerLeg = gi.joints_per_leg;
		o->half = { gi.half[0], gi.half[1], gi.half[2] };
		o->widthCount = gi.width_count;
		for ( int i = 0; i < o->widthCount && i < 4; ++i )
		{
			o->widths[i] = gi.widths[i];
		}
		o->mass = gi.mass;
		o->climber = gi.climber != 0;
		o->faction = gi.faction;
		o->wireMax = gi.wire_max;
		o->walkSpeed = gi.walk_speed;
		o->runSpeed = gi.run_speed;
		o->floats = gi.floats;
		return true;
	}
#if defined( MECH_HAVE_PYTHON )
	PyObject* api = detail::State().pyApi;
	PyObject* r = PyObject_CallMethod( api, "info", "i", h );
	if ( r == nullptr )
	{
		detail::CapturePyError( "info" );
		return false;
	}
	const char* name = PyUnicode_AsUTF8( PyTuple_GetItem( r, 0 ) );
	o->name = name ? name : "?";
	o->kind = (int)PyLong_AsLong( PyTuple_GetItem( r, 1 ) );
	o->legs = (int)PyLong_AsLong( PyTuple_GetItem( r, 2 ) );
	o->jointsPerLeg = (int)PyLong_AsLong( PyTuple_GetItem( r, 3 ) );
	o->half = { (float)PyFloat_AsDouble( PyTuple_GetItem( r, 4 ) ), (float)PyFloat_AsDouble( PyTuple_GetItem( r, 5 ) ),
				(float)PyFloat_AsDouble( PyTuple_GetItem( r, 6 ) ) };
	PyObject* w = PyTuple_GetItem( r, 7 );
	o->widthCount = (int)PyTuple_Size( w );
	if ( o->widthCount > 4 )
	{
		o->widthCount = 4;
	}
	for ( int i = 0; i < o->widthCount; ++i )
	{
		o->widths[i] = (float)PyFloat_AsDouble( PyTuple_GetItem( w, i ) );
	}
	o->mass = (float)PyFloat_AsDouble( PyTuple_GetItem( r, 8 ) );
	const char* blurb = PyUnicode_AsUTF8( PyTuple_GetItem( r, 9 ) );
	o->blurb = blurb ? blurb : "";
	o->climber = PyLong_AsLong( PyTuple_GetItem( r, 10 ) ) != 0;
	if ( PyTuple_Size( r ) >= 13 )
	{
		o->faction = (int)PyLong_AsLong( PyTuple_GetItem( r, 11 ) );
		o->wireMax = (float)PyFloat_AsDouble( PyTuple_GetItem( r, 12 ) );
	}
	if ( PyTuple_Size( r ) >= 15 )
	{
		o->walkSpeed = (float)PyFloat_AsDouble( PyTuple_GetItem( r, 13 ) );
		o->runSpeed = (float)PyFloat_AsDouble( PyTuple_GetItem( r, 14 ) );
	}
	Py_DECREF( r );
	PyObject* fl = PyObject_CallMethod( api, "floats_per_mech", "i", h );
	if ( fl == nullptr )
	{
		detail::CapturePyError( "floats_per_mech" );
		return false;
	}
	o->floats = (int)PyLong_AsLong( fl );
	Py_DECREF( fl );
	if ( PyErr_Occurred() )
	{
		detail::CapturePyError( "info" );
		return false;
	}
	return true;
#else
	(void)o;
	return false;
#endif
}

inline bool Step( float dt, const float* inputs, int mechCount )
{
	if ( Ready() == false )
	{
		return false;
	}
	if ( ActiveBackend() == BackendC )
	{
		GcWorldCallbacks cb = detail::MakeWorldCallbacks();
		gc_set_world( detail::State().cCtx, ActiveWorld() ? &cb : nullptr );
		gc_step( detail::State().cCtx, dt, inputs, mechCount );
		return true;
	}
#if defined( MECH_HAVE_PYTHON )
	PyObject* b = PyBytes_FromStringAndSize( (const char*)inputs, (Py_ssize_t)mechCount * kInputStride * 4 );
	PyObject* r = PyObject_CallMethod( detail::State().pyApi, "step", "fO", dt, b );
	Py_DECREF( b );
	if ( r == nullptr )
	{
		detail::CapturePyError( "step" );
		return false;
	}
	Py_DECREF( r );
	return true;
#else
	(void)dt;
	(void)inputs;
	(void)mechCount;
	return false;
#endif
}

inline int Poses( float* buf, int maxFloats )
{
	if ( Ready() == false )
	{
		return 0;
	}
	if ( ActiveBackend() == BackendC )
	{
		return gc_poses( detail::State().cCtx, buf, maxFloats );
	}
#if defined( MECH_HAVE_PYTHON )
	return detail::CopyPyBytes( PyObject_CallMethod( detail::State().pyApi, "poses", nullptr ), buf, maxFloats, "poses" );
#else
	(void)buf;
	(void)maxFloats;
	return 0;
#endif
}

inline int Events( float* buf, int maxFloats )
{
	if ( Ready() == false )
	{
		return 0;
	}
	if ( ActiveBackend() == BackendC )
	{
		return gc_events( detail::State().cCtx, buf, maxFloats );
	}
#if defined( MECH_HAVE_PYTHON )
	return detail::CopyPyBytes( PyObject_CallMethod( detail::State().pyApi, "events", nullptr ), buf, maxFloats,
								"events" );
#else
	(void)buf;
	(void)maxFloats;
	return 0;
#endif
}

inline int DebugLines( int h, float* buf, int maxFloats )
{
	if ( Ready() == false )
	{
		return 0;
	}
	if ( ActiveBackend() == BackendC )
	{
		return gc_debug_lines( detail::State().cCtx, h, buf, maxFloats );
	}
#if defined( MECH_HAVE_PYTHON )
	return detail::CopyPyBytes( PyObject_CallMethod( detail::State().pyApi, "debug_lines", "i", h ), buf, maxFloats,
								"debug_lines" );
#else
	(void)h;
	(void)buf;
	(void)maxFloats;
	return 0;
#endif
}

inline void SetDebug( bool on )
{
	if ( Ready() == false )
	{
		return;
	}
	if ( ActiveBackend() == BackendC )
	{
		gc_set_debug( detail::State().cCtx, on ? 1 : 0 );
		return;
	}
#if defined( MECH_HAVE_PYTHON )
	PyObject* r = PyObject_CallMethod( detail::State().pyApi, "set_debug", "i", on ? 1 : 0 );
	if ( r == nullptr )
	{
		detail::CapturePyError( "set_debug" );
		return;
	}
	Py_DECREF( r );
#else
	(void)on;
#endif
}

inline bool Stats( int h, MechStats* o )
{
	if ( Ready() == false )
	{
		return false;
	}
	if ( ActiveBackend() == BackendC )
	{
		GcMechStats gs;
		if ( gc_stats( detail::State().cCtx, h, &gs ) == 0 )
		{
			return false;
		}
		o->mode = gs.mode;
		o->gait = gs.gait;
		o->gaitOverride = gs.gait_override != 0;
		o->state = gs.state;
		o->duty = gs.duty;
		o->cycle = gs.cycle;
		o->maxStretch = gs.max_stretch;
		o->simMs = gs.sim_ms;
		o->rays = gs.rays;
		o->climbEnabled = gs.climb_enabled != 0;
		o->crouchToggle = gs.crouch_toggle != 0;
		return true;
	}
#if defined( MECH_HAVE_PYTHON )
	PyObject* r = PyObject_CallMethod( detail::State().pyApi, "stats", "i", h );
	if ( r == nullptr )
	{
		detail::CapturePyError( "stats" );
		return false;
	}
	auto str = [&]( int i ) {
		const char* c = PyUnicode_AsUTF8( PyTuple_GetItem( r, i ) );
		return std::string( c ? c : "" );
	};
	auto num = [&]( int i ) { return (float)PyFloat_AsDouble( PyTuple_GetItem( r, i ) ); };
	o->mode = str( 0 );
	o->gait = str( 1 );
	o->gaitOverride = PyLong_AsLong( PyTuple_GetItem( r, 2 ) ) != 0;
	o->state = str( 3 );
	o->duty = num( 4 );
	o->cycle = num( 5 );
	o->maxStretch = num( 6 );
	o->simMs = num( 7 );
	o->rays = (int)PyLong_AsLong( PyTuple_GetItem( r, 8 ) );
	o->climbEnabled = PyLong_AsLong( PyTuple_GetItem( r, 9 ) ) != 0;
	o->crouchToggle = PyLong_AsLong( PyTuple_GetItem( r, 10 ) ) != 0;
	Py_DECREF( r );
	if ( PyErr_Occurred() )
	{
		detail::CapturePyError( "stats" );
		return false;
	}
	return true;
#else
	(void)h;
	(void)o;
	return false;
#endif
}

inline bool HotReload()
{
	if ( Ready() == false || ActiveBackend() != BackendPython )
	{
		return false;
	}
#if defined( MECH_HAVE_PYTHON )
	PyObject* r = PyObject_CallMethod( detail::State().pyApi, "hot_reload", nullptr );
	if ( r == nullptr )
	{
		detail::CapturePyError( "hot_reload" );
		return false;
	}
	bool ok = PyObject_IsTrue( r ) == 1;
	Py_DECREF( r );
	detail::State().pyError = !ok;
	detail::State().pyLastError = ok ? "" : "hot_reload failed: traceback printed to the console";
	return ok;
#else
	return false;
#endif
}

inline void SetNativeProfile( bool on )
{
	detail::BridgeState& s = detail::State();
	s.pyProfileNative = on;
	s.pyNativeStats = NativeQueryStats();
	if ( s.cCtx )
	{
		gc_reset_query_stats( s.cCtx );
	}
}

inline void ResetNativeQueryStats()
{
	detail::BridgeState& s = detail::State();
	s.pyNativeStats = NativeQueryStats();
	if ( s.cCtx )
	{
		gc_reset_query_stats( s.cCtx );
	}
}

inline NativeQueryStats GetNativeQueryStats()
{
	const detail::BridgeState& s = detail::State();
	if ( s.backend == BackendC && s.cCtx )
	{
		GcQueryStats gs = gc_get_query_stats( s.cCtx );
		NativeQueryStats out;
		out.rayCalls = gs.ray_calls;
		out.pushCalls = gs.push_calls;
		out.heightCalls = gs.height_calls;
		out.rayMs = gs.ray_ms;
		out.pushMs = gs.push_ms;
		out.heightMs = gs.height_ms;
		return out;
	}
	return s.pyNativeStats;
}

inline int DespawnTo( int n )
{
	if ( Ready() == false )
	{
		return 0;
	}
	if ( ActiveBackend() == BackendC )
	{
		return gc_despawn_to( detail::State().cCtx, n );
	}
#if defined( MECH_HAVE_PYTHON )
	PyObject* r = PyObject_CallMethod( detail::State().pyApi, "despawn_to", "i", n );
	if ( r == nullptr )
	{
		detail::CapturePyError( "despawn_to" );
		return 0;
	}
	int rem = (int)PyLong_AsLong( r );
	Py_DECREF( r );
	return rem;
#else
	(void)n;
	return 0;
#endif
}

inline bool SetBenchConfig( float maxSubstep, float legIkHz, float probeHz, float speedScale, bool planeRoll, bool profile )
{
	if ( Ready() == false )
	{
		return false;
	}
	if ( ActiveBackend() == BackendC )
	{
		GcBenchConfig cfg;
		cfg.max_substep = maxSubstep;
		cfg.leg_ik_hz = legIkHz;
		cfg.probe_hz = probeHz;
		cfg.speed_scale = speedScale;
		cfg.plane_roll = planeRoll ? 1 : 0;
		cfg.profile = profile ? 1 : 0;
		gc_set_bench_config( detail::State().cCtx, &cfg );
		return true;
	}
#if defined( MECH_HAVE_PYTHON )
	PyObject* r = PyObject_CallMethod( detail::State().pyApi, "set_bench_config", "ffffii", maxSubstep, legIkHz, probeHz,
									   speedScale, planeRoll ? 1 : 0, profile ? 1 : 0 );
	if ( r == nullptr )
	{
		detail::CapturePyError( "set_bench_config" );
		return false;
	}
	Py_DECREF( r );
	return true;
#else
	(void)maxSubstep;
	(void)legIkHz;
	(void)probeHz;
	(void)speedScale;
	(void)planeRoll;
	(void)profile;
	return false;
#endif
}

inline int ProfileData( float* buf, int maxFloats )
{
	if ( Ready() == false )
	{
		return 0;
	}
	if ( ActiveBackend() == BackendC )
	{
		return gc_profile_data( detail::State().cCtx, buf, maxFloats );
	}
#if defined( MECH_HAVE_PYTHON )
	return detail::CopyPyBytes( PyObject_CallMethod( detail::State().pyApi, "profile_data", nullptr ), buf, maxFloats,
								"profile_data" );
#else
	(void)buf;
	(void)maxFloats;
	return 0;
#endif
}

} 
