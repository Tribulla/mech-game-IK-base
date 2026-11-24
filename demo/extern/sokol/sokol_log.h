#if defined(SOKOL_IMPL) && !defined(SOKOL_LOG_IMPL)
#define SOKOL_LOG_IMPL
#endif
#ifndef SOKOL_LOG_INCLUDED

#define SOKOL_LOG_INCLUDED (1)
#include <stdint.h>

#if defined(SOKOL_API_DECL) && !defined(SOKOL_LOG_API_DECL)
#define SOKOL_LOG_API_DECL SOKOL_API_DECL
#endif
#ifndef SOKOL_LOG_API_DECL
#if defined(_WIN32) && defined(SOKOL_DLL) && defined(SOKOL_LOG_IMPL)
#define SOKOL_LOG_API_DECL __declspec(dllexport)
#elif defined(_WIN32) && defined(SOKOL_DLL)
#define SOKOL_LOG_API_DECL __declspec(dllimport)
#else
#define SOKOL_LOG_API_DECL extern
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

SOKOL_LOG_API_DECL void slog_func(const char* tag, uint32_t log_level, uint32_t log_item, const char* message, uint32_t line_nr, const char* filename, void* user_data);

#ifdef __cplusplus
} 
#endif
#endif 

#ifdef SOKOL_LOG_IMPL
#define SOKOL_LOG_IMPL_INCLUDED (1)

#ifndef SOKOL_API_IMPL
    #define SOKOL_API_IMPL
#endif
#ifndef SOKOL_DEBUG
    #ifndef NDEBUG
        #define SOKOL_DEBUG
    #endif
#endif
#ifndef SOKOL_ASSERT
    #include <assert.h>
    #define SOKOL_ASSERT(c) assert(c)
#endif

#ifndef _SOKOL_PRIVATE
    #if defined(__GNUC__) || defined(__clang__)
        #define _SOKOL_PRIVATE __attribute__((unused)) static
    #else
        #define _SOKOL_PRIVATE static
    #endif
#endif

#ifndef _SOKOL_UNUSED
    #define _SOKOL_UNUSED(x) (void)(x)
#endif

#if defined(__APPLE__)
    #define _SLOG_APPLE (1)
#elif defined(__EMSCRIPTEN__)
    #define _SLOG_EMSCRIPTEN (1)
#elif defined(_WIN32)
    #define _SLOG_WINDOWS (1)
#elif defined(__ANDROID__)
    #define _SLOG_ANDROID (1)
#elif defined(__linux__) || defined(__unix__)
    #define _SLOG_LINUX (1)
#else
#error "sokol_log.h: unknown platform"
#endif

#include <stdlib.h> 
#include <stdio.h>  
#include <stddef.h> 

#if defined(_SLOG_EMSCRIPTEN)
#include <emscripten/emscripten.h>
#elif defined(_SLOG_WINDOWS)
#ifndef WIN32_LEAN_AND_MEAN
    #define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
    #define NOMINMAX
#endif
#include <windows.h>
#elif defined(_SLOG_ANDROID)
#include <android/log.h>
#elif defined(_SLOG_LINUX) || defined(_SLOG_APPLE)
#include <syslog.h>
#endif

#define _SLOG_LINE_LENGTH (512)

_SOKOL_PRIVATE char* _slog_append(const char* str, char* dst, char* end) {
    if (str) {
        char c;
        while (((c = *str++) != 0) && (dst < (end - 1))) {
            *dst++ = c;
        }
    }
    *dst = 0;
    return dst;
}

_SOKOL_PRIVATE char* _slog_itoa(uint32_t x, char* buf, size_t buf_size) {
    const size_t max_digits_and_null = 11;
    if (buf_size < max_digits_and_null) {
        return 0;
    }
    char* p = buf + max_digits_and_null;
    *--p = 0;
    do {
        *--p = '0' + (x % 10);
        x /= 10;
    } while (x != 0);
    return p;
}

#if defined(_SLOG_EMSCRIPTEN)
EM_JS(void, slog_js_log, (uint32_t level, const char* c_str), {
    const str = UTF8ToString(c_str);
    switch (level) {
        case 0: console.error(str); break;
        case 1: console.error(str); break;
        case 2: console.warn(str); break;
        default: console.info(str); break;
    }
})
#endif

SOKOL_API_IMPL void slog_func(const char* tag, uint32_t log_level, uint32_t log_item, const char* message, uint32_t line_nr, const char* filename, void* user_data) {
    _SOKOL_UNUSED(user_data);

    const char* log_level_str;
    switch (log_level) {
        case 0: log_level_str = "panic"; break;
        case 1: log_level_str = "error"; break;
        case 2: log_level_str = "warning"; break;
        default: log_level_str = "info"; break;
    }

    char line_buf[_SLOG_LINE_LENGTH];
    char* str = line_buf;
    char* end = line_buf + sizeof(line_buf);
    char num_buf[32];
    if (tag) {
        str = _slog_append("[", str, end);
        str = _slog_append(tag, str, end);
        str = _slog_append("]", str, end);
    }
    str = _slog_append("[", str, end);
    str = _slog_append(log_level_str, str, end);
    str = _slog_append("]", str, end);
    str = _slog_append("[id:", str, end);
    str = _slog_append(_slog_itoa(log_item, num_buf, sizeof(num_buf)), str, end);
    str = _slog_append("]", str, end);

    if (filename) {
        str = _slog_append(" ", str, end);
        #if defined(_MSC_VER)

            str = _slog_append(filename, str, end);
            str = _slog_append("(", str, end);
            str = _slog_append(_slog_itoa(line_nr, num_buf, sizeof(num_buf)), str, end);
            str = _slog_append("): ", str, end);
        #else

            str = _slog_append(filename, str, end);
            str = _slog_append(":", str, end);
            str = _slog_append(_slog_itoa(line_nr, num_buf, sizeof(num_buf)), str, end);
            str = _slog_append(":0: ", str, end);
        #endif
    }
    else {
        str = _slog_append("[line:", str, end);
        str = _slog_append(_slog_itoa(line_nr, num_buf, sizeof(num_buf)), str, end);
        str = _slog_append("] ", str, end);
    }
    if (message) {
        str = _slog_append("\n\t", str, end);
        str = _slog_append(message, str, end);
    }
    str = _slog_append("\n\n", str, end);
    if (0 == log_level) {
        str = _slog_append("ABORTING because of [panic]\n", str, end);
        (void)str;
    }

    #if defined(_SLOG_LINUX) || defined(_SLOG_WINDOWS) || defined(_SLOG_APPLE)
        fputs(line_buf, stderr);
    #endif

    #if defined(_SLOG_WINDOWS)
        OutputDebugStringA(line_buf);
    #elif defined(_SLOG_ANDROID)
        int prio;
        switch (log_level) {
            case 0: prio = ANDROID_LOG_FATAL; break;
            case 1: prio = ANDROID_LOG_ERROR; break;
            case 2: prio = ANDROID_LOG_WARN; break;
            default: prio = ANDROID_LOG_INFO; break;
        }
        __android_log_write(prio, "SOKOL", line_buf);
    #elif defined(_SLOG_EMSCRIPTEN)
        slog_js_log(log_level, line_buf);
    #endif
    if (0 == log_level) {
        abort();
    }
}
#endif 
