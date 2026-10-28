/// @file
/// Selects the OpenGL API for the platform: OpenGL ES 3.0 on iOS, desktop OpenGL 3.3
/// core through the glad loader elsewhere.
///
/// Shared GL code (map_renderer.h) includes only this file and uses only calls that
/// exist in both APIs; VRST_GLES tells it where the two differ. On desktop this file
/// must be included before GLFW, because glad has to come first.
#pragma once

#if defined(__APPLE__)
#include <TargetConditionals.h>
#endif

#if defined(__APPLE__) && TARGET_OS_IPHONE
#ifndef GLES_SILENCE_DEPRECATION
#define GLES_SILENCE_DEPRECATION  // Apple deprecated OpenGL ES; it still works on iOS
#endif
#include <OpenGLES/ES3/gl.h>
/// 1 when compiling against OpenGL ES 3.0, 0 for desktop OpenGL 3.3 core.
#define VRST_GLES 1
#else
#include <glad/glad.h>
/// 1 when compiling against OpenGL ES 3.0, 0 for desktop OpenGL 3.3 core.
#define VRST_GLES 0
#endif
