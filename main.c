// Copyright 2019-2021, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief OpenXR playground, exercising many areas of the OpenXR API.
 * Advanced version of the openxr-simple-example
 * @author Christoph Haag <christoph.haag@collabora.com>
 */

#include <stdio.h>
#include <stdbool.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

#ifdef __linux__

// Required headers for OpenGL rendering, as well as for including openxr_platform
#define GL_GLEXT_PROTOTYPES
#define GL3_PROTOTYPES
#include <GL/gl.h>
#include <GL/glext.h>

// Required headers for windowing, as well as the XrGraphicsBindingOpenGLXlibKHR struct.
#include <X11/Xlib.h>
#include <GL/glx.h>

#define XR_USE_PLATFORM_XLIB
#define XR_USE_GRAPHICS_API_OPENGL
#include "openxr_headers/openxr.h"
#include "openxr_headers/openxr_platform.h"
#include "openxr_headers/openxr_reflection.h"

#else
#error Only Linux/XLib supported for now
#endif

/*
This file contains expansion macros (X Macros) for OpenXR enumerations and structures.
Example of how to use expansion macros to make an enum-to-string function:
*/

#define XR_ENUM_CASE_STR(name, val)                                                                \
	case name:                                                                                       \
		return #name;
#define XR_ENUM_STR(enumType)                                                                      \
	const char* XrStr_##enumType(uint64_t e)                                                         \
	{                                                                                                \
		switch (e) {                                                                                   \
			XR_LIST_ENUM_##enumType(XR_ENUM_CASE_STR) default : return "Unknown";                        \
		}                                                                                              \
	}

XR_ENUM_STR(XrResult)
XR_ENUM_STR(XrFormFactor)
XR_ENUM_STR(XrReferenceSpaceType)
XR_ENUM_STR(XrViewConfigurationType)

typedef const char* (*XrStr_fn)(long value);

#include <SDL2/SDL.h>
#include <SDL2/SDL_events.h>

#define degrees_to_radians(angle_degrees) ((angle_degrees)*M_PI / 180.0)
#define radians_to_degrees(angle_radians) ((angle_radians)*180.0 / M_PI)

// we need an identity pose for creating spaces without offsets
static XrPosef identity_pose = {.orientation = {.x = 0, .y = 0, .z = 0, .w = 1.0},
                                .position = {.x = 0, .y = 0, .z = 0}};

#define HAND_LEFT_INDEX 0
#define HAND_RIGHT_INDEX 1
#define HAND_COUNT 2



// =============================================================================
// math code adapted from
// https://github.com/KhronosGroup/OpenXR-SDK-Source/blob/master/src/common/xr_linear.h
// Copyright (c) 2017 The Khronos Group Inc.
// Copyright (c) 2016 Oculus VR, LLC.
// SPDX-License-Identifier: Apache-2.0
// =============================================================================

typedef enum
{
	GRAPHICS_VULKAN,
	GRAPHICS_OPENGL,
	GRAPHICS_OPENGL_ES
} GraphicsAPI;

typedef struct XrMatrix4x4f
{
	float m[16];
} XrMatrix4x4f;

inline static void
XrMatrix4x4f_CreateProjectionFov(XrMatrix4x4f* result,
                                 GraphicsAPI graphicsApi,
                                 const XrFovf fov,
                                 const float nearZ,
                                 const float farZ)
{
	const float tanAngleLeft = tanf(fov.angleLeft);
	const float tanAngleRight = tanf(fov.angleRight);

	const float tanAngleDown = tanf(fov.angleDown);
	const float tanAngleUp = tanf(fov.angleUp);

	const float tanAngleWidth = tanAngleRight - tanAngleLeft;

	// Set to tanAngleDown - tanAngleUp for a clip space with positive Y
	// down (Vulkan). Set to tanAngleUp - tanAngleDown for a clip space with
	// positive Y up (OpenGL / D3D / Metal).
	const float tanAngleHeight =
	    graphicsApi == GRAPHICS_VULKAN ? (tanAngleDown - tanAngleUp) : (tanAngleUp - tanAngleDown);

	// Set to nearZ for a [-1,1] Z clip space (OpenGL / OpenGL ES).
	// Set to zero for a [0,1] Z clip space (Vulkan / D3D / Metal).
	const float offsetZ =
	    (graphicsApi == GRAPHICS_OPENGL || graphicsApi == GRAPHICS_OPENGL_ES) ? nearZ : 0;

	if (farZ <= nearZ) {
		// place the far plane at infinity
		result->m[0] = 2 / tanAngleWidth;
		result->m[4] = 0;
		result->m[8] = (tanAngleRight + tanAngleLeft) / tanAngleWidth;
		result->m[12] = 0;

		result->m[1] = 0;
		result->m[5] = 2 / tanAngleHeight;
		result->m[9] = (tanAngleUp + tanAngleDown) / tanAngleHeight;
		result->m[13] = 0;

		result->m[2] = 0;
		result->m[6] = 0;
		result->m[10] = -1;
		result->m[14] = -(nearZ + offsetZ);

		result->m[3] = 0;
		result->m[7] = 0;
		result->m[11] = -1;
		result->m[15] = 0;
	} else {
		// normal projection
		result->m[0] = 2 / tanAngleWidth;
		result->m[4] = 0;
		result->m[8] = (tanAngleRight + tanAngleLeft) / tanAngleWidth;
		result->m[12] = 0;

		result->m[1] = 0;
		result->m[5] = 2 / tanAngleHeight;
		result->m[9] = (tanAngleUp + tanAngleDown) / tanAngleHeight;
		result->m[13] = 0;

		result->m[2] = 0;
		result->m[6] = 0;
		result->m[10] = -(farZ + offsetZ) / (farZ - nearZ);
		result->m[14] = -(farZ * (nearZ + offsetZ)) / (farZ - nearZ);

		result->m[3] = 0;
		result->m[7] = 0;
		result->m[11] = -1;
		result->m[15] = 0;
	}
}

inline static void
XrMatrix4x4f_CreateFromQuaternion(XrMatrix4x4f* result, const XrQuaternionf* quat)
{
	const float x2 = quat->x + quat->x;
	const float y2 = quat->y + quat->y;
	const float z2 = quat->z + quat->z;

	const float xx2 = quat->x * x2;
	const float yy2 = quat->y * y2;
	const float zz2 = quat->z * z2;

	const float yz2 = quat->y * z2;
	const float wx2 = quat->w * x2;
	const float xy2 = quat->x * y2;
	const float wz2 = quat->w * z2;
	const float xz2 = quat->x * z2;
	const float wy2 = quat->w * y2;

	result->m[0] = 1.0f - yy2 - zz2;
	result->m[1] = xy2 + wz2;
	result->m[2] = xz2 - wy2;
	result->m[3] = 0.0f;

	result->m[4] = xy2 - wz2;
	result->m[5] = 1.0f - xx2 - zz2;
	result->m[6] = yz2 + wx2;
	result->m[7] = 0.0f;

	result->m[8] = xz2 + wy2;
	result->m[9] = yz2 - wx2;
	result->m[10] = 1.0f - xx2 - yy2;
	result->m[11] = 0.0f;

	result->m[12] = 0.0f;
	result->m[13] = 0.0f;
	result->m[14] = 0.0f;
	result->m[15] = 1.0f;
}

inline static void
XrMatrix4x4f_CreateTranslation(XrMatrix4x4f* result, const float x, const float y, const float z)
{
	result->m[0] = 1.0f;
	result->m[1] = 0.0f;
	result->m[2] = 0.0f;
	result->m[3] = 0.0f;
	result->m[4] = 0.0f;
	result->m[5] = 1.0f;
	result->m[6] = 0.0f;
	result->m[7] = 0.0f;
	result->m[8] = 0.0f;
	result->m[9] = 0.0f;
	result->m[10] = 1.0f;
	result->m[11] = 0.0f;
	result->m[12] = x;
	result->m[13] = y;
	result->m[14] = z;
	result->m[15] = 1.0f;
}

inline static void
XrMatrix4x4f_Multiply(XrMatrix4x4f* result, const XrMatrix4x4f* a, const XrMatrix4x4f* b)
{
	result->m[0] = a->m[0] * b->m[0] + a->m[4] * b->m[1] + a->m[8] * b->m[2] + a->m[12] * b->m[3];
	result->m[1] = a->m[1] * b->m[0] + a->m[5] * b->m[1] + a->m[9] * b->m[2] + a->m[13] * b->m[3];
	result->m[2] = a->m[2] * b->m[0] + a->m[6] * b->m[1] + a->m[10] * b->m[2] + a->m[14] * b->m[3];
	result->m[3] = a->m[3] * b->m[0] + a->m[7] * b->m[1] + a->m[11] * b->m[2] + a->m[15] * b->m[3];

	result->m[4] = a->m[0] * b->m[4] + a->m[4] * b->m[5] + a->m[8] * b->m[6] + a->m[12] * b->m[7];
	result->m[5] = a->m[1] * b->m[4] + a->m[5] * b->m[5] + a->m[9] * b->m[6] + a->m[13] * b->m[7];
	result->m[6] = a->m[2] * b->m[4] + a->m[6] * b->m[5] + a->m[10] * b->m[6] + a->m[14] * b->m[7];
	result->m[7] = a->m[3] * b->m[4] + a->m[7] * b->m[5] + a->m[11] * b->m[6] + a->m[15] * b->m[7];

	result->m[8] = a->m[0] * b->m[8] + a->m[4] * b->m[9] + a->m[8] * b->m[10] + a->m[12] * b->m[11];
	result->m[9] = a->m[1] * b->m[8] + a->m[5] * b->m[9] + a->m[9] * b->m[10] + a->m[13] * b->m[11];
	result->m[10] = a->m[2] * b->m[8] + a->m[6] * b->m[9] + a->m[10] * b->m[10] + a->m[14] * b->m[11];
	result->m[11] = a->m[3] * b->m[8] + a->m[7] * b->m[9] + a->m[11] * b->m[10] + a->m[15] * b->m[11];

	result->m[12] =
	    a->m[0] * b->m[12] + a->m[4] * b->m[13] + a->m[8] * b->m[14] + a->m[12] * b->m[15];
	result->m[13] =
	    a->m[1] * b->m[12] + a->m[5] * b->m[13] + a->m[9] * b->m[14] + a->m[13] * b->m[15];
	result->m[14] =
	    a->m[2] * b->m[12] + a->m[6] * b->m[13] + a->m[10] * b->m[14] + a->m[14] * b->m[15];
	result->m[15] =
	    a->m[3] * b->m[12] + a->m[7] * b->m[13] + a->m[11] * b->m[14] + a->m[15] * b->m[15];
}

inline static void
XrMatrix4x4f_Invert(XrMatrix4x4f* result, const XrMatrix4x4f* src)
{
	result->m[0] = src->m[0];
	result->m[1] = src->m[4];
	result->m[2] = src->m[8];
	result->m[3] = 0.0f;
	result->m[4] = src->m[1];
	result->m[5] = src->m[5];
	result->m[6] = src->m[9];
	result->m[7] = 0.0f;
	result->m[8] = src->m[2];
	result->m[9] = src->m[6];
	result->m[10] = src->m[10];
	result->m[11] = 0.0f;
	result->m[12] = -(src->m[0] * src->m[12] + src->m[1] * src->m[13] + src->m[2] * src->m[14]);
	result->m[13] = -(src->m[4] * src->m[12] + src->m[5] * src->m[13] + src->m[6] * src->m[14]);
	result->m[14] = -(src->m[8] * src->m[12] + src->m[9] * src->m[13] + src->m[10] * src->m[14]);
	result->m[15] = 1.0f;
}

inline static void
XrMatrix4x4f_CreateViewMatrix(XrMatrix4x4f* result,
                              const XrVector3f* translation,
                              const XrQuaternionf* rotation)
{

	XrMatrix4x4f rotationMatrix;
	XrMatrix4x4f_CreateFromQuaternion(&rotationMatrix, rotation);

	XrMatrix4x4f translationMatrix;
	XrMatrix4x4f_CreateTranslation(&translationMatrix, translation->x, translation->y,
	                               translation->z);

	XrMatrix4x4f viewMatrix;
	XrMatrix4x4f_Multiply(&viewMatrix, &translationMatrix, &rotationMatrix);

	XrMatrix4x4f_Invert(result, &viewMatrix);
}

inline static void
XrMatrix4x4f_CreateScale(XrMatrix4x4f* result, const float x, const float y, const float z)
{
	result->m[0] = x;
	result->m[1] = 0.0f;
	result->m[2] = 0.0f;
	result->m[3] = 0.0f;
	result->m[4] = 0.0f;
	result->m[5] = y;
	result->m[6] = 0.0f;
	result->m[7] = 0.0f;
	result->m[8] = 0.0f;
	result->m[9] = 0.0f;
	result->m[10] = z;
	result->m[11] = 0.0f;
	result->m[12] = 0.0f;
	result->m[13] = 0.0f;
	result->m[14] = 0.0f;
	result->m[15] = 1.0f;
}

inline static void
XrMatrix4x4f_CreateModelMatrix(XrMatrix4x4f* result,
                               const XrVector3f* translation,
                               const XrQuaternionf* rotation,
                               const XrVector3f* scale)
{
	XrMatrix4x4f scaleMatrix;
	XrMatrix4x4f_CreateScale(&scaleMatrix, scale->x, scale->y, scale->z);

	XrMatrix4x4f rotationMatrix;
	XrMatrix4x4f_CreateFromQuaternion(&rotationMatrix, rotation);

	XrMatrix4x4f translationMatrix;
	XrMatrix4x4f_CreateTranslation(&translationMatrix, translation->x, translation->y,
	                               translation->z);

	XrMatrix4x4f combinedMatrix;
	XrMatrix4x4f_Multiply(&combinedMatrix, &rotationMatrix, &scaleMatrix);
	XrMatrix4x4f_Multiply(result, &translationMatrix, &combinedMatrix);
}
// =============================================================================



// =============================================================================
// OpenGL rendering code at the end of the file
// =============================================================================
struct gl_renderer_t
{
	// To render into a texture we need a framebuffer (one per texture to make it easy)
	GLuint** framebuffers;

	float near_z;
	float far_z;

	GLuint shader_program_id;
	GLuint VAO;

	struct
	{
		bool initialized;
		GLuint texture;
		GLuint fbo;
	} quad;
};

struct hand_tracking_t;

#ifdef __linux__
bool
init_sdl_window(Display** xDisplay,
                uint32_t* visualid,
                GLXFBConfig* glxFBConfig,
                GLXDrawable* glxDrawable,
                GLXContext* glxContext,
                int w,
                int h);

int
init_gl(uint32_t view_count, uint32_t* swapchain_lengths, struct gl_renderer_t* gl_renderer);

void
render_frame(int w,
             int h,
             struct gl_renderer_t* gl_renderer,
             uint32_t projection_index,
             XrTime predictedDisplayTime,
             int view_index,
             XrSpaceLocation* hand_locations,
             struct hand_tracking_t* hand_tracking,
             XrMatrix4x4f projectionmatrix,
             XrMatrix4x4f viewmatrix,
             GLuint image,
             bool depth_supported,
             GLuint depthbuffer);

struct quad_layer_t;
void
render_quad(struct gl_renderer_t* gl_rendering,
            struct quad_layer_t* quad,
            uint32_t swapchain_index,
            XrTime predictedDisplayTime);

#endif
// =============================================================================



// true if XrResult is a success code, else print error message and return false
bool
xr_check(XrInstance instance, XrResult result, const char* format, ...)
{
	if (XR_SUCCEEDED(result))
		return true;

	char resultString[XR_MAX_RESULT_STRING_SIZE];
	xrResultToString(instance, result, resultString);

	char formatRes[XR_MAX_RESULT_STRING_SIZE + 1024];
	snprintf(formatRes, XR_MAX_RESULT_STRING_SIZE + 1023, "%s [%s]\n", format, resultString);

	va_list args;
	va_start(args, format);
	vprintf(formatRes, args);
	va_end(args);

	return false;
}



static void
print_instance_properties(XrInstance instance)
{
	XrResult result;
	XrInstanceProperties instance_props = {
	    .type = XR_TYPE_INSTANCE_PROPERTIES,
	    .next = NULL,
	};

	result = xrGetInstanceProperties(instance, &instance_props);
	if (!xr_check(NULL, result, "Failed to get instance info"))
		return;

	printf("Runtime Name: %s\n", instance_props.runtimeName);
	printf("Runtime Version: %d.%d.%d\n", XR_VERSION_MAJOR(instance_props.runtimeVersion),
	       XR_VERSION_MINOR(instance_props.runtimeVersion),
	       XR_VERSION_PATCH(instance_props.runtimeVersion));
}

static void
print_system_properties(XrSystemProperties* system_properties)
{
	printf("System properties for system %lu: \"%s\", vendor ID %d\n", system_properties->systemId,
	       system_properties->systemName, system_properties->vendorId);
	printf("\tMax layers          : %d\n", system_properties->graphicsProperties.maxLayerCount);
	printf("\tMax swapchain height: %d\n",
	       system_properties->graphicsProperties.maxSwapchainImageHeight);
	printf("\tMax swapchain width : %d\n",
	       system_properties->graphicsProperties.maxSwapchainImageWidth);
	printf("\tOrientation Tracking: %d\n", system_properties->trackingProperties.orientationTracking);
	printf("\tPosition Tracking   : %d\n", system_properties->trackingProperties.positionTracking);

	const XrBaseInStructure* next = system_properties->next;
	while (next) {
		if (next->type == XR_TYPE_SYSTEM_HAND_TRACKING_PROPERTIES_EXT) {
			XrSystemHandTrackingPropertiesEXT* ht = system_properties->next;
			printf("\tHand Tracking       : %d\n", ht->supportsHandTracking);
		}
		next = next->next;
	}
}

static void
print_supported_view_configs(XrInstance instance, XrSystemId system_id)
{
	XrResult result;

	uint32_t view_config_count;
	result = xrEnumerateViewConfigurations(instance, system_id, 0, &view_config_count, NULL);
	if (!xr_check(instance, result, "Failed to get view configuration count"))
		return;

	printf("Runtime supports %d view configurations\n", view_config_count);

	XrViewConfigurationType view_configs[view_config_count];
	result = xrEnumerateViewConfigurations(instance, system_id, view_config_count, &view_config_count,
	                                       view_configs);
	if (!xr_check(instance, result, "Failed to enumerate view configurations!"))
		return;

	for (uint32_t i = 0; i < view_config_count; ++i) {
		XrViewConfigurationProperties props = {.type = XR_TYPE_VIEW_CONFIGURATION_PROPERTIES,
		                                       .next = NULL};

		result = xrGetViewConfigurationProperties(instance, system_id, view_configs[i], &props);
		if (!xr_check(instance, result, "Failed to get view configuration info %d!", i))
			return;

		printf("type %d: FOV mutable: %d\n", props.viewConfigurationType, props.fovMutable);
	}
}

static void
print_viewconfig_view_info(uint32_t view_count, XrViewConfigurationView* viewconfig_views)
{
	for (uint32_t i = 0; i < view_count; i++) {
		printf("View Configuration View %d:\n", i);
		printf("\tResolution       : Recommended %dx%d, Max: %dx%d\n",
		       viewconfig_views[0].recommendedImageRectWidth,
		       viewconfig_views[0].recommendedImageRectHeight, viewconfig_views[0].maxImageRectWidth,
		       viewconfig_views[0].maxImageRectHeight);
		printf("\tSwapchain Samples: Recommended: %d, Max: %d)\n",
		       viewconfig_views[0].recommendedSwapchainSampleCount,
		       viewconfig_views[0].maxSwapchainSampleCount);
	}
}

static void
print_reference_spaces(XrInstance instance, XrSession session)
{
	XrResult result;

	uint32_t ref_space_count;
	result = xrEnumerateReferenceSpaces(session, 0, &ref_space_count, NULL);
	if (!xr_check(instance, result, "Getting number of reference spaces failed!"))
		return;

	XrReferenceSpaceType* ref_spaces = malloc(sizeof(XrReferenceSpaceType) * ref_space_count);
	result = xrEnumerateReferenceSpaces(session, ref_space_count, &ref_space_count, ref_spaces);
	if (!xr_check(instance, result, "Enumerating reference spaces failed!"))
		return;

	printf("Runtime supports %d reference spaces:\n", ref_space_count);
	for (uint32_t i = 0; i < ref_space_count; i++) {
		if (ref_spaces[i] == XR_REFERENCE_SPACE_TYPE_LOCAL) {
			printf("\tXR_REFERENCE_SPACE_TYPE_LOCAL\n");
		} else if (ref_spaces[i] == XR_REFERENCE_SPACE_TYPE_STAGE) {
			printf("\tXR_REFERENCE_SPACE_TYPE_STAGE\n");
		} else if (ref_spaces[i] == XR_REFERENCE_SPACE_TYPE_VIEW) {
			printf("\tXR_REFERENCE_SPACE_TYPE_VIEW\n");
		} else {
			printf("\tOther (extension?) refspace %u\\n", ref_spaces[i]);
		}
	}
	free(ref_spaces);
}

static bool
check_opengl_version(XrGraphicsRequirementsOpenGLKHR* opengl_reqs)
{
	XrVersion desired_opengl_version = XR_MAKE_VERSION(3, 3, 0);
	if (desired_opengl_version > opengl_reqs->maxApiVersionSupported ||
	    desired_opengl_version < opengl_reqs->minApiVersionSupported) {
		printf(
		    "We want OpenGL %d.%d.%d, but runtime only supports OpenGL "
		    "%d.%d.%d - %d.%d.%d!\n",
		    XR_VERSION_MAJOR(desired_opengl_version), XR_VERSION_MINOR(desired_opengl_version),
		    XR_VERSION_PATCH(desired_opengl_version),
		    XR_VERSION_MAJOR(opengl_reqs->minApiVersionSupported),
		    XR_VERSION_MINOR(opengl_reqs->minApiVersionSupported),
		    XR_VERSION_PATCH(opengl_reqs->minApiVersionSupported),
		    XR_VERSION_MAJOR(opengl_reqs->maxApiVersionSupported),
		    XR_VERSION_MINOR(opengl_reqs->maxApiVersionSupported),
		    XR_VERSION_PATCH(opengl_reqs->maxApiVersionSupported));
		return false;
	}
	return true;
}

// returns the preferred swapchain format if it is supported
// else:
// - if fallback is true, return the first supported format
// - if fallback is false, return -1
static int64_t
get_swapchain_format(XrInstance instance,
                     XrSession session,
                     int64_t preferred_format,
                     bool fallback)
{
	XrResult result;

	uint32_t swapchain_format_count;
	result = xrEnumerateSwapchainFormats(session, 0, &swapchain_format_count, NULL);
	if (!xr_check(instance, result, "Failed to get number of supported swapchain formats"))
		return -1;

	printf("Runtime supports %d swapchain formats\n", swapchain_format_count);
	int64_t* swapchain_formats = malloc(sizeof(int64_t) * swapchain_format_count);
	result = xrEnumerateSwapchainFormats(session, swapchain_format_count, &swapchain_format_count,
	                                     swapchain_formats);
	if (!xr_check(instance, result, "Failed to enumerate swapchain formats"))
		return -1;

	int64_t chosen_format = fallback ? swapchain_formats[0] : -1;

	for (uint32_t i = 0; i < swapchain_format_count; i++) {
		printf("Supported GL format: %#lx\n", swapchain_formats[i]);
		if (swapchain_formats[i] == preferred_format) {
			chosen_format = swapchain_formats[i];
			printf("Using preferred swapchain format %#lx\n", chosen_format);
			break;
		}
	}
	if (fallback && chosen_format != preferred_format) {
		printf("Falling back to non preferred swapchain format %#lx\n", chosen_format);
	}

	free(swapchain_formats);

	return chosen_format;
}

struct opengl_t
{
	bool supported;
	uint32_t version;
	// functions belonging to extensions must be loaded with xrGetInstanceProcAddr before use
	PFN_xrGetOpenGLGraphicsRequirementsKHR pfnGetOpenGLGraphicsRequirementsKHR;
};

struct hand_tracking_t
{
	bool supported;
	uint32_t version;

	// whether the current VR system in use has hand tracking
	bool system_supported;
	XrHandTrackerEXT trackers[HAND_COUNT];

	// out data
	XrHandJointLocationEXT joints[HAND_COUNT][XR_HAND_JOINT_COUNT_EXT];
	XrHandJointLocationsEXT joint_locations[HAND_COUNT];

	// optional
	XrHandJointVelocitiesEXT joint_velocities[HAND_COUNT];
	XrHandJointVelocityEXT joint_velocities_arr[HAND_COUNT][XR_HAND_JOINT_COUNT_EXT];

	PFN_xrLocateHandJointsEXT pfnLocateHandJointsEXT;
	PFN_xrCreateHandTrackerEXT pfnCreateHandTrackerEXT;
};

struct depth_t
{
	bool supported;
	XrCompositionLayerDepthInfoKHR* infos;
};

struct ext_t
{
	struct opengl_t opengl;
	struct hand_tracking_t hand_tracking;

	// technically not extensions, but can be supported or not
	struct depth_t depth;
};


static bool
_extension_supported(XrExtensionProperties* extension_props, uint32_t ext_count, char* ext_name)
{
	for (uint32_t i = 0; i < ext_count; i++) {
		if (strcmp(ext_name, extension_props[i].extensionName) == 0) {
			return true;
		}
	}
	return false;
}

static uint32_t
_extension_version(XrExtensionProperties* extension_props, uint32_t ext_count, char* ext_name)
{
	for (uint32_t i = 0; i < ext_count; i++) {
		if (strcmp(ext_name, extension_props[i].extensionName) == 0) {
			return extension_props[i].extensionVersion;
		}
	}
	return 0;
}

static XrResult
_init_opengl_ext(XrInstance instance, struct ext_t* ext)
{
	XrResult result =
	    xrGetInstanceProcAddr(instance, "xrGetOpenGLGraphicsRequirementsKHR",
	                          (PFN_xrVoidFunction*)&ext->opengl.pfnGetOpenGLGraphicsRequirementsKHR);
	if (!xr_check(instance, result, "Failed to get xrGetOpenGLGraphicsRequirementsKHR function!"))
		return result;
	return XR_SUCCESS;
}

static XrResult
_init_hand_tracking_ext(XrInstance instance, struct ext_t* ext)
{
	XrResult result;
	result = xrGetInstanceProcAddr(instance, "xrLocateHandJointsEXT",
	                               (PFN_xrVoidFunction*)&ext->hand_tracking.pfnLocateHandJointsEXT);
	if (!xr_check(instance, result, "Failed to get xrLocateHandJointsEXT function!")) {
		return result;
	}

	result = xrGetInstanceProcAddr(instance, "xrCreateHandTrackerEXT",
	                               (PFN_xrVoidFunction*)&ext->hand_tracking.pfnCreateHandTrackerEXT);
	if (!xr_check(instance, result, "Failed to get xrCreateHandTrackerEXT function!")) {
		return result;
	}
	return XR_SUCCESS;
}

static XrResult
_check_extension_support(struct ext_t* ext)
{
	XrResult result;

	// xrEnumerate*() functions are usually called once with CapacityInput = 0.
	// The function will write the required amount into CountOutput. We then have
	// to allocate an array to hold CountOutput elements and call the function
	// with CountOutput as CapacityInput.
	uint32_t ext_count = 0;
	result = xrEnumerateInstanceExtensionProperties(NULL, 0, &ext_count, NULL);

	/* TODO: instance null will not be able to convert XrResult to string */
	if (!xr_check(NULL, result, "Failed to enumerate number of extension properties"))
		return result;


	XrExtensionProperties ext_props[ext_count];
	for (uint16_t i = 0; i < ext_count; i++) {
		// we usually have to fill in the type (for validation) and set
		// next to NULL (or a pointer to an extension specific struct)
		ext_props[i].type = XR_TYPE_EXTENSION_PROPERTIES;
		ext_props[i].next = NULL;
	}

	result = xrEnumerateInstanceExtensionProperties(NULL, ext_count, &ext_count, ext_props);
	if (!xr_check(NULL, result, "Failed to enumerate extension properties"))
		return result;

	printf("Runtime supports %d extensions\n", ext_count);
	for (uint32_t i = 0; i < ext_count; i++) {
		printf("\t%s v%d\n", ext_props[i].extensionName, ext_props[i].extensionVersion);
	}


	if (_extension_supported(ext_props, ext_count, XR_KHR_OPENGL_ENABLE_EXTENSION_NAME)) {
		ext->opengl.supported = true;
		ext->opengl.version =
		    _extension_version(ext_props, ext_count, XR_KHR_OPENGL_ENABLE_EXTENSION_NAME);
	} else {
		// opengl required
		return XR_ERROR_EXTENSION_NOT_PRESENT;
	}

	if (_extension_supported(ext_props, ext_count, XR_EXT_HAND_TRACKING_EXTENSION_NAME)) {
		ext->hand_tracking.supported = true;
		ext->hand_tracking.version =
		    _extension_version(ext_props, ext_count, XR_EXT_HAND_TRACKING_EXTENSION_NAME);
	}

	return XR_SUCCESS;
}

static XrResult
_init_extensions(XrInstance instance, struct ext_t* ext)
{
	XrResult result;

	// opengl is required
	result = _init_opengl_ext(instance, ext);
	if (!xr_check(instance, result, "Failed to init OpenGL ext")) {
		return XR_ERROR_EXTENSION_NOT_PRESENT;
	}

	// those may succeed or not
	result = _init_hand_tracking_ext(instance, ext);

	return XR_SUCCESS;
}

struct swapchain_t
{
	uint32_t* swapchain_lengths;
	XrSwapchainImageOpenGLKHR** images;
	XrSwapchain* swapchains;
	uint32_t swapchain_count;
};

enum Swapchain
{
	SWAPCHAIN_PROJECTION = 0,
	SWAPCHAIN_DEPTH,
	SWAPCHAIN_LAST
};

// --- Create swapchain
static bool
_create_swapchain(XrInstance instance,
                  XrSession session,
                  struct swapchain_t* swapchain,
                  int num_swapchain,
                  int64_t format,
                  uint32_t sample_count,
                  uint32_t w,
                  uint32_t h,
                  XrSwapchainUsageFlags usage_flags)
{
	XrSwapchainCreateInfo swapchain_create_info = {
	    .type = XR_TYPE_SWAPCHAIN_CREATE_INFO,
	    .usageFlags = usage_flags,
	    .createFlags = 0,
	    .format = format,
	    .sampleCount = sample_count,
	    .width = w,
	    .height = h,
	    .faceCount = 1,
	    .arraySize = 1,
	    .mipCount = 1,
	    .next = NULL,
	};

	XrResult result;

	result =
	    xrCreateSwapchain(session, &swapchain_create_info, &swapchain->swapchains[num_swapchain]);
	if (!xr_check(instance, result, "Failed to create swapchain!"))
		return false;

	// The runtime controls how many textures we have to be able to render to
	// (e.g. "triple buffering")
	result = xrEnumerateSwapchainImages(swapchain->swapchains[num_swapchain], 0,
	                                    &swapchain->swapchain_lengths[num_swapchain], NULL);
	if (!xr_check(instance, result, "Failed to enumerate swapchains"))
		return false;

	swapchain->images[num_swapchain] =
	    malloc(sizeof(XrSwapchainImageOpenGLKHR) * swapchain->swapchain_lengths[num_swapchain]);
	for (uint32_t j = 0; j < swapchain->swapchain_lengths[num_swapchain]; j++) {
		swapchain->images[num_swapchain][j].type = XR_TYPE_SWAPCHAIN_IMAGE_OPENGL_KHR;
		swapchain->images[num_swapchain][j].next = NULL;
	}
	result = xrEnumerateSwapchainImages(
	    swapchain->swapchains[num_swapchain], swapchain->swapchain_lengths[num_swapchain],
	    &swapchain->swapchain_lengths[num_swapchain],
	    (XrSwapchainImageBaseHeader*)swapchain->images[num_swapchain]);
	if (!xr_check(instance, result, "Failed to enumerate swapchain images"))
		return false;

	return true;
}

static bool
create_one_swapchain(XrInstance instance,
                     XrSession session,
                     struct swapchain_t* swapchain,
                     int64_t format,
                     uint32_t sample_count,
                     uint32_t w,
                     uint32_t h,
                     XrSwapchainUsageFlags usage_flags)
{
	swapchain->swapchains = malloc(sizeof(XrSwapchain));
	swapchain->swapchain_lengths = malloc(sizeof(uint32_t));
	swapchain->images = malloc(sizeof(XrSwapchainImageOpenGLKHR*));
	swapchain->swapchain_count = 1;

	return _create_swapchain(instance, session, swapchain, 0, format, sample_count, w, h,
	                         usage_flags);
}

static bool
create_swapchain_from_views(XrInstance instance,
                            XrSession session,
                            struct swapchain_t* swapchain,
                            uint32_t view_count,
                            int64_t format,
                            XrViewConfigurationView* viewconfig_views,
                            XrSwapchainUsageFlags usage_flags)
{
	swapchain->swapchains = malloc(sizeof(XrSwapchain) * view_count);
	swapchain->swapchain_lengths = malloc(sizeof(uint32_t) * view_count);
	swapchain->images = malloc(sizeof(XrSwapchainImageOpenGLKHR*) * view_count);
	swapchain->swapchain_count = view_count;

	for (uint32_t i = 0; i < view_count; i++) {
		uint32_t sample_count = viewconfig_views[i].recommendedSwapchainSampleCount;
		uint32_t w = viewconfig_views[i].recommendedImageRectWidth;
		uint32_t h = viewconfig_views[i].recommendedImageRectHeight;

		if (!_create_swapchain(instance, session, swapchain, i, format, sample_count, w, h,
		                       usage_flags))
			return false;
	}

	return true;
}

static bool
acquire_swapchain(XrInstance instance,
                  struct swapchain_t* swapchain,
                  int num_swapchain,
                  uint32_t* index)
{
	XrResult result;
	XrSwapchainImageAcquireInfo acquire_info = {.type = XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO,
	                                            .next = NULL};
	result = xrAcquireSwapchainImage(swapchain->swapchains[num_swapchain], &acquire_info, index);
	if (!xr_check(instance, result, "failed to acquire swapchain image!"))
		return false;

	XrSwapchainImageWaitInfo wait_info = {
	    .type = XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO, .next = NULL, .timeout = 1000};
	result = xrWaitSwapchainImage(swapchain->swapchains[num_swapchain], &wait_info);
	if (!xr_check(instance, result, "failed to wait for swapchain image!"))
		return false;

	return true;
}

static void
destroy_swapchain(struct swapchain_t* swapchain)
{
	free(swapchain->swapchains);
	free(swapchain->images);
	free(swapchain->swapchain_lengths);
}


bool
create_action(XrInstance instance,
              XrActionType type,
              char* name,
              char* localized_name,
              XrActionSet set,
              int subaction_count,
              XrPath* subactions,
              XrAction* out_action)
{
	XrActionCreateInfo actionInfo = {.type = XR_TYPE_ACTION_CREATE_INFO,
	                                 .actionType = type,
	                                 .countSubactionPaths = subaction_count,
	                                 .subactionPaths = subactions};
	strcpy(actionInfo.actionName, name);
	strcpy(actionInfo.localizedActionName, localized_name);

	XrResult result = xrCreateAction(set, &actionInfo, out_action);
	if (!xr_check(instance, result, "Failed to create action %s", name))
		return false;

	return true;
}

struct Binding
{
	XrAction action;
	char* paths[HAND_COUNT];
	int path_count;
};

bool
suggest_actions(XrInstance instance, char* profile, struct Binding* b, int binding_count)
{
	XrPath interactionProfilePath;
	XrResult result = xrStringToPath(instance, profile, &interactionProfilePath);
	if (!xr_check(instance, result, "Failed to get interaction profile path %s", profile))
		return false;

	int total = 0;
	for (int i = 0; i < binding_count; i++) {
		struct Binding* binding = &b[i];
		total += binding->path_count;
	}

	XrActionSuggestedBinding* bindings = malloc(sizeof(XrActionSuggestedBinding) * total);

	printf("Suggesting %d actions for %s\n", binding_count, profile);

	int processed_bindings = 0;
	for (int i = 0; i < binding_count; i++) {
		struct Binding* binding = &b[i];

		for (int j = 0; j < binding->path_count; j++) {
			XrPath path;
			result = xrStringToPath(instance, binding->paths[j], &path);
			if (!xr_check(instance, result, "Failed to get binding path %s", binding->paths[j]))
				return false;

			int current = processed_bindings++;
			bindings[current].action = binding->action;
			bindings[current].binding = path;

			printf("%p (%d): %s\n", (void*)binding->action, j, binding->paths[j]);
		}
	};

	const XrInteractionProfileSuggestedBinding suggestedBindings = {
	    .type = XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING,
	    .interactionProfile = interactionProfilePath,
	    .countSuggestedBindings = total,
	    .suggestedBindings = bindings};

	result = xrSuggestInteractionProfileBindings(instance, &suggestedBindings);
	if (!xr_check(instance, result, "Failed to suggest actions"))
		return false;

	free(bindings);

	return true;
}

struct action_t
{
	XrAction action;
	XrActionType action_type;
	union {
		XrActionStateFloat float_;
		XrActionStateBoolean boolean_;
		XrActionStatePose pose_;
		XrActionStateVector2f vec2f_;
	} states[HAND_COUNT];
	XrSpace pose_spaces[HAND_COUNT];
	XrSpaceLocation pose_locations[HAND_COUNT];
	XrSpaceVelocity pose_velocities[HAND_COUNT];
};

bool
create_action_space(XrInstance instance,
                    XrSession session,
                    struct action_t* action,
                    XrPath* hand_paths)
{
	// poses can't be queried directly, we need to create a space for each
	for (int hand = 0; hand < HAND_COUNT; hand++) {
		XrActionSpaceCreateInfo action_space_info = {.type = XR_TYPE_ACTION_SPACE_CREATE_INFO,
		                                             .next = NULL,
		                                             .action = action->action,
		                                             .poseInActionSpace = identity_pose,
		                                             .subactionPath = hand_paths[hand]};

		XrResult result;
		result = xrCreateActionSpace(session, &action_space_info, &action->pose_spaces[hand]);
		if (!xr_check(instance, result, "failed to create hand %d pose space", hand))
			return false;
	}
	return true;
}

bool
get_action_data(XrInstance instance,
                XrSession session,
                struct action_t* action,
                int hand,
                XrPath* subaction_paths,
                XrSpace space,
                XrTime time,
                bool velocities)
{
	XrActionStateGetInfo info = {
	    .type = XR_TYPE_ACTION_STATE_GET_INFO,
	    .next = NULL,
	    .action = action->action,
	    .subactionPath = subaction_paths[hand],
	};
	XrResult result;
	if (action->action_type == XR_ACTION_TYPE_FLOAT_INPUT) {
		action->states[hand].float_.type = XR_TYPE_ACTION_STATE_FLOAT;
		action->states[hand].float_.next = NULL;
		result = xrGetActionStateFloat(session, &info, &action->states[hand].float_);
		if (!xr_check(instance, result, "Failed to get float"))
			return false;
	}
	if (action->action_type == XR_ACTION_TYPE_BOOLEAN_INPUT) {
		action->states[hand].boolean_.type = XR_TYPE_ACTION_STATE_BOOLEAN;
		action->states[hand].boolean_.next = NULL;
		result = xrGetActionStateBoolean(session, &info, &action->states[hand].boolean_);
		if (!xr_check(instance, result, "Failed to get bool"))
			return false;
	}
	if (action->action_type == XR_ACTION_TYPE_VECTOR2F_INPUT) {
		action->states[hand].vec2f_.type = XR_TYPE_ACTION_STATE_VECTOR2F;
		action->states[hand].vec2f_.next = NULL;
		result = xrGetActionStateVector2f(session, &info, &action->states[hand].vec2f_);
		if (!xr_check(instance, result, "Failed to get vec2f"))
			return false;
	}
	if (action->action_type == XR_ACTION_TYPE_POSE_INPUT) {
		action->states[hand].pose_.type = XR_TYPE_ACTION_STATE_POSE;
		action->states[hand].pose_.next = NULL;
		result = xrGetActionStatePose(session, &info, &action->states[hand].pose_);
		if (!xr_check(instance, result, "Failed to get action state pose"))
			return false;

		if (action->states[hand].pose_.isActive) {
			action->pose_locations[hand].type = XR_TYPE_SPACE_LOCATION;
			action->pose_locations[hand].next = NULL;

			if (velocities) {
				action->pose_velocities[hand].type = XR_TYPE_SPACE_VELOCITY;
				action->pose_velocities[hand].next = NULL;
				action->pose_locations[hand].next = &action->pose_velocities[hand];
			} else {
				action->pose_locations[hand].next = NULL;
			}

			result = xrLocateSpace(action->pose_spaces[hand], space, time, &action->pose_locations[hand]);
			if (!xr_check(instance, result, "Failed to locate hand space"))
				return false;
		}
	}

	if (!xr_check(instance, result, "Failed to get action state")) {
		return false;
	}

	return true;
}

struct quad_layer_t
{
	// quad layers are placed into world space, no need to render them per eye
	struct swapchain_t swapchain;
	uint32_t pixel_width, pixel_height;
};

static bool
create_hand_trackers(XrInstance instance, XrSession session, struct hand_tracking_t* hand_tracking)
{
	XrResult result;

	result = xrGetInstanceProcAddr(instance, "xrLocateHandJointsEXT",
	                               (PFN_xrVoidFunction*)&hand_tracking->pfnLocateHandJointsEXT);

	XrHandEXT hands[HAND_COUNT] = {
	    [HAND_LEFT_INDEX] = XR_HAND_LEFT_EXT, [HAND_RIGHT_INDEX] = XR_HAND_RIGHT_EXT};

	for (int i = 0; i < HAND_COUNT; i++) {
		XrHandTrackerCreateInfoEXT hand_tracker_create_info = {
		    .type = XR_TYPE_HAND_TRACKER_CREATE_INFO_EXT,
		    .next = NULL,
		    .hand = hands[i],
		    .handJointSet = XR_HAND_JOINT_SET_DEFAULT_EXT};
		result = hand_tracking->pfnCreateHandTrackerEXT(session, &hand_tracker_create_info,
		                                                &hand_tracking->trackers[i]);
		if (!xr_check(instance, result, "Failed to create hand tracker %d", i)) {
			return false;
		}

		hand_tracking->joint_locations[i] = (XrHandJointLocationsEXT){
		    .type = XR_TYPE_HAND_JOINT_LOCATIONS_EXT,
		    .jointCount = XR_HAND_JOINT_COUNT_EXT,
		    .jointLocations = hand_tracking->joints[i],
		};

		printf("Created hand tracker %d\n", i);
	}
	return true;
}

static bool
get_hand_tracking(XrInstance instance,
                  XrSpace space,
                  XrTime time,
                  bool query_joint_velocities,
                  struct hand_tracking_t* hand_tracking,
                  int hand)
{
	if (query_joint_velocities) {
		hand_tracking->joint_velocities[hand].type = XR_TYPE_HAND_JOINT_VELOCITIES_EXT;
		hand_tracking->joint_velocities[hand].next = NULL;
		hand_tracking->joint_velocities[hand].jointCount = XR_HAND_JOINT_COUNT_EXT;
		hand_tracking->joint_velocities[hand].jointVelocities =
		    hand_tracking->joint_velocities_arr[hand];
		hand_tracking->joint_locations[hand].next = &hand_tracking->joint_velocities[hand];
	} else {
		hand_tracking->joint_locations[hand].next = NULL;
	}

	XrHandJointsLocateInfoEXT locateInfo = {
	    .type = XR_TYPE_HAND_JOINTS_LOCATE_INFO_EXT, .next = NULL, .baseSpace = space, .time = time};

	XrResult result;
	result = hand_tracking->pfnLocateHandJointsEXT(hand_tracking->trackers[hand], &locateInfo,
	                                               &hand_tracking->joint_locations[hand]);
	if (!xr_check(instance, result, "failed to locate hand joints!"))
		return false;

	return true;
}


static char*
get_arg(int argc, char** argv, char* arg_name)
{
	for (int i = 0; i < argc; i++) {
		if (strcmp(argv[i], arg_name) == 0 && i + 1 < argc) {
			return argv[i + 1];
		}
	}
	return false;
}

static void
arg_to_enum(int argc, char** argv, char* arg_name, XrStr_fn fn, uint64_t* val_ptr)
{
	char* arg_value = get_arg(argc, argv, arg_name);
	if (!arg_value) {
		return;
	}
	printf("Parsing arg %s: %s\n", arg_name, arg_value);

	for (long i = 0; i < 0x7FFFFFFF; i++) {
		// hack: not all enums start with 0
		if (strcmp(fn(i), "Unknown") == 0 && i > 1) {
			printf("no value found for arg %s %s\n", arg_name, arg_value);
			return;
		}
		if (strcmp(fn(i), arg_value) == 0) {
			*val_ptr = i;
			printf("parsed arg %s %s (%lu)\n", arg_name, arg_value, i);
			return;
		}
	}
}

int
main(int argc, char** argv)
{
	XrFormFactor form_factor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
	XrViewConfigurationType view_type = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
	XrReferenceSpaceType play_space_type = XR_REFERENCE_SPACE_TYPE_LOCAL;

	// use optional XrSpaceVelocity in xrLocateSpace for controllers and visualize linear velocity
	bool query_hand_velocities = true;

	// use optional XrSpaceVelocity in xrLocateHandJointsEXT and visualize linear velocity
	bool query_joint_velocities = false;

	arg_to_enum(argc, argv, "--form_factor", (XrStr_fn)XrStr_XrFormFactor, (uint64_t*)&form_factor);
	arg_to_enum(argc, argv, "--view_type", (XrStr_fn)XrStr_XrViewConfigurationType,
	            (uint64_t*)&view_type);
	arg_to_enum(argc, argv, "--play_space_type", (XrStr_fn)XrStr_XrReferenceSpaceType,
	            (uint64_t*)&play_space_type);

	for (int i = 0; i < argc; i++) {
		if (strcmp(argv[i], "--joint_velocities") == 0) {
			query_joint_velocities = true;
		}
	}

	// every OpenXR app that displays something needs at least an instance and a session
	XrInstance instance = XR_NULL_HANDLE;
	XrSession session = XR_NULL_HANDLE;
	XrSystemId system_id = XR_NULL_SYSTEM_ID;
	XrSessionState state = XR_SESSION_STATE_UNKNOWN;

	// Play space is usually local (head is origin, seated) or stage (room scale)
	XrSpace play_space = XR_NULL_HANDLE;

	// Each physical Display/Eye is described by a view
	uint32_t view_count = 0;
	XrViewConfigurationView* viewconfig_views = NULL;
	XrCompositionLayerProjectionView* projection_views = NULL;
	XrView* views = NULL;

	// The runtime interacts with the OpenGL images (textures) via a Swapchain.
	XrGraphicsBindingOpenGLXlibKHR graphics_binding_gl = {0};

	struct swapchain_t vr_swapchains[SWAPCHAIN_LAST];

	struct quad_layer_t quad_layer = {.pixel_width = 320, .pixel_height = 240};

	XrPath hand_paths[HAND_COUNT];

	struct gl_renderer_t gl_rendering = {
	    .near_z = 0.01f,
	    .far_z = 100.0f,
	};

	struct ext_t ext = {0};

	// reuse this variable for all our OpenXR return codes
	XrResult result = XR_SUCCESS;

	result = _check_extension_support(&ext);
	if (!xr_check(instance, result, "Extensions check failed!")) {
		return 1;
	}

	// --- Create XrInstance
	int enabled_ext_count = 1;
	const char* enabled_exts[2] = {XR_KHR_OPENGL_ENABLE_EXTENSION_NAME};
	if (ext.hand_tracking.supported) {
		enabled_exts[enabled_ext_count++] = XR_EXT_HAND_TRACKING_EXTENSION_NAME;
	}

	// same can be done for API layers, but API layers can also be enabled by env var

	XrInstanceCreateInfo instance_create_info = {
	    .type = XR_TYPE_INSTANCE_CREATE_INFO,
	    .next = NULL,
	    .createFlags = 0,
	    .enabledExtensionCount = enabled_ext_count,
	    .enabledExtensionNames = enabled_exts,
	    .enabledApiLayerCount = 0,
	    .enabledApiLayerNames = NULL,
	    .applicationInfo =
	        {
	            // some compilers have trouble with char* initialization
	            .applicationName = "",
	            .engineName = "",
	            .applicationVersion = 1,
	            .engineVersion = 0,
	            .apiVersion = XR_CURRENT_API_VERSION,
	        },
	};
	strncpy(instance_create_info.applicationInfo.applicationName, "OpenXR OpenGL Example",
	        XR_MAX_APPLICATION_NAME_SIZE);
	strncpy(instance_create_info.applicationInfo.engineName, "Custom", XR_MAX_ENGINE_NAME_SIZE);

	result = xrCreateInstance(&instance_create_info, &instance);
	if (!xr_check(NULL, result, "Failed to create XR instance."))
		return 1;

	result = _init_extensions(instance, &ext);
	if (!xr_check(instance, result, "Failed to init extensions!")) {
		return 1;
	}

	// Optionally get runtime name and version
	print_instance_properties(instance);

	// --- Create XrSystem
	XrSystemGetInfo system_get_info = {
	    .type = XR_TYPE_SYSTEM_GET_INFO, .formFactor = form_factor, .next = NULL};

	result = xrGetSystem(instance, &system_get_info, &system_id);
	if (!xr_check(instance, result, "Failed to get system for HMD form factor."))
		return 1;

	printf("Successfully got XrSystem with id %lu for HMD form factor\n", system_id);


	// checking system properties is generally  optional, but we are interested in hand tracking
	// support
	{
		XrSystemProperties system_props = {
		    .type = XR_TYPE_SYSTEM_PROPERTIES,
		    .next = NULL,
		    .graphicsProperties = {0},
		    .trackingProperties = {0},
		};

		XrSystemHandTrackingPropertiesEXT ht = {.type = XR_TYPE_SYSTEM_HAND_TRACKING_PROPERTIES_EXT,
		                                        .next = NULL};
		if (ext.hand_tracking.supported) {
			system_props.next = &ht;
		}

		result = xrGetSystemProperties(instance, system_id, &system_props);
		if (!xr_check(instance, result, "Failed to get System properties"))
			return 1;

		ext.hand_tracking.system_supported = ext.hand_tracking.supported && ht.supportsHandTracking;

		print_system_properties(&system_props);
	}

	print_supported_view_configs(instance, system_id);

	// view_count usually depends on the form_factor / view_type.
	// dynamically allocating all view related structs hopefully allows this app to scale easily to
	// different view_counts.

	result = xrEnumerateViewConfigurationViews(instance, system_id, view_type, 0, &view_count, NULL);
	if (!xr_check(instance, result, "Failed to get view configuration view count!"))
		return 1;

	viewconfig_views = malloc(sizeof(XrViewConfigurationView) * view_count);
	for (uint32_t i = 0; i < view_count; i++) {
		viewconfig_views[i].type = XR_TYPE_VIEW_CONFIGURATION_VIEW;
		viewconfig_views[i].next = NULL;
	}

	result = xrEnumerateViewConfigurationViews(instance, system_id, view_type, view_count,
	                                           &view_count, viewconfig_views);
	if (!xr_check(instance, result, "Failed to enumerate view configuration views!"))
		return 1;
	print_viewconfig_view_info(view_count, viewconfig_views);


	// OpenXR requires checking graphics requirements before creating a session.
	XrGraphicsRequirementsOpenGLKHR opengl_reqs = {.type = XR_TYPE_GRAPHICS_REQUIREMENTS_OPENGL_KHR,
	                                               .next = NULL};

	// this function pointer was loaded with xrGetInstanceProcAddr
	result = ext.opengl.pfnGetOpenGLGraphicsRequirementsKHR(instance, system_id, &opengl_reqs);
	if (!xr_check(instance, result, "Failed to get OpenGL graphics requirements!"))
		return 1;

	// On OpenGL we never fail this check because the version requirement is not useful.
	// Other APIs may have more useful requirements.
	check_opengl_version(&opengl_reqs);


	// --- Create session
	graphics_binding_gl = (XrGraphicsBindingOpenGLXlibKHR){
	    .type = XR_TYPE_GRAPHICS_BINDING_OPENGL_XLIB_KHR,
	};

	// create SDL window the size of the left eye & fill GL graphics binding info
	if (!init_sdl_window(&graphics_binding_gl.xDisplay, &graphics_binding_gl.visualid,
	                     &graphics_binding_gl.glxFBConfig, &graphics_binding_gl.glxDrawable,
	                     &graphics_binding_gl.glxContext,
	                     viewconfig_views[0].recommendedImageRectWidth,
	                     viewconfig_views[0].recommendedImageRectHeight)) {
		printf("GLX init failed!\n");
		return 1;
	}

	printf("Using OpenGL version: %s\n", glGetString(GL_VERSION));
	printf("Using OpenGL Renderer: %s\n", glGetString(GL_RENDERER));

	state = XR_SESSION_STATE_UNKNOWN;

	XrSessionCreateInfo session_create_info = {
	    .type = XR_TYPE_SESSION_CREATE_INFO, .next = &graphics_binding_gl, .systemId = system_id};

	result = xrCreateSession(instance, &session_create_info, &session);
	if (!xr_check(instance, result, "Failed to create session"))
		return 1;

	printf("Successfully created a session with OpenGL!\n");

	// Many runtimes support at least STAGE and LOCAL but not all do.
	// Sophisticated apps might check if the chosen one is supported and try another one if not.
	// Here we will get an error from xrCreateReferenceSpace() and exit.
	print_reference_spaces(instance, session);
	XrReferenceSpaceCreateInfo play_space_create_info = {.type = XR_TYPE_REFERENCE_SPACE_CREATE_INFO,
	                                                     .next = NULL,
	                                                     .referenceSpaceType = play_space_type,
	                                                     .poseInReferenceSpace = identity_pose};

	result = xrCreateReferenceSpace(session, &play_space_create_info, &play_space);
	if (!xr_check(instance, result, "Failed to create play space!"))
		return 1;

	// --- Create Swapchains
	uint32_t swapchain_format_count;
	result = xrEnumerateSwapchainFormats(session, 0, &swapchain_format_count, NULL);
	if (!xr_check(instance, result, "Failed to get number of supported swapchain formats"))
		return 1;

	printf("Runtime supports %d swapchain formats\n", swapchain_format_count);
	int64_t swapchain_formats[swapchain_format_count];
	result = xrEnumerateSwapchainFormats(session, swapchain_format_count, &swapchain_format_count,
	                                     swapchain_formats);
	if (!xr_check(instance, result, "Failed to enumerate swapchain formats"))
		return 1;

	// SRGB is usually a better choice than linear
	// a more sophisticated approach would iterate supported swapchain formats and choose from them
	int64_t color_format = get_swapchain_format(instance, session, GL_SRGB8_ALPHA8_EXT, true);

	int64_t quad_format = get_swapchain_format(instance, session, GL_RGBA8_EXT, true);

	int64_t depth_format = get_swapchain_format(instance, session, GL_DEPTH_COMPONENT16, false);
	if (depth_format < 0) {
		printf("Preferred depth format GL_DEPTH_COMPONENT16 not supported, disabling depth\n");
		ext.depth.supported = false;
	} else {
		ext.depth.supported = true;
	}

	XrSwapchainUsageFlags color_flags =
	    XR_SWAPCHAIN_USAGE_SAMPLED_BIT | XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT;
	if (!create_swapchain_from_views(instance, session, &vr_swapchains[SWAPCHAIN_PROJECTION],
	                                 view_count, color_format, viewconfig_views, color_flags))
		return 1;

	if (ext.depth.supported) {
		XrSwapchainUsageFlags depth_flags = XR_SWAPCHAIN_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
		if (!create_swapchain_from_views(instance, session, &vr_swapchains[SWAPCHAIN_DEPTH], view_count,
		                                 depth_format, viewconfig_views, depth_flags)) {
			return 1;
		}
	}

	if (!create_one_swapchain(instance, session, &quad_layer.swapchain, quad_format, 1,
	                          quad_layer.pixel_width, quad_layer.pixel_height, color_flags))
		return 1;

	// Do not allocate these every frame to save some resources
	views = (XrView*)malloc(sizeof(XrView) * view_count);
	projection_views = (XrCompositionLayerProjectionView*)malloc(
	    sizeof(XrCompositionLayerProjectionView) * view_count);
	for (uint32_t i = 0; i < view_count; i++) {
		views[i].type = XR_TYPE_VIEW;
		views[i].next = NULL;

		projection_views[i].type = XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW;
		projection_views[i].next = NULL;

		projection_views[i].subImage.swapchain = vr_swapchains[SWAPCHAIN_PROJECTION].swapchains[i];
		projection_views[i].subImage.imageArrayIndex = 0;
		projection_views[i].subImage.imageRect.offset.x = 0;
		projection_views[i].subImage.imageRect.offset.y = 0;
		projection_views[i].subImage.imageRect.extent.width =
		    viewconfig_views[i].recommendedImageRectWidth;
		projection_views[i].subImage.imageRect.extent.height =
		    viewconfig_views[i].recommendedImageRectHeight;

		// projection_views[i].{pose, fov} have to be filled every frame in frame loop
	};


	if (ext.depth.supported) {
		ext.depth.infos = (XrCompositionLayerDepthInfoKHR*)malloc(
		    sizeof(XrCompositionLayerDepthInfoKHR) * view_count);
		for (uint32_t i = 0; i < view_count; i++) {
			ext.depth.infos[i].type = XR_TYPE_COMPOSITION_LAYER_DEPTH_INFO_KHR;
			ext.depth.infos[i].next = NULL;
			ext.depth.infos[i].minDepth = 0.f;
			ext.depth.infos[i].maxDepth = 1.f;
			ext.depth.infos[i].nearZ = gl_rendering.near_z;
			ext.depth.infos[i].farZ = gl_rendering.far_z;

			ext.depth.infos[i].subImage.swapchain = vr_swapchains[SWAPCHAIN_DEPTH].swapchains[i];

			ext.depth.infos[i].subImage.imageArrayIndex = 0;
			ext.depth.infos[i].subImage.imageRect.offset.x = 0;
			ext.depth.infos[i].subImage.imageRect.offset.y = 0;
			ext.depth.infos[i].subImage.imageRect.extent.width =
			    viewconfig_views[i].recommendedImageRectWidth;
			ext.depth.infos[i].subImage.imageRect.extent.height =
			    viewconfig_views[i].recommendedImageRectHeight;

			projection_views[i].next = &ext.depth.infos[i];
		};
	}


	// --- Set up input (actions)

	xrStringToPath(instance, "/user/hand/left", &hand_paths[HAND_LEFT_INDEX]);
	xrStringToPath(instance, "/user/hand/right", &hand_paths[HAND_RIGHT_INDEX]);

	XrActionSetCreateInfo gameplay_actionset_info = {
	    .type = XR_TYPE_ACTION_SET_CREATE_INFO, .next = NULL, .priority = 0};
	strcpy(gameplay_actionset_info.actionSetName, "gameplay_actionset");
	strcpy(gameplay_actionset_info.localizedActionSetName, "Gameplay Actions");

	XrActionSet gameplay_actionset;
	result = xrCreateActionSet(instance, &gameplay_actionset_info, &gameplay_actionset);
	if (!xr_check(instance, result, "failed to create actionset"))
		return 1;


	// Grabbing objects is not actually implemented in this demo, it only gives some  haptic feebdack.
	struct action_t grab_action = {.action = XR_NULL_HANDLE,
	                               .action_type = XR_ACTION_TYPE_FLOAT_INPUT};
	if (!create_action(instance, XR_ACTION_TYPE_FLOAT_INPUT, "grabobjectfloat", "Grab Object",
	                   gameplay_actionset, HAND_COUNT, hand_paths, &grab_action.action))
		return 1;

	// A 1D action that is fed by one axis of a 2D input (y axis of thumbstick).
	struct action_t accelerate_action = {.action = XR_NULL_HANDLE,
	                                     .action_type = XR_ACTION_TYPE_FLOAT_INPUT};
	if (!create_action(instance, XR_ACTION_TYPE_FLOAT_INPUT, "accelerate", "Accelerate",
	                   gameplay_actionset, HAND_COUNT, hand_paths, &accelerate_action.action))
		return 1;

	struct action_t hand_pose_action = {.action = XR_NULL_HANDLE,
	                                    .action_type = XR_ACTION_TYPE_POSE_INPUT};
	if (!create_action(instance, XR_ACTION_TYPE_POSE_INPUT, "handpose", "Hand Pose",
	                   gameplay_actionset, HAND_COUNT, hand_paths, &hand_pose_action.action))
		return 1;
	if (!create_action_space(instance, session, &hand_pose_action, hand_paths))
		return 1;


	struct action_t haptic_action = {.action = XR_NULL_HANDLE,
	                                 .action_type = XR_ACTION_TYPE_VIBRATION_OUTPUT};
	if (!create_action(instance, XR_ACTION_TYPE_VIBRATION_OUTPUT, "haptic", "Haptic Vibration",
	                   gameplay_actionset, HAND_COUNT, hand_paths, &haptic_action.action))
		return 1;


	struct Binding simple_bindings[] = {
	    {.action = grab_action.action,
	     .paths = {"/user/hand/left/input/select/click", "/user/hand/right/input/select/click"},
	     .path_count = 2},
	    {.action = hand_pose_action.action,
	     .paths = {"/user/hand/left/input/grip/pose", "/user/hand/right/input/grip/pose"},
	     .path_count = 2},
	    {.action = haptic_action.action,
	     .paths = {"/user/hand/left/output/haptic", "/user/hand/right/output/haptic"},
	     .path_count = 2},
	};
	if (!suggest_actions(instance, "/interaction_profiles/khr/simple_controller", simple_bindings,
	                     ARRAY_SIZE(simple_bindings)))
		return 1;

	struct Binding index_bindings[] = {
	    {.action = grab_action.action,
	     .paths = {"/user/hand/left/input/trigger/value", "/user/hand/right/input/trigger/value"},
	     .path_count = 2},
	    {.action = accelerate_action.action,
	     .paths = {"/user/hand/left/input/thumbstick/y", "/user/hand/right/input/thumbstick/y"},
	     .path_count = 2},
	    {.action = hand_pose_action.action,
	     .paths = {"/user/hand/left/input/grip/pose", "/user/hand/right/input/grip/pose"},
	     .path_count = 2},
	    {.action = haptic_action.action,
	     .paths = {"/user/hand/left/output/haptic", "/user/hand/right/output/haptic"},
	     .path_count = 2},
	};
	if (!suggest_actions(instance, "/interaction_profiles/valve/index_controller", index_bindings,
	                     ARRAY_SIZE(index_bindings)))
		return 1;


	if (ext.hand_tracking.system_supported) {
		if (!create_hand_trackers(instance, session, &ext.hand_tracking))
			return 1;
	}

	// TODO: should not be necessary, but is for SteamVR 1.16.4 (but not 1.15.x)
	glXMakeCurrent(graphics_binding_gl.xDisplay, graphics_binding_gl.glxDrawable,
	               graphics_binding_gl.glxContext);

	// Set up rendering (compile shaders, ...) before starting the session
	if (init_gl(view_count, vr_swapchains[SWAPCHAIN_PROJECTION].swapchain_lengths, &gl_rendering) !=
	    0) {
		printf("OpenGl setup failed!\n");
		return 1;
	}

	XrSessionActionSetsAttachInfo actionset_attach_info = {
	    .type = XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO,
	    .next = NULL,
	    .countActionSets = 1,
	    .actionSets = &gameplay_actionset};
	result = xrAttachSessionActionSets(session, &actionset_attach_info);
	if (!xr_check(instance, result, "failed to attach action set"))
		return 1;


	bool quit_renderloop = false;
	bool session_running = false; // to avoid beginning an already running session
	while (!quit_renderloop) {

		// --- Poll SDL for events so we can exit with esc
		SDL_Event sdl_event;
		while (SDL_PollEvent(&sdl_event)) {
			if (sdl_event.type == SDL_QUIT ||
			    (sdl_event.type == SDL_KEYDOWN && sdl_event.key.keysym.sym == SDLK_ESCAPE)) {
				printf("Requesting exit...\n");
				xrRequestExitSession(session);
			}
		}


		// for several session states we want to skip the render loop
		bool skip_renderloop = false;

		// --- Handle runtime Events
		// we do this before xrWaitFrame() so we can go idle or
		// break out of the main render loop as early as possible and don't have to
		// uselessly render or submit one. Calling xrWaitFrame commits you to
		// calling xrBeginFrame eventually.
		XrEventDataBuffer runtime_event = {.type = XR_TYPE_EVENT_DATA_BUFFER, .next = NULL};
		XrResult poll_result = xrPollEvent(instance, &runtime_event);
		while (poll_result == XR_SUCCESS) {
			switch (runtime_event.type) {
			case XR_TYPE_EVENT_DATA_EVENTS_LOST: {
				XrEventDataEventsLost* event = (XrEventDataEventsLost*)&runtime_event;
				printf("EVENT: %d events data lost!\n", event->lostEventCount);
				// do we care if the runtime loses events?
				break;
			}
			case XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING: {
				XrEventDataInstanceLossPending* event = (XrEventDataInstanceLossPending*)&runtime_event;
				printf("EVENT: instance loss pending at %lu! Destroying instance.\n", event->lossTime);
				quit_renderloop = true;
				continue;
			}
			case XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED: {
				XrEventDataSessionStateChanged* event = (XrEventDataSessionStateChanged*)&runtime_event;
				printf("EVENT: session state changed from %d to %d\n", state, event->state);
				state = event->state;

				// react to session state changes, see OpenXR spec 9.3 diagram
				switch (state) {

				// just keep polling, skip render loop
				case XR_SESSION_STATE_MAX_ENUM:
					// must be a bug, just keep polling
				case XR_SESSION_STATE_IDLE:
				case XR_SESSION_STATE_UNKNOWN: {
					skip_renderloop = true;
					break; // state handling switch
				}

				// do nothing, run render loop normally
				case XR_SESSION_STATE_FOCUSED:
				case XR_SESSION_STATE_SYNCHRONIZED:
				case XR_SESSION_STATE_VISIBLE: {
					skip_renderloop = false;
					break; // state handling switch
				}

				// begin session and then run render loop
				case XR_SESSION_STATE_READY: {
					// start session only if it is not running, i.e. not when we already called xrBeginSession
					// but the runtime did not switch to the next state yet
					if (!session_running) {
						XrSessionBeginInfo session_begin_info = {.type = XR_TYPE_SESSION_BEGIN_INFO,
						                                         .next = NULL,
						                                         .primaryViewConfigurationType = view_type};
						result = xrBeginSession(session, &session_begin_info);
						if (!xr_check(instance, result, "Failed to begin session!"))
							return 1;
						printf("Session started!\n");
						session_running = true;
					}
					skip_renderloop = false;
					break; // state handling switch
				}

				// end session, skip render loop, keep polling for next state change
				case XR_SESSION_STATE_STOPPING: {
					// end session only if it is running, i.e. not when we already called xrEndSession but the
					// runtime did not switch to the next state yet
					if (session_running) {
						result = xrEndSession(session);
						if (!xr_check(instance, result, "Failed to end session!"))
							return 1;
						session_running = false;
					}
					skip_renderloop = true;
					break; // state handling switch
				}

				// destroy session, skip render loop, exit render loop and quit
				case XR_SESSION_STATE_LOSS_PENDING:
				case XR_SESSION_STATE_EXITING:
					result = xrDestroySession(session);
					if (!xr_check(instance, result, "Failed to destroy session!"))
						return 1;
					quit_renderloop = true;
					skip_renderloop = true;
					break; // state handling switch
				}
				break;
			}
			case XR_TYPE_EVENT_DATA_REFERENCE_SPACE_CHANGE_PENDING: {
				printf("EVENT: reference space change pending!\n");
				XrEventDataReferenceSpaceChangePending* event =
				    (XrEventDataReferenceSpaceChangePending*)&runtime_event;
				(void)event;
				// TODO: do something
				break;
			}
			case XR_TYPE_EVENT_DATA_INTERACTION_PROFILE_CHANGED: {
				printf("EVENT: interaction profile changed!\n");
				XrEventDataInteractionProfileChanged* event =
				    (XrEventDataInteractionProfileChanged*)&runtime_event;
				(void)event;

				XrInteractionProfileState state = {.type = XR_TYPE_INTERACTION_PROFILE_STATE};

				for (int i = 0; i < 2; i++) {
					XrResult res = xrGetCurrentInteractionProfile(session, hand_paths[i], &state);
					if (!xr_check(instance, res, "Failed to get interaction profile for %d", i))
						continue;

					XrPath prof = state.interactionProfile;

					uint32_t strl;
					char profile_str[XR_MAX_PATH_LENGTH];
					res = xrPathToString(instance, prof, XR_MAX_PATH_LENGTH, &strl, profile_str);
					if (!xr_check(instance, res, "Failed to get interaction profile path str for %d", i))
						continue;

					printf("Event: Interaction profile changed for %d: %s\n", i, profile_str);
				}
				// TODO: do something
				break;
			}

			case XR_TYPE_EVENT_DATA_VISIBILITY_MASK_CHANGED_KHR: {
				printf("EVENT: visibility mask changed!!\n");
				XrEventDataVisibilityMaskChangedKHR* event =
				    (XrEventDataVisibilityMaskChangedKHR*)&runtime_event;
				(void)event;
				// this event is from an extension
				break;
			}
			case XR_TYPE_EVENT_DATA_PERF_SETTINGS_EXT: {
				printf("EVENT: perf settings!\n");
				XrEventDataPerfSettingsEXT* event = (XrEventDataPerfSettingsEXT*)&runtime_event;
				(void)event;
				// this event is from an extension
				break;
			}
			default: printf("Unhandled event type %d\n", runtime_event.type);
			}

			runtime_event.type = XR_TYPE_EVENT_DATA_BUFFER;
			poll_result = xrPollEvent(instance, &runtime_event);
		}
		if (poll_result == XR_EVENT_UNAVAILABLE) {
			// processed all events in the queue
		} else {
			printf("Failed to poll events!\n");
			break;
		}

		if (skip_renderloop) {
			continue;
		}

		// --- Wait for our turn to do head-pose dependent computation and render a frame
		XrFrameState frameState = {.type = XR_TYPE_FRAME_STATE, .next = NULL};
		XrFrameWaitInfo frameWaitInfo = {.type = XR_TYPE_FRAME_WAIT_INFO, .next = NULL};
		result = xrWaitFrame(session, &frameWaitInfo, &frameState);
		if (!xr_check(instance, result, "xrWaitFrame() was not successful, exiting..."))
			break;



		// --- Create projection matrices and view matrices for each eye
		XrViewLocateInfo view_locate_info = {.type = XR_TYPE_VIEW_LOCATE_INFO,
		                                     .next = NULL,
		                                     .viewConfigurationType =
		                                         XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO,
		                                     .displayTime = frameState.predictedDisplayTime,
		                                     .space = play_space};

		XrView views[view_count];
		for (uint32_t i = 0; i < view_count; i++) {
			views[i].type = XR_TYPE_VIEW;
			views[i].next = NULL;
		};

		XrViewState view_state = {.type = XR_TYPE_VIEW_STATE, .next = NULL};
		result = xrLocateViews(session, &view_locate_info, &view_state, view_count, &view_count, views);
		if (!xr_check(instance, result, "Could not locate views"))
			break;


		//! @todo Move this action processing to before xrWaitFrame, probably.
		const XrActiveActionSet active_actionsets[] = {
		    {.actionSet = gameplay_actionset, .subactionPath = XR_NULL_PATH}};

		XrActionsSyncInfo actions_sync_info = {
		    .type = XR_TYPE_ACTIONS_SYNC_INFO,
		    .countActiveActionSets = sizeof(active_actionsets) / sizeof(active_actionsets[0]),
		    .activeActionSets = active_actionsets,
		};
		result = xrSyncActions(session, &actions_sync_info);
		xr_check(instance, result, "failed to sync actions!");

		for (int i = 0; i < HAND_COUNT; i++) {
			if (!get_action_data(instance, session, &hand_pose_action, i, hand_paths, play_space,
			                     frameState.predictedDisplayTime, query_hand_velocities))
				printf("Failed to get hand pose action datafor hand %d\n", i);

			if (!get_action_data(instance, session, &grab_action, i, hand_paths, XR_NULL_HANDLE, 0,
			                     false))
				printf("Failed to get grab action data for hand %d\n", i);

			if (!get_action_data(instance, session, &accelerate_action, i, hand_paths, XR_NULL_HANDLE, 0,
			                     false))
				printf("Failed to get accelerate action data for hand %d\n", i);

			if (grab_action.states[i].float_.isActive &&
			    grab_action.states[i].float_.currentState > 0.75) {
				XrHapticVibration vibration = {.type = XR_TYPE_HAPTIC_VIBRATION,
				                               .next = NULL,
				                               .amplitude = 0.5,
				                               .duration = XR_MIN_HAPTIC_DURATION,
				                               .frequency = XR_FREQUENCY_UNSPECIFIED};

				XrHapticActionInfo haptic_action_info = {.type = XR_TYPE_HAPTIC_ACTION_INFO,
				                                         .next = NULL,
				                                         .action = haptic_action.action,
				                                         .subactionPath = hand_paths[i]};
				result = xrApplyHapticFeedback(session, &haptic_action_info,
				                               (const XrHapticBaseHeader*)&vibration);
				xr_check(instance, result, "failed to apply haptic feedback!");
				// printf("Sent haptic output to hand %d\n", i);
			}


			if (accelerate_action.states[i].float_.isActive &&
			    accelerate_action.states[i].float_.currentState != 0) {
				printf("Throttle value %d: changed %d: %f\n", i,
				       accelerate_action.states[i].float_.changedSinceLastSync,
				       accelerate_action.states[i].float_.currentState);
			}

			if (ext.hand_tracking.system_supported) {
				get_hand_tracking(instance, play_space, frameState.predictedDisplayTime,
				                  query_joint_velocities, &ext.hand_tracking, i);
			}
		};

		// --- Begin frame
		XrFrameBeginInfo frame_begin_info = {.type = XR_TYPE_FRAME_BEGIN_INFO, .next = NULL};

		result = xrBeginFrame(session, &frame_begin_info);
		if (!xr_check(instance, result, "failed to begin frame!"))
			break;

		// all swapchain release infos happen to be the same
		XrSwapchainImageReleaseInfo release_info = {.type = XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO,
		                                            .next = NULL};

		// render projection layer (once per view) and fill projection_views with the result
		for (uint32_t i = 0; i < view_count; i++) {
			XrMatrix4x4f projection_matrix;
			XrMatrix4x4f_CreateProjectionFov(&projection_matrix, GRAPHICS_OPENGL, views[i].fov,
			                                 gl_rendering.near_z, gl_rendering.far_z);

			XrMatrix4x4f view_matrix;
			XrMatrix4x4f_CreateViewMatrix(&view_matrix, &views[i].pose.position,
			                              &views[i].pose.orientation);


			uint32_t projection_index;
			if (!acquire_swapchain(instance, &vr_swapchains[SWAPCHAIN_PROJECTION], i, &projection_index))
				break;

			uint32_t depth_index = 0;
			if (ext.depth.supported) {
				if (!acquire_swapchain(instance, &vr_swapchains[SWAPCHAIN_DEPTH], i, &depth_index))
					break;
			}

			GLuint depth_image =
			    ext.depth.supported ? vr_swapchains[SWAPCHAIN_DEPTH].images[i][depth_index].image : 0;
			GLuint projection_image =
			    vr_swapchains[SWAPCHAIN_PROJECTION].images[i][projection_index].image;

			int w = viewconfig_views[i].recommendedImageRectWidth;
			int h = viewconfig_views[i].recommendedImageRectHeight;

			// TODO: should not be necessary, but is for SteamVR 1.16.4 (but not 1.15.x)
			glXMakeCurrent(graphics_binding_gl.xDisplay, graphics_binding_gl.glxDrawable,
			               graphics_binding_gl.glxContext);

			render_frame(w, h, &gl_rendering, projection_index, frameState.predictedDisplayTime, i,
			             hand_pose_action.pose_locations, &ext.hand_tracking, projection_matrix,
			             view_matrix, projection_image, ext.depth.supported, depth_image);

			result =
			    xrReleaseSwapchainImage(vr_swapchains[SWAPCHAIN_PROJECTION].swapchains[i], &release_info);
			if (!xr_check(instance, result, "failed to release swapchain image!"))
				break;

			if (ext.depth.supported) {
				result =
				    xrReleaseSwapchainImage(vr_swapchains[SWAPCHAIN_DEPTH].swapchains[i], &release_info);
				if (!xr_check(instance, result, "failed to release swapchain image!"))
					break;
			}

			projection_views[i].pose = views[i].pose;
			projection_views[i].fov = views[i].fov;
		}


		uint32_t quad_index = 0;
		if (!acquire_swapchain(instance, &quad_layer.swapchain, 0, &quad_index))
			break;

		render_quad(&gl_rendering, &quad_layer, quad_index, frameState.predictedDisplayTime);

		result = xrReleaseSwapchainImage(quad_layer.swapchain.swapchains[0], &release_info);
		if (!xr_check(instance, result, "failed to release swapchain image!"))
			break;


		// projectionLayers struct reused for every frame
		XrCompositionLayerProjection projection_layer = {
		    .type = XR_TYPE_COMPOSITION_LAYER_PROJECTION,
		    .next = NULL,
		    .layerFlags = 0,
		    .space = play_space,
		    .viewCount = view_count,
		    .views = projection_views,
		};


		float quad_aspect = (float)quad_layer.pixel_width / (float)quad_layer.pixel_height;
		float quad_width = 1.f;
		XrCompositionLayerQuad quad_comp_layer = {
		    .type = XR_TYPE_COMPOSITION_LAYER_QUAD,
		    .next = NULL,
		    .layerFlags = XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT,
		    .space = play_space,
		    .eyeVisibility = XR_EYE_VISIBILITY_BOTH,
		    .pose = {.orientation = {.x = 0.f, .y = 0.f, .z = 0.f, .w = 1.f},
		             .position = {.x = 1.5f, .y = .7f, .z = -1.5f}},
		    .size = {.width = quad_width, .height = quad_width / quad_aspect},
		    .subImage = {
		        .swapchain = quad_layer.swapchain.swapchains[0],
		        .imageRect = {
		            .offset = {.x = 0, .y = 0},
		            .extent = {.width = quad_layer.pixel_width, .height = quad_layer.pixel_height},
		        }}};


		int submitted_layer_count = 1;
		const XrCompositionLayerBaseHeader* submitted_layers[2] = {
		    (const XrCompositionLayerBaseHeader* const) & projection_layer};
		// already set projection_views[i].next = &depth.infos[i]; if depth supported


		submitted_layers[submitted_layer_count++] =
		    (const XrCompositionLayerBaseHeader* const) & quad_comp_layer;

		if ((view_state.viewStateFlags & XR_VIEW_STATE_ORIENTATION_VALID_BIT) == 0) {
			printf("Not submitting layers because orientation is invalid\n");
			submitted_layer_count = 0;
		}

		XrFrameEndInfo frameEndInfo = {.type = XR_TYPE_FRAME_END_INFO,
		                               .displayTime = frameState.predictedDisplayTime,
		                               .layerCount = submitted_layer_count,
		                               .layers = submitted_layers,
		                               .environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE,
		                               .next = NULL};
		result = xrEndFrame(session, &frameEndInfo);
		if (!xr_check(instance, result, "failed to end frame!"))
			break;
	}



	// --- Clean up after render loop quits
	for (uint32_t i = 0; i < view_count; i++) {
		free(vr_swapchains[SWAPCHAIN_PROJECTION].images[i]);
		if (ext.depth.supported) {
			free(vr_swapchains[SWAPCHAIN_DEPTH].images[i]);
		}

		glDeleteFramebuffers(vr_swapchains[SWAPCHAIN_PROJECTION].swapchain_lengths[i],
		                     gl_rendering.framebuffers[i]);
		free(gl_rendering.framebuffers[i]);
	}
	xrDestroyInstance(instance);

	free(viewconfig_views);
	free(projection_views);
	free(views);

	destroy_swapchain(&vr_swapchains[SWAPCHAIN_PROJECTION]);
	destroy_swapchain(&vr_swapchains[SWAPCHAIN_DEPTH]);
	free(gl_rendering.framebuffers);

	free(ext.depth.infos);

	printf("Cleaned up!\n");
}


// =============================================================================
// OpenGL rendering code
// =============================================================================

// A small header with functions for OpenGL math
#define MATH_3D_IMPLEMENTATION
#include "math_3d.h"

static SDL_Window* desktop_window;
static SDL_GLContext gl_context;

// don't need a gl loader for just one function, just load it ourselves'
PFNGLBLITNAMEDFRAMEBUFFERPROC _glBlitNamedFramebuffer;

void GLAPIENTRY
MessageCallback(GLenum source,
                GLenum type,
                GLuint id,
                GLenum severity,
                GLsizei length,
                const GLchar* message,
                const void* userParam)
{
	if (severity == GL_DEBUG_SEVERITY_NOTIFICATION)
		return;
	fprintf(stderr, "GL CALLBACK: %s type = 0x%x, severity = 0x%x, message = %s\n",
	        (type == GL_DEBUG_TYPE_ERROR ? "** GL ERROR **" : ""), type, severity, message);
}


#ifdef __linux__
bool
init_sdl_window(Display** xDisplay,
                uint32_t* visualid,
                GLXFBConfig* glxFBConfig,
                GLXDrawable* glxDrawable,
                GLXContext* glxContext,
                int w,
                int h)
{

	if (SDL_Init(SDL_INIT_VIDEO) < 0) {
		printf("Unable to initialize SDL");
		return false;
	}

	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);

	SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 0);


	/* Create our window centered at half the VR resolution */
	desktop_window =
	    SDL_CreateWindow("OpenXR Example", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, w / 2,
	                     h / 2, SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN);
	if (!desktop_window) {
		printf("Unable to create window");
		return false;
	}

	gl_context = SDL_GL_CreateContext(desktop_window);

	glEnable(GL_DEBUG_OUTPUT);
	glDebugMessageCallback(MessageCallback, 0);

	SDL_GL_SetSwapInterval(0);

	_glBlitNamedFramebuffer =
	    (PFNGLBLITNAMEDFRAMEBUFFERPROC)glXGetProcAddressARB((GLubyte*)"glBlitNamedFramebuffer");

	// HACK? OpenXR wants us to report these values, so "work around" SDL a
	// bit and get the underlying glx stuff. Does this still work when e.g.
	// SDL switches to xcb?
	*xDisplay = XOpenDisplay(NULL);
	*glxContext = glXGetCurrentContext();
	*glxDrawable = glXGetCurrentDrawable();

	return true;
}


static const char* vertexshader =
    "#version 330 core\n"
    "#extension GL_ARB_explicit_uniform_location : require\n"
    "layout(location = 0) in vec3 aPos;\n"
    "layout(location = 2) uniform mat4 model;\n"
    "layout(location = 3) uniform mat4 view;\n"
    "layout(location = 4) uniform mat4 proj;\n"
    "layout(location = 5) in vec2 aColor;\n"
    "out vec2 vertexColor;\n"
    "void main() {\n"
    "	gl_Position = proj * view * model * vec4(aPos.x, aPos.y, aPos.z, "
    "1.0);\n"
    "	vertexColor = aColor;\n"
    "}\n";

static const char* fragmentshader =
    "#version 330 core\n"
    "#extension GL_ARB_explicit_uniform_location : require\n"
    "layout(location = 0) out vec4 FragColor;\n"
    "layout(location = 1) uniform vec3 uniformColor;\n"
    "in vec2 vertexColor;\n"
    "void main() {\n"
    "	FragColor = (uniformColor.x < 0.01 && uniformColor.y < 0.01 && "
    "uniformColor.z < 0.01) ? vec4(vertexColor, 1.0, 1.0) : vec4(uniformColor, "
    "1.0);\n"
    "}\n";



int
init_gl(uint32_t view_count, uint32_t* swapchain_lengths, struct gl_renderer_t* gl_renderer)
{

	/* Allocate resources that we use for our own rendering.
	 * We will bind framebuffers to the runtime provided textures for rendering.
	 * For this, we create one framebuffer per OpenGL texture.
	 * This is not mandated by OpenXR, other ways to render to textures will work too.
	 */
	gl_renderer->framebuffers = malloc(sizeof(GLuint*) * view_count);
	for (uint32_t i = 0; i < view_count; i++) {
		gl_renderer->framebuffers[i] = malloc(sizeof(GLuint) * swapchain_lengths[i]);
		glGenFramebuffers(swapchain_lengths[i], gl_renderer->framebuffers[i]);
	}

	GLuint vertex_shader_id = glCreateShader(GL_VERTEX_SHADER);
	const GLchar* vertex_shader_source[1];
	vertex_shader_source[0] = vertexshader;
	// printf("Vertex Shader:\n%s\n", vertexShaderSource);
	glShaderSource(vertex_shader_id, 1, vertex_shader_source, NULL);
	glCompileShader(vertex_shader_id);
	int vertex_compile_res;
	glGetShaderiv(vertex_shader_id, GL_COMPILE_STATUS, &vertex_compile_res);
	if (!vertex_compile_res) {
		char info_log[512];
		glGetShaderInfoLog(vertex_shader_id, 512, NULL, info_log);
		printf("Vertex Shader failed to compile: %s\n", info_log);
		return 1;
	} else {
		printf("Successfully compiled vertex shader!\n");
	}

	GLuint fragment_shader_id = glCreateShader(GL_FRAGMENT_SHADER);
	const GLchar* fragment_shader_source[1];
	fragment_shader_source[0] = fragmentshader;
	glShaderSource(fragment_shader_id, 1, fragment_shader_source, NULL);
	glCompileShader(fragment_shader_id);
	int fragment_compile_res;
	glGetShaderiv(fragment_shader_id, GL_COMPILE_STATUS, &fragment_compile_res);
	if (!fragment_compile_res) {
		char info_log[512];
		glGetShaderInfoLog(fragment_shader_id, 512, NULL, info_log);
		printf("Fragment Shader failed to compile: %s\n", info_log);
		return 1;
	} else {
		printf("Successfully compiled fragment shader!\n");
	}

	gl_renderer->shader_program_id = glCreateProgram();
	glAttachShader(gl_renderer->shader_program_id, vertex_shader_id);
	glAttachShader(gl_renderer->shader_program_id, fragment_shader_id);
	glLinkProgram(gl_renderer->shader_program_id);
	GLint shader_program_res;
	glGetProgramiv(gl_renderer->shader_program_id, GL_LINK_STATUS, &shader_program_res);
	if (!shader_program_res) {
		char info_log[512];
		glGetProgramInfoLog(gl_renderer->shader_program_id, 512, NULL, info_log);
		printf("Shader Program failed to link: %s\n", info_log);
		return 1;
	} else {
		printf("Successfully linked shader program!\n");
	}

	glDeleteShader(vertex_shader_id);
	glDeleteShader(fragment_shader_id);

	float vertices[] = {-0.5f, -0.5f, -0.5f, 0.0f, 0.0f, 0.5f,  -0.5f, -0.5f, 1.0f, 0.0f,
	                    0.5f,  0.5f,  -0.5f, 1.0f, 1.0f, 0.5f,  0.5f,  -0.5f, 1.0f, 1.0f,
	                    -0.5f, 0.5f,  -0.5f, 0.0f, 1.0f, -0.5f, -0.5f, -0.5f, 0.0f, 0.0f,

	                    -0.5f, -0.5f, 0.5f,  0.0f, 0.0f, 0.5f,  -0.5f, 0.5f,  1.0f, 0.0f,
	                    0.5f,  0.5f,  0.5f,  1.0f, 1.0f, 0.5f,  0.5f,  0.5f,  1.0f, 1.0f,
	                    -0.5f, 0.5f,  0.5f,  0.0f, 1.0f, -0.5f, -0.5f, 0.5f,  0.0f, 0.0f,

	                    -0.5f, 0.5f,  0.5f,  1.0f, 0.0f, -0.5f, 0.5f,  -0.5f, 1.0f, 1.0f,
	                    -0.5f, -0.5f, -0.5f, 0.0f, 1.0f, -0.5f, -0.5f, -0.5f, 0.0f, 1.0f,
	                    -0.5f, -0.5f, 0.5f,  0.0f, 0.0f, -0.5f, 0.5f,  0.5f,  1.0f, 0.0f,

	                    0.5f,  0.5f,  0.5f,  1.0f, 0.0f, 0.5f,  0.5f,  -0.5f, 1.0f, 1.0f,
	                    0.5f,  -0.5f, -0.5f, 0.0f, 1.0f, 0.5f,  -0.5f, -0.5f, 0.0f, 1.0f,
	                    0.5f,  -0.5f, 0.5f,  0.0f, 0.0f, 0.5f,  0.5f,  0.5f,  1.0f, 0.0f,

	                    -0.5f, -0.5f, -0.5f, 0.0f, 1.0f, 0.5f,  -0.5f, -0.5f, 1.0f, 1.0f,
	                    0.5f,  -0.5f, 0.5f,  1.0f, 0.0f, 0.5f,  -0.5f, 0.5f,  1.0f, 0.0f,
	                    -0.5f, -0.5f, 0.5f,  0.0f, 0.0f, -0.5f, -0.5f, -0.5f, 0.0f, 1.0f,

	                    -0.5f, 0.5f,  -0.5f, 0.0f, 1.0f, 0.5f,  0.5f,  -0.5f, 1.0f, 1.0f,
	                    0.5f,  0.5f,  0.5f,  1.0f, 0.0f, 0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
	                    -0.5f, 0.5f,  0.5f,  0.0f, 0.0f, -0.5f, 0.5f,  -0.5f, 0.0f, 1.0f};

	GLuint VBOs[1];
	glGenBuffers(1, VBOs);

	glGenVertexArrays(1, &gl_renderer->VAO);

	glBindVertexArray(gl_renderer->VAO);
	glBindBuffer(GL_ARRAY_BUFFER, VBOs[0]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_DYNAMIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);

	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_DYNAMIC_DRAW);
	glVertexAttribPointer(5, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
	glEnableVertexAttribArray(5);

	glEnable(GL_DEPTH_TEST);

	return 0;
}

static void
render_block(XrVector3f* position, XrQuaternionf* orientation, XrVector3f* radi, int modelLoc)
{
	XrMatrix4x4f model_matrix;
	XrMatrix4x4f_CreateModelMatrix(&model_matrix, position, orientation, radi);
	glUniformMatrix4fv(modelLoc, 1, GL_FALSE, (float*)model_matrix.m);
	glDrawArrays(GL_TRIANGLES, 0, 36);
}

static void
render_cube(XrVector3f* position, XrQuaternionf* orientation, float cube_size, int modelLoc)
{
	XrVector3f s = {cube_size / 2., cube_size / 2., cube_size / 2.};
	render_block(position, orientation, &s, modelLoc);
}

void
render_rotated_cube(
    vec3_t position, float cube_size, float rotation, float* projection_matrix, int modelLoc)
{
	mat4_t rotationmatrix = m4_rotation_y(degrees_to_radians(rotation));
	mat4_t modelmatrix = m4_mul(m4_translation(position),
	                            m4_scaling(vec3(cube_size / 2., cube_size / 2., cube_size / 2.)));
	modelmatrix = m4_mul(modelmatrix, rotationmatrix);

	glUniformMatrix4fv(modelLoc, 1, GL_FALSE, (float*)modelmatrix.m);
	glDrawArrays(GL_TRIANGLES, 0, 36);
}

static float
vec3_mag(XrVector3f* vec)
{
	return sqrt(vec->x * vec->x + vec->y * vec->y + vec->z * vec->z);
}

static XrVector3f
vec3_norm(XrVector3f* vec)
{
	float mag = vec3_mag(vec);
	XrVector3f r = {.x = vec->x / mag, .y = vec->y / mag, .z = vec->z / mag};
	return r;
}

static void
visualize_velocity(XrPosef* base,
                   XrVector3f* linearVelocity,
                   XrVector3f* angularVelocity,
                   int modelLoc,
                   float size)
{
	float cube_radius = size / 2;
	float lin_len = vec3_mag(linearVelocity);
	float block_radius = lin_len / 2.;
	XrVector3f lin_direction = vec3_norm(linearVelocity);

#if 0
	for (float i = 0; i < lin_len / size; i++) {
		XrVector3f pos = base->position;
		pos.x += lin_direction.x * size * i;
		pos.y += lin_direction.y * size * i;
		pos.z += lin_direction.z * size * i;
		render_cube(&pos, &identity, cube_radius, modelLoc);
}
#endif


// linear velocity
#if 1
	{
		/* create matrix that translates lin_len / 2. in lin_direction (because
		 * block origin is in the middle), scales to lin_len in "z" direction and
		 * rotates in lin_direction. Used as model matrix for unit cube this renders
		 * a block of length lin_len in lin_direction starting at the base pose.
		 */
		vec3_t from = vec3(base->position.x + lin_direction.x * block_radius / 2.,
		                   base->position.y + lin_direction.y * block_radius / 2.,
		                   base->position.z + lin_direction.z * block_radius / 2.);
		vec3_t to = vec3(base->position.x + lin_direction.x, base->position.y + lin_direction.y,
		                 base->position.z + lin_direction.z);
		mat4_t look_at = m4_invert_affine(m4_look_at(from, to, vec3(0, 1, 0)));

		mat4_t scale = m4_scaling(vec3(cube_radius, cube_radius, block_radius));
		look_at = m4_mul(look_at, scale);

		glUniformMatrix4fv(modelLoc, 1, GL_FALSE, (float*)look_at.m);
		glDrawArrays(GL_TRIANGLES, 0, 36);
	}
#endif

// angular velocity - block is axis, length is velocity
#if 0
{
vec3_t from = vec3(0, 0, 0);
vec3_t to = vec3(angularVelocity->x / 2., angularVelocity->y / 2., angularVelocity->z / 2.0);
mat4_t look_at = m4_invert_affine(m4_look_at(from, to, vec3(0, 1, 0)));

float ang_len = vec3_mag(angularVelocity);
mat4_t scale = m4_scaling(vec3(cube_radius, cube_radius, ang_len / 2.));
look_at = m4_mul(look_at, scale);

vec3_t pos = vec3(base->position.x, base->position.y, base->position.z);
mat4_t model = m4_mul(m4_translation(pos), look_at);
glUniformMatrix4fv(modelLoc, 1, GL_FALSE, (float*)model.m);
glDrawArrays(GL_TRIANGLES, 0, 36);
}
#endif
}

void
render_frame(int w,
             int h,
             struct gl_renderer_t* gl_renderer,
             uint32_t projection_index,
             XrTime predictedDisplayTime,
             int view_index,
             XrSpaceLocation* hand_locations,
             struct hand_tracking_t* hand_tracking,
             XrMatrix4x4f projectionmatrix,
             XrMatrix4x4f viewmatrix,
             GLuint image,
             bool depth_supported,
             GLuint depthbuffer)
{
	GLuint framebuffer = gl_renderer->framebuffers[view_index][projection_index];
	glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);

	glViewport(0, 0, w, h);
	glScissor(0, 0, w, h);

	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, image, 0);
	if (depth_supported) {
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthbuffer, 0);
	} else {
		// TODO: need a depth attachment for depth test when rendering to fbo
	}

	glClearColor(.0f, 0.0f, 0.2f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


	glUseProgram(gl_renderer->shader_program_id);
	glBindVertexArray(gl_renderer->VAO);

	int modelLoc = glGetUniformLocation(gl_renderer->shader_program_id, "model");
	int colorLoc = glGetUniformLocation(gl_renderer->shader_program_id, "uniformColor");
	int viewLoc = glGetUniformLocation(gl_renderer->shader_program_id, "view");
	glUniformMatrix4fv(viewLoc, 1, GL_FALSE, (float*)viewmatrix.m);
	int projLoc = glGetUniformLocation(gl_renderer->shader_program_id, "proj");
	glUniformMatrix4fv(projLoc, 1, GL_FALSE, (float*)projectionmatrix.m);


	// render scene with 4 colorful cubes
	{
		// the special color value (0, 0, 0) will get replaced by some UV color in the shader
		glUniform3f(colorLoc, 0.0, 0.0, 0.0);

		double display_time_seconds = ((double)predictedDisplayTime) / (1000. * 1000. * 1000.);
		const float rotations_per_sec = .25;
		float angle = ((long)(display_time_seconds * 360. * rotations_per_sec)) % 360;

		float dist = 1.5f;
		float height = 0.5f;
		render_rotated_cube(vec3(0, height, -dist), .33f, angle, projectionmatrix.m, modelLoc);
		render_rotated_cube(vec3(0, height, dist), .33f, angle, projectionmatrix.m, modelLoc);
		render_rotated_cube(vec3(dist, height, 0), .33f, angle, projectionmatrix.m, modelLoc);
		render_rotated_cube(vec3(-dist, height, 0), .33f, angle, projectionmatrix.m, modelLoc);
	}

	// render controllers / hand joints
	for (int hand = 0; hand < 2; hand++) {
		if (hand == 0) {
			glUniform3f(colorLoc, 1.0, 0.5, 0.5);
		} else {
			glUniform3f(colorLoc, 0.5, 1.0, 0.5);
		}


		// if at least some joints had valid poses, draw them instead of controller blocks
		bool any_joints_valid = false;

		struct XrHandJointLocationsEXT* joint_locations = &hand_tracking->joint_locations[hand];
		if (joint_locations->isActive) {
			for (uint32_t i = 0; i < joint_locations->jointCount; i++) {
				struct XrHandJointLocationEXT* joint_location = &joint_locations->jointLocations[i];

				if (!(joint_location->locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT)) {
					// printf("Hand %d Joint %d: Position not valid\n", hand, i);
					continue;
				}

				float size = joint_location->radius;
				render_cube(&joint_location->pose.position, &joint_location->pose.orientation, size,
				            modelLoc);

				if (joint_locations->next != NULL) {
					// we set .next only to null or XrHandJointVelocitiesEXT in main
					XrHandJointVelocitiesEXT* vel = (XrHandJointVelocitiesEXT*)joint_locations->next;
					if ((vel->jointVelocities[i].velocityFlags & XR_SPACE_VELOCITY_LINEAR_VALID_BIT) != 0) {
						visualize_velocity(&joint_location->pose, &vel->jointVelocities[i].linearVelocity,
						                   &vel->jointVelocities[i].angularVelocity, modelLoc, 0.005);
					} else {
						printf("Joint velocities %d invalid\n", i);
					}
				}

				any_joints_valid = true;
			}
		}



		bool hand_location_valid =
		    //(spaceLocation[hand].locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) != 0 &&
		    (hand_locations[hand].locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT) != 0;

		// don't draw blocks unless we know their position'
		if (!hand_location_valid)
			continue;

		// the controller blocks itself are only drawn if we didn't draw hand joints'
		if (!any_joints_valid) {
			XrVector3f scale = {.x = .05f, .y = .05f, .z = .2f};
			render_block(&hand_locations[hand].pose.position, &hand_locations[hand].pose.orientation,
			             &scale, modelLoc);
		}

		// controller velocities are always drawn if available
		if (hand_locations[hand].next != NULL) {
			// we set .next only to null or XrSpaceVelocity in main
			XrSpaceVelocity* vel = (XrSpaceVelocity*)hand_locations[hand].next;
			if ((vel->velocityFlags & XR_SPACE_VELOCITY_LINEAR_VALID_BIT) != 0) {
				visualize_velocity(&hand_locations[hand].pose, &vel->linearVelocity, &vel->angularVelocity,
				                   modelLoc, 0.005);
			}
		}
	}


	// blit left eye to desktop window
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	if (view_index == 0) {
		_glBlitNamedFramebuffer((GLuint)framebuffer,             // readFramebuffer
		                        (GLuint)0,                       // backbuffer     // drawFramebuffer
		                        (GLint)0,                        // srcX0
		                        (GLint)0,                        // srcY0
		                        (GLint)w,                        // srcX1
		                        (GLint)h,                        // srcY1
		                        (GLint)0,                        // dstX0
		                        (GLint)0,                        // dstY0
		                        (GLint)w / 2,                    // dstX1
		                        (GLint)h / 2,                    // dstY1
		                        (GLbitfield)GL_COLOR_BUFFER_BIT, // mask
		                        (GLenum)GL_LINEAR);              // filter

		SDL_GL_SwapWindow(desktop_window);
	}
}

void
initialze_quad(struct gl_renderer_t* gl_rendering, struct quad_layer_t* quad)
{
	glGenTextures(1, &gl_rendering->quad.texture);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, gl_rendering->quad.texture);

	int w = quad->pixel_width;
	int h = quad->pixel_height;
	glViewport(0, 0, w, h);
	glScissor(0, 0, w, h);

	uint8_t* rgb = malloc(sizeof(uint8_t) * w * h * 4);
	for (int row = 0; row < h; row++) {
		for (int col = 0; col < w; col++) {
			uint8_t* base = &rgb[(row * w * 4 + col * 4)];
			*(base + 0) = (((float)row / (float)h)) * 255.;
			*(base + 1) = 0;
			*(base + 2) = 0;
			*(base + 3) = 255;

			if (abs(row - col) < 3) {
				*(base + 0) = 255.;
				*(base + 1) = 255;
				*(base + 2) = 255;
				*(base + 3) = 255;
			}

			if (abs((w - col) - (row)) < 3) {
				*(base + 0) = 0.;
				*(base + 1) = 0;
				*(base + 2) = 0;
				*(base + 3) = 255;
			}
		}
	}

	// TODO respect format
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, (GLsizei)w, (GLsizei)h, 0, GL_RGBA, GL_UNSIGNED_BYTE,
	             (GLvoid*)rgb);
	free(rgb);

	glGenFramebuffers(1, &gl_rendering->quad.fbo);
	glBindFramebuffer(GL_READ_FRAMEBUFFER, gl_rendering->quad.fbo);
	glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
	                       gl_rendering->quad.texture, 0);

	gl_rendering->quad.initialized = true;
}

void
render_quad(struct gl_renderer_t* gl_rendering,
            struct quad_layer_t* quad,
            uint32_t swapchain_index,
            XrTime predictedDisplayTime)
{
	if (!gl_rendering->quad.initialized) {
		printf("Creating Quad texture\n");
		initialze_quad(gl_rendering, quad);
	}

	glBindFramebuffer(GL_READ_FRAMEBUFFER, gl_rendering->quad.fbo);

	GLuint texture = quad->swapchain.images[0][swapchain_index].image;
	glBindTexture(GL_TEXTURE_2D, texture);
	glCopyTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 0, 0, quad->pixel_width, quad->pixel_height);

	glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
}


#endif
