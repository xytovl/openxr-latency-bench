cmake_minimum_required(VERSION 3.25)
project(openxr-latency-bench VERSION 0.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include(CompileGLSL)
include(FetchContent)

if(ANDROID)
    set(ANDROID_NDK $ENV{ANDROID_NDK_ROOT})
    set(CMAKE_ANDROID_NDK $ENV{ANDROID_NDK_ROOT})
    include(AndroidNdkModules)
    android_ndk_import_module_native_app_glue()
endif()

FetchContent_Declare(spdlog        EXCLUDE_FROM_ALL SYSTEM URL https://github.com/gabime/spdlog/archive/refs/tags/v1.14.1.tar.gz)
FetchContent_Declare(openxr_loader EXCLUDE_FROM_ALL SYSTEM URL https://github.com/KhronosGroup/OpenXR-SDK/archive/refs/tags/release-1.0.34.tar.gz)

find_package(PkgConfig REQUIRED)
find_package(Vulkan REQUIRED)

if (ANDROID)
	FetchContent_MakeAvailable(spdlog)
	FetchContent_MakeAvailable(openxr_loader)
	add_library(OpenXR::openxr_loader ALIAS openxr_loader)
else()
	find_package(OpenXR 1.0.20 REQUIRED)
	find_package(spdlog REQUIRED)
endif()

if (ANDROID)
	add_library(openxr-latency-bench MODULE)
else()
	add_executable(openxr-latency-bench)
endif()

target_include_directories(openxr-latency-bench PRIVATE .)
target_compile_definitions(openxr-latency-bench PRIVATE XR_USE_GRAPHICS_API_VULKAN)
target_compile_definitions(openxr-latency-bench PUBLIC VULKAN_HPP_NO_STRUCT_CONSTRUCTORS)

target_link_libraries(openxr-latency-bench
	PUBLIC
	OpenXR::openxr_loader
	spdlog::spdlog 
	Vulkan::Vulkan
)
target_sources(openxr-latency-bench
	PRIVATE
	application.cpp
	renderer.cpp
	main.cpp
	vk/error_category.cpp
	xr/actionset.cpp
	xr/error_category.cpp
	xr/instance.cpp
	xr/session.cpp
	xr/swapchain.cpp
	xr/system.cpp
	xr/xr.cpp
)

bench_compile_glsl(openxr-latency-bench ${CMAKE_CURRENT_SOURCE_DIR}/noise.glsl)
