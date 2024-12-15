/*
 * OpenXR latency bench
 * Copyright (C) 2022-2024  Guillaume Meunier <guillaume.meunier@centraliens.net>
 * Copyright (C) 2022-2024  Patrick Nicolas <patricknicolas@laposte.net>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include "xr/actionset.h"
#include "xr/instance.h"
#include "xr/session.h"
#include "xr/system.h"

#include <unordered_set>
#include <vector>
#include <string>

#include <vulkan/vulkan_raii.hpp>
#include <openxr/openxr.h>

struct application_info
{
	std::string name = "Unnamed application";
	int version = VK_MAKE_VERSION(1, 0, 0);
	XrFormFactor formfactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
	XrViewConfigurationType viewconfig = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
	XrVersion min_vulkan_version = XR_MAKE_VERSION(1, 1, 0);

#ifdef __ANDROID__
	android_app * native_app;
#endif
};

class application
{
	application_info app_info;

	// Vulkan stuff
	vk::raii::Context vk_context;
	vk::raii::Instance vk_instance = nullptr;
	vk::raii::PhysicalDevice vk_physical_device = nullptr;
	vk::raii::Device vk_device = nullptr;
	uint32_t vk_queue_family_index;
	vk::raii::Queue vk_queue = nullptr;
	vk::raii::CommandPool vk_cmdpool = nullptr;
	vk::PhysicalDeviceProperties physical_device_properties;

	std::vector<const char *> vk_device_extensions;

	std::mutex debug_report_mutex;
	static VkBool32 vulkan_debug_report_callback(VkDebugReportFlagsEXT flags,
	                                             VkDebugReportObjectTypeEXT objectType,
	                                             uint64_t object,
	                                             size_t location,
	                                             int32_t messageCode,
	                                             const char * pLayerPrefix,
	                                             const char * pMessage,
	                                             void * pUserData);
	std::unordered_set<uint64_t> debug_report_ignored_objects;
	std::unordered_map<uint64_t, std::string> debug_report_object_name;
#ifndef NDEBUG
	vk::raii::DebugReportCallbackEXT debug_report_callback = nullptr;
#endif

	xr::instance xr_instance;
	xr::system xr_system_id;
	xr::session xr_session;
	XrSessionState session_state = XR_SESSION_STATE_UNKNOWN;
	xr::actionset xr_actionset;
	std::vector<std::tuple<XrAction, XrActionType, std::string>> actions;

	xr::space space_view;
	xr::space space_local;
	xr::space space_left_grip;
	xr::space space_right_grip;

	std::vector<std::string> xr_extensions;

	bool session_running = false;
	bool session_focused = false;
	bool session_visible = false;

	void initialize();
	void initialize_vulkan();
	void initialize_actions();

	void session_state_changed(XrSessionState new_state, XrTime timestamp);
	void poll_events();
	void loop();

public:
	application(application_info info);
	void run();
};
