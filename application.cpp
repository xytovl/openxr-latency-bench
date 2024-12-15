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

#include "application.h"

#include "vk/check.h"
#include "xr/xr.h"

#include <atomic>
#include <chrono> // IWYU pragma: keep
#include <spdlog/spdlog.h>
#include <unordered_set>
#include <vulkan/vulkan_structs.hpp>
#include <openxr/openxr_platform.h>

#ifdef __ANDROID__
#include <android/native_activity.h>
#include <sys/system_properties.h>
#else
#include <signal.h>
#endif

using namespace std::chrono_literals;

namespace
{
std::atomic<bool> exit_requested = false;

struct interaction_profile
{
	std::string profile_name;
	std::vector<std::string> required_extensions;
	std::vector<std::string> input_sources;
	bool available;
};

static std::vector<interaction_profile> interaction_profiles{
        interaction_profile{
                "/interaction_profiles/khr/simple_controller",
                {},
                {
                        "/user/hand/left/output/haptic",
                        "/user/hand/right/output/haptic",

                        "/user/hand/left/input/grip/pose",
                        "/user/hand/left/input/aim/pose",

                        "/user/hand/right/input/grip/pose",
                        "/user/hand/right/input/aim/pose",

                        "/user/hand/left/input/menu/click",
                        "/user/hand/left/input/select/click",

                        "/user/hand/right/input/menu/click",
                        "/user/hand/right/input/select/click",

                }},
};

std::string make_xr_name(std::string name)
{
	// Generate a name suitable for a path component (see OpenXR spec §6.2)
	for (char & c: name)
	{
		if (!isalnum(c) && c != '-' && c != '_' && c != '.')
			c = '_';
		else
			c = tolower(c);
	}

	size_t pos = name.find_first_of("abcdefghijklmnopqrstuvwxyz");

	return name.substr(pos);
}

const std::pair<std::string_view, XrActionType> action_suffixes[] =
        {
                // clang-format off
		// From OpenXR spec 1.0.33, §6.3.2 Input subpaths

		// Standard components
		{"/click",      XR_ACTION_TYPE_BOOLEAN_INPUT},
		{"/touch",      XR_ACTION_TYPE_BOOLEAN_INPUT},
		{"/force",      XR_ACTION_TYPE_FLOAT_INPUT},
		{"/value",      XR_ACTION_TYPE_FLOAT_INPUT},
		{"/x",          XR_ACTION_TYPE_FLOAT_INPUT},
		{"/y",          XR_ACTION_TYPE_FLOAT_INPUT},
		{"/twist",      XR_ACTION_TYPE_FLOAT_INPUT},
		{"/pose",       XR_ACTION_TYPE_POSE_INPUT},

		// Standard 2D identifier, can be used without the /x and /y cmoponents
		{"/trackpad",   XR_ACTION_TYPE_VECTOR2F_INPUT},
		{"/thumbstick", XR_ACTION_TYPE_VECTOR2F_INPUT},
		{"/joystick",   XR_ACTION_TYPE_VECTOR2F_INPUT},
		{"/trackball",  XR_ACTION_TYPE_VECTOR2F_INPUT},

		// Output paths
		{"/haptic",     XR_ACTION_TYPE_VIBRATION_OUTPUT},
                // clang-format on
};

XrActionType guess_action_type(const std::string & name)
{
	for (const auto & [suffix, type]: action_suffixes)
	{
		if (name.ends_with(suffix))
			return type;
	}

	return XR_ACTION_TYPE_FLOAT_INPUT;
}
} // namespace

application::application(application_info info) :
        app_info(std::move(info))

{
#ifdef __ANDROID__
	app_info.native_app->userData = this;
	app_info.native_app->onAppCmd = [](android_app * app, int32_t cmd) {
		switch (cmd)
		{
			// There is no APP_CMD_CREATE. The ANativeActivity creates the
			// application thread from onCreate(). The application thread
			// then calls android_main().
			case APP_CMD_START:
				break;
			case APP_CMD_RESUME:
				static_cast<application *>(app->userData)->resumed = true;
				break;
			case APP_CMD_PAUSE:
				static_cast<application *>(app->userData)->resumed = false;
				break;
			case APP_CMD_STOP:
				break;
			case APP_CMD_DESTROY:
				static_cast<application *>(app->userData)->native_window = nullptr;
				break;
			case APP_CMD_INIT_WINDOW:
				static_cast<application *>(app->userData)->native_window = app->window;
				break;
			case APP_CMD_TERM_WINDOW:
				static_cast<application *>(app->userData)->native_window = nullptr;
				break;
		}
	};

	// Initialize the loader for this platform
	PFN_xrInitializeLoaderKHR initializeLoader = nullptr;
	if (XR_SUCCEEDED(xrGetInstanceProcAddr(XR_NULL_HANDLE, "xrInitializeLoaderKHR", (PFN_xrVoidFunction *)(&initializeLoader))))
	{
		XrLoaderInitInfoAndroidKHR loaderInitInfoAndroid{
		        .type = XR_TYPE_LOADER_INIT_INFO_ANDROID_KHR,
		        .next = nullptr,
		        .applicationVM = app_info.native_app->activity->vm,
		        .applicationContext = app_info.native_app->activity->clazz,
		};
		initializeLoader((const XrLoaderInitInfoBaseHeaderKHR *)&loaderInitInfoAndroid);
	}

#endif

	try
	{
		initialize();
	}
	catch (std::exception & e)
	{
		spdlog::error("Error during initialization: {}", e.what());
		throw;
	}
}

void application::initialize()
{
	// LogLayersAndExtensions
	assert(!xr_instance);

	// Optional extensions
	std::vector<std::string> opt_extensions;
	opt_extensions.push_back(XR_FB_DISPLAY_REFRESH_RATE_EXTENSION_NAME);

	for (const auto & i: xr::instance::extensions())
	{
		if (std::ranges::contains(opt_extensions, i.extensionName))
			xr_extensions.push_back(i.extensionName);
	}

	std::vector<const char *> extensions;
	for (const auto & i: xr_extensions)
	{
		extensions.push_back(i.c_str());
	}

#ifdef __ANDROID__
	xr_instance =
	        xr::instance(app_info.name, app_info.native_app->activity->vm, app_info.native_app->activity->clazz, extensions);
#else
	xr_instance = xr::instance(app_info.name, extensions);
#endif

	spdlog::info("Created OpenXR instance, runtime {}, version {}", xr_instance.get_runtime_name(), xr_instance.get_runtime_version());

	xr_system_id = xr::system(xr_instance, app_info.formfactor);
	spdlog::info("Created OpenXR system for form factor {}", xr::to_string(app_info.formfactor));

	// Log system properties
	XrSystemProperties properties = xr_system_id.properties();
	spdlog::info("OpenXR system properties:");
	spdlog::info("    Vendor ID: {:#x}", properties.vendorId);
	spdlog::info("    System name: {}", properties.systemName);
	spdlog::info("    Graphics properties:");
	spdlog::info("        Maximum swapchain image size: {}x{}", properties.graphicsProperties.maxSwapchainImageWidth, properties.graphicsProperties.maxSwapchainImageHeight);
	spdlog::info("        Maximum layer count: {}", properties.graphicsProperties.maxLayerCount);
	spdlog::info("    Tracking properties:");
	spdlog::info("        Orientation tracking: {}", (bool)properties.trackingProperties.orientationTracking);
	spdlog::info("        Position tracking: {}", (bool)properties.trackingProperties.positionTracking);

	initialize_vulkan();

	xr_session = xr::session(xr_instance, xr_system_id, vk_instance, vk_physical_device, vk_device, vk_queue_family_index);

	vk::CommandPoolCreateInfo cmdpool_create_info;
	cmdpool_create_info.queueFamilyIndex = vk_queue_family_index;
	cmdpool_create_info.flags = vk::CommandPoolCreateFlagBits::eResetCommandBuffer;

	vk_cmdpool = vk::raii::CommandPool{vk_device, cmdpool_create_info};

	initialize_actions();

	auto view_configs = xr_system_id.view_configuration_views(XrViewConfigurationType::XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO);

	r.emplace(vk_device, xr_session, vk::Extent2D{view_configs[0].recommendedImageRectWidth, view_configs[0].recommendedImageRectHeight});
}

void application::initialize_vulkan()
{
	auto graphics_requirements = xr_system_id.graphics_requirements();
	XrVersion vulkan_version = std::max(app_info.min_vulkan_version, graphics_requirements.minApiVersionSupported);
	spdlog::info("OpenXR runtime wants Vulkan {}", xr::to_string(graphics_requirements.minApiVersionSupported));
	spdlog::info("Requesting Vulkan {}", xr::to_string(vulkan_version));

	std::vector<const char *> layers;

	spdlog::info("Available Vulkan layers:");
	[[maybe_unused]] bool validation_layer_found = false;

	for (vk::LayerProperties & i: vk_context.enumerateInstanceLayerProperties())
	{
		spdlog::info("    {}", i.layerName.data());
		if (!strcmp(i.layerName, "VK_LAYER_KHRONOS_validation"))
		{
			validation_layer_found = true;
		}
	}
#ifndef NDEBUG
	if (validation_layer_found)
	{
		spdlog::info("Using Vulkan validation layer");
		layers.push_back("VK_LAYER_KHRONOS_validation");
	}
#endif

	std::vector<const char *> instance_extensions{};
	std::unordered_set<std::string_view> optional_device_extensions{};

#ifndef NDEBUG
	bool debug_report_found = false;
	bool debug_utils_found = false;
#endif
	spdlog::info("Available Vulkan instance extensions:");
	for (vk::ExtensionProperties & i: vk_context.enumerateInstanceExtensionProperties(nullptr))
	{
		spdlog::info("    {} (version {})", i.extensionName.data(), i.specVersion);

#ifndef NDEBUG
		if (!strcmp(i.extensionName, VK_EXT_DEBUG_REPORT_EXTENSION_NAME))
			debug_report_found = true;

		if (!strcmp(i.extensionName, VK_EXT_DEBUG_UTILS_EXTENSION_NAME))
			debug_utils_found = true;
#endif
	}

#ifndef NDEBUG
	if (debug_utils_found && debug_report_found)
	{
		// debug_extensions_found = true;
		// spdlog::info("Using debug extensions");
		instance_extensions.push_back(VK_EXT_DEBUG_REPORT_EXTENSION_NAME);
		// instance_extensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
	}
#endif

	vk_device_extensions.push_back(VK_KHR_PUSH_DESCRIPTOR_EXTENSION_NAME);

#ifdef __ANDROID__
	vk_device_extensions.push_back(VK_ANDROID_EXTERNAL_MEMORY_ANDROID_HARDWARE_BUFFER_EXTENSION_NAME);
	vk_device_extensions.push_back(VK_KHR_SAMPLER_YCBCR_CONVERSION_EXTENSION_NAME);
	vk_device_extensions.push_back(VK_KHR_EXTERNAL_MEMORY_EXTENSION_NAME);
	vk_device_extensions.push_back(VK_EXT_QUEUE_FAMILY_FOREIGN_EXTENSION_NAME);
	vk_device_extensions.push_back(VK_KHR_DEDICATED_ALLOCATION_EXTENSION_NAME);
	vk_device_extensions.push_back(VK_KHR_MAINTENANCE1_EXTENSION_NAME);
	vk_device_extensions.push_back(VK_KHR_BIND_MEMORY_2_EXTENSION_NAME);
	vk_device_extensions.push_back(VK_KHR_GET_MEMORY_REQUIREMENTS_2_EXTENSION_NAME);
	instance_extensions.push_back(VK_KHR_GET_PHYSICAL_DEVICE_PROPERTIES_2_EXTENSION_NAME);
	instance_extensions.push_back(VK_KHR_EXTERNAL_MEMORY_CAPABILITIES_EXTENSION_NAME);
#endif

	vk::ApplicationInfo application_info{
	        .pApplicationName = app_info.name.c_str(),
	        .applicationVersion = (uint32_t)app_info.version,
	        .pEngineName = "no engine",
	        .engineVersion = VK_MAKE_VERSION(1, 0, 0),
	        .apiVersion = VK_MAKE_API_VERSION(0, XR_VERSION_MAJOR(vulkan_version), XR_VERSION_MINOR(vulkan_version), 0),
	};

	vk::InstanceCreateInfo instance_create_info{
	        .pApplicationInfo = &application_info,
	};
	instance_create_info.setPEnabledLayerNames(layers);
	instance_create_info.setPEnabledExtensionNames(instance_extensions);

	XrVulkanInstanceCreateInfoKHR create_info{
	        .type = XR_TYPE_VULKAN_INSTANCE_CREATE_INFO_KHR,
	        .systemId = xr_system_id,
	        .createFlags = 0,
	        .pfnGetInstanceProcAddr = vkGetInstanceProcAddr,
	        .vulkanCreateInfo = &(VkInstanceCreateInfo &)instance_create_info,
	        .vulkanAllocator = nullptr,
	};

	auto xrCreateVulkanInstanceKHR =
	        xr_instance.get_proc<PFN_xrCreateVulkanInstanceKHR>("xrCreateVulkanInstanceKHR");

	VkResult vresult;
	VkInstance tmp;
	XrResult xresult = xrCreateVulkanInstanceKHR(xr_instance, &create_info, &tmp, &vresult);
	CHECK_VK(vresult, "xrCreateVulkanInstanceKHR");
	CHECK_XR(xresult, "xrCreateVulkanInstanceKHR");
	vk_instance = vk::raii::Instance(vk_context, tmp);

#ifndef NDEBUG
	if (debug_report_found)
	{
		vk::DebugReportCallbackCreateInfoEXT debug_report_info{
		        .flags = vk::DebugReportFlagBitsEXT::eInformation |
		                 vk::DebugReportFlagBitsEXT::eWarning |
		                 vk::DebugReportFlagBitsEXT::ePerformanceWarning |
		                 vk::DebugReportFlagBitsEXT::eError |
		                 vk::DebugReportFlagBitsEXT::eDebug,
		        .pfnCallback = vulkan_debug_report_callback,
		        .pUserData = this,
		};
		debug_report_callback = vk::raii::DebugReportCallbackEXT(vk_instance, debug_report_info);
	}
#endif

	vk_physical_device = xr_system_id.physical_device(vk_instance);
	physical_device_properties = vk_physical_device.getProperties();

	spdlog::info("Available Vulkan device extensions:");
	for (vk::ExtensionProperties & i: vk_physical_device.enumerateDeviceExtensionProperties())
	{
		spdlog::info("    {}", i.extensionName.data());
		if (auto it = optional_device_extensions.find(i.extensionName); it != optional_device_extensions.end())
			vk_device_extensions.push_back(it->data());
	}

	spdlog::info("Initializing Vulkan with device {}", physical_device_properties.deviceName.data());
	spdlog::info("    Vendor ID: 0x{:04x}", physical_device_properties.vendorID);
	spdlog::info("    Device ID: 0x{:04x}", physical_device_properties.deviceID);

	std::vector<vk::QueueFamilyProperties> queue_properties = vk_physical_device.getQueueFamilyProperties();

	vk_queue_family_index = -1;
	[[maybe_unused]] bool vk_queue_found = false;
	for (size_t i = 0; i < queue_properties.size(); i++)
	{
		if (queue_properties[i].queueFlags & vk::QueueFlagBits::eGraphics)
		{
			vk_queue_found = true;
			vk_queue_family_index = i;
			break;
		}
	}
	assert(vk_queue_found);

	float queuePriority = 0.0f;

	vk::DeviceQueueCreateInfo queueCreateInfo{
	        .queueFamilyIndex = vk_queue_family_index,
	        .queueCount = 1,
	        .pQueuePriorities = &queuePriority,
	};

	vk::DeviceCreateInfo device_create_info{
	        .queueCreateInfoCount = 1,
	        .pQueueCreateInfos = &queueCreateInfo,
	        .enabledExtensionCount = (uint32_t)vk_device_extensions.size(),
	        .ppEnabledExtensionNames = vk_device_extensions.data(),
	};

	vk_device = xr_system_id.create_device(vk_physical_device, device_create_info);

	vk_queue = vk_device.getQueue(vk_queue_family_index, 0);
}

void application::initialize_actions()
{
	spdlog::debug("Initializing actions");

	// Build an action set with all possible input sources
	std::vector<XrActionSet> action_sets;
	xr_actionset = xr::actionset(xr_instance, "all_actions", "All actions");
	action_sets.push_back(xr_actionset);

	std::unordered_map<std::string, std::vector<XrActionSuggestedBinding>> suggested_bindings;

	// Build the list of all possible input sources, without duplicates,
	// checking which profiles are supported by the runtime
	std::unordered_set<std::string> sources;
	for (auto & profile: interaction_profiles)
	{
		profile.available = std::ranges::all_of(
		        profile.required_extensions,
		        [&](const auto & ext) {
			        return std::ranges::contains(xr_extensions, ext);
		        });

		if (not profile.available)
			continue;

		suggested_bindings.emplace(profile.profile_name, std::vector<XrActionSuggestedBinding>{});

		sources.insert(profile.input_sources.begin(), profile.input_sources.end());
	}

	// For each possible input source, create a XrAction and add it to the suggested binding
	std::unordered_map<std::string, XrAction> actions_by_name;

	for (const std::string & name: sources)
	{
		std::string name_without_slashes = make_xr_name(name);

		XrActionType type = guess_action_type(name);

		auto a = xr_actionset.create_action(type, name_without_slashes);
		actions.emplace_back(a, type, name);
		actions_by_name.emplace(name, a);

		if (name == "/user/hand/left/input/grip/pose")
			space_left_grip = xr_session.create_action_space(a);
		else if (name == "/user/hand/right/input/grip/pose")
			space_right_grip = xr_session.create_action_space(a);
	}

	// Suggest bindings for all supported controllers
	for (const auto & profile: interaction_profiles)
	{
		// Skip unavailable interaction profiles
		if (not profile.available)
			continue;

		std::vector<XrActionSuggestedBinding> & xr_bindings = suggested_bindings[profile.profile_name];

		for (const auto & name: profile.input_sources)
		{
			xr_bindings.push_back({actions_by_name[name], xr_instance.string_to_path(name)});
		}

		try
		{
			xr_instance.suggest_bindings(profile.profile_name, xr_bindings);
		}
		catch (...)
		{
			// Ignore errors
		}
	}

	xr_session.attach_actionsets(action_sets);
}

VkBool32 application::vulkan_debug_report_callback(
        VkDebugReportFlagsEXT flags,
        VkDebugReportObjectTypeEXT objectType,
        uint64_t object,
        size_t location,
        int32_t messageCode,
        const char * pLayerPrefix,
        const char * pMessage,
        void * pUserData)
{
	auto instance = (application *)pUserData;
	std::lock_guard lock(instance->debug_report_mutex);
	if (instance->debug_report_ignored_objects.contains(object))
		return VK_FALSE;

	spdlog::level::level_enum level = spdlog::level::info;

	if (flags & VK_DEBUG_REPORT_INFORMATION_BIT_EXT)
	{
		level = spdlog::level::info;
	}
	else if (flags & (VK_DEBUG_REPORT_WARNING_BIT_EXT | VK_DEBUG_REPORT_PERFORMANCE_WARNING_BIT_EXT))
	{
		level = spdlog::level::warn;
	}
	else if (flags & VK_DEBUG_REPORT_ERROR_BIT_EXT)
	{
		level = spdlog::level::err;
	}
	else if (flags & VK_DEBUG_REPORT_DEBUG_BIT_EXT)
	{
		level = spdlog::level::debug;
	}

	// for(const std::string& s: utils::split(pMessage, "|"))
	// spdlog::log(level, s);
	spdlog::log(level, pMessage);

	auto it = instance->debug_report_object_name.find(object);
	if (it != instance->debug_report_object_name.end())
	{
		spdlog::log(level, "{:#016x}: {}", object, it->second);
	}

	return VK_FALSE;
}

#ifdef __ANDROID__
void application::run()
{
	auto application_thread = utils::named_thread("application_thread", [&]() {
		setup_jni();

		while (!is_exit_requested())
		{
			try
			{
				loop();
			}
			catch (std::exception & e)
			{
				spdlog::error("Caught exception in application_thread: \"{}\"", e.what());
				exit_requested = true;
			}
			catch (...)
			{
				spdlog::error("Caught unknown exception in application_thread");
				exit_requested = true;
			}
		}
	});

	// Read all pending events.
	while (!exit_requested)
	{
		int events;
		struct android_poll_source * source;

		// TODO signal with a file descriptor instead of a 100ms timeout
		while (ALooper_pollOnce(100, nullptr, &events, (void **)&source) >= 0)
		{
			// Process this event.
			if (source != nullptr)
				source->process(app_info.native_app, source);
		}

		if (app_info.native_app->destroyRequested)
		{
			exit_requested = true;
		}
	}

	application_thread.join();
}
#else
void application::run()
{
	struct sigaction act
	{};
	act.sa_handler = [](int) {
		exit_requested = true;
	};
	sigaction(SIGINT, &act, nullptr);

	while (not exit_requested)
	{
		loop();
	}
}
#endif

void application::session_state_changed(XrSessionState new_state, XrTime timestamp)
{
	// See HandleSessionStateChangedEvent
	spdlog::info("Session state changed at timestamp {}: {} => {}", timestamp, xr::to_string(session_state), xr::to_string(new_state));
	session_state = new_state;

	switch (new_state)
	{
		case XR_SESSION_STATE_READY:
			xr_session.begin_session(app_info.viewconfig);
			session_running = true;
			break;

		case XR_SESSION_STATE_SYNCHRONIZED:
			session_visible = false;
			session_focused = false;
			break;

		case XR_SESSION_STATE_VISIBLE:
			session_visible = true;
			session_focused = false;
			break;

		case XR_SESSION_STATE_FOCUSED:
			session_visible = true;
			session_focused = true;
			break;

		case XR_SESSION_STATE_STOPPING:
			session_visible = false;
			session_focused = false;
			xr_session.end_session();
			session_running = false;
			break;

		case XR_SESSION_STATE_EXITING:
			exit_requested = true;
			break;

		case XR_SESSION_STATE_LOSS_PENDING:
			exit_requested = true;
			break;
		default:
			break;
	}
}

void application::poll_events()
{
	xr::event e;
	while (xr_instance.poll_event(e))
	{
		switch (e.header.type)
		{
			case XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING: {
				exit_requested = true;
			}
			break;
			case XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED: {
				if (e.state_changed.session == xr_session)
					session_state_changed(e.state_changed.state, e.state_changed.time);
				else
					spdlog::error("Received XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED for unknown "
					              "session");
			}
			break;
			case XR_TYPE_EVENT_DATA_DISPLAY_REFRESH_RATE_CHANGED_FB: {
				spdlog::info("Refresh rate changed from {} to {}",
				             e.refresh_rate_changed.fromDisplayRefreshRate,
				             e.refresh_rate_changed.toDisplayRefreshRate);
			}
			break;
			default:
				spdlog::info("Received event type {}", xr::to_string(e.header.type));
				break;
		}
	}
}

void application::loop()
{
	poll_events();

	if (not session_running)
	{
		// Throttle loop since xrWaitFrame won't be called.
		std::this_thread::sleep_for(250ms);
	}
	else
	{
		xr_session.sync_actions(xr_actionset);
		XrFrameState framestate = xr_session.wait_frame();

		if (not framestate.shouldRender)
		{
			xr_session.begin_frame();
			xr_session.end_frame(framestate.predictedDisplayTime, {});
			return;
		}

		vk::CommandBuffer cmd_buf = vk_device.allocateCommandBuffers(vk::CommandBufferAllocateInfo{.commandPool = *vk_cmdpool, .commandBufferCount = 1})[0].release();

		xr_session.begin_frame();

		cmd_buf.begin(vk::CommandBufferBeginInfo{.flags = vk::CommandBufferUsageFlagBits::eOneTimeSubmit});
		auto layer = r->render(cmd_buf, framestate);

		std::vector<XrCompositionLayerBaseHeader*> layers;
		layers.push_back(reinterpret_cast<XrCompositionLayerBaseHeader*>(&layer));

		cmd_buf.end();

		vk_queue.submit(vk::SubmitInfo{
			.commandBufferCount = 1,
			.pCommandBuffers = &cmd_buf
		});

		r->end_frame();

		xr_session.end_frame(framestate.predictedDisplayPeriod, layers);

		vk_device.waitIdle();
	}
}
