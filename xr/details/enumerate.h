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

#ifdef __ANDROID__
#include <jni.h>
#endif

#include "utils/typename.h"
#include "xr/check.h"
#include <type_traits>
#include <vector>
#include <vulkan/vulkan.h>
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

namespace xr::details
{

template <typename T>
struct structure_traits_base
{
	static constexpr inline XrStructureType type = XR_TYPE_UNKNOWN;
	using base = T;
};

template <typename T>
struct structure_traits : structure_traits_base<T>
{};

template <>
struct structure_traits<XrViewConfigurationView> : structure_traits_base<XrViewConfigurationView>
{
	static constexpr inline XrStructureType type = XR_TYPE_VIEW_CONFIGURATION_VIEW;
};

template <>
struct structure_traits<XrView> : structure_traits_base<XrView>
{
	static constexpr inline XrStructureType type = XR_TYPE_VIEW;
};

template <>
struct structure_traits<XrApiLayerProperties> : structure_traits_base<XrApiLayerProperties>
{
	static constexpr inline XrStructureType type = XR_TYPE_API_LAYER_PROPERTIES;
};

template <>
struct structure_traits<XrExtensionProperties> : structure_traits_base<XrExtensionProperties>
{
	static constexpr inline XrStructureType type = XR_TYPE_EXTENSION_PROPERTIES;
};

template <>
struct structure_traits<XrSwapchainImageVulkanKHR> : structure_traits_base<XrSwapchainImageVulkanKHR>
{
	static constexpr inline XrStructureType type = XR_TYPE_SWAPCHAIN_IMAGE_VULKAN_KHR;
	using base = XrSwapchainImageBaseHeader;
};

template <typename T>
constexpr inline XrStructureType structure_type = structure_traits<T>::type;

template <typename T, typename F, typename... Args>
auto enumerate(F f, Args &&... args) -> auto
{
	using array_type = std::conditional_t<std::is_same_v<T, char>, std::string, std::vector<T>>;
	array_type array;
	uint32_t count;
	XrResult result;

	result = f(std::forward<Args>(args)..., 0, &count, nullptr);

	if (XR_SUCCEEDED(result))
	{
		if constexpr (std::is_same_v<T, char>)
		{
			array.resize(count - 1); // count includes the null terminator
		}
		else if constexpr (structure_type<T> == XR_TYPE_UNKNOWN)
		{
			array.resize(count);
		}
		else
		{
			T default_value{};
			default_value.type = structure_type<T>;
			array.resize(count, default_value);
		}
		result = f(std::forward<Args>(args)..., count, &count, reinterpret_cast<typename structure_traits<T>::base *>(array.data()));
	}

	if (!XR_SUCCEEDED(result))
	{
		throw std::system_error(result, xr::error_category(), "enumerating " + type_name<T>());
	}

	return array;
}

template <typename T, typename F, typename... Args>
auto enumerate2(F f, uint32_t count, Args &&... args) -> auto
{
	assert(count);
	using array_type = std::conditional_t<std::is_same_v<T, char>, std::string, std::vector<T>>;
	T default_value{};
	uint32_t size_offset = 0;
	if constexpr (std::is_same_v<T, char>)
	{
		size_offset = 1;
	}
	else if constexpr (structure_type<T> != XR_TYPE_UNKNOWN)
	{
		default_value.type = structure_type<T>;
	}
	array_type array(count - size_offset, default_value);

	XrResult result = f(std::forward<Args>(args)..., count, &count, reinterpret_cast<typename structure_traits<T>::base *>(array.data()));

	if (XR_SUCCEEDED(result))
	{
		assert(count >= size_offset);
		array.resize(count - size_offset, default_value);
		return array;
	}
	if (result == XR_ERROR_SIZE_INSUFFICIENT)
	{
		assert(count >= size_offset);
		array.resize(count - size_offset, default_value);
		result = f(std::forward<Args>(args)..., count, &count, reinterpret_cast<typename structure_traits<T>::base *>(array.data()));
	}

	if (!XR_SUCCEEDED(result))
	{
		throw std::system_error(result, xr::error_category(), "enumerating " + type_name<T>());
	}

	return array;
}

} // namespace xr::details
