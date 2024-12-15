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

#include <system_error>
#include <vulkan/vulkan.h>

namespace vk
{
const std::error_category & error_category();
}

static inline VkResult check(VkResult result, const char * statement)
{
	if (result != VK_SUCCESS)
		throw std::system_error(result, vk::error_category(), statement);

	return result;
}

static inline VkResult check(VkResult result, const char * /*statement*/, const char * message)
{
	if (result != VK_SUCCESS)
		throw std::system_error(result, vk::error_category(), message);

	return result;
}

#define CHECK_VK(result, ...) check(result, #result __VA_OPT__(, ) __VA_ARGS__)
