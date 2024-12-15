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

#include "vk/check.h"
#include <system_error>
#include <vulkan/vulkan.hpp>
#include <vulkan/vulkan_to_string.hpp>

namespace
{
struct : std::error_category
{
	const char * name() const noexcept override
	{
		return "vulkan";
	}

	std::string message(int condition) const override
	{
		return vk::to_string(static_cast<vk::Result>(condition));
	}
} vulkan_error_category;
} // namespace

const std::error_category & vk::error_category()
{
	return vulkan_error_category;
}
