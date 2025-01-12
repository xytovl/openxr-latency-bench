/*
 * OpenXR latency bench
 * Copyright (C) 2024  Guillaume Meunier <guillaume.meunier@centraliens.net>
 * Copyright (C) 2024  Patrick Nicolas <patricknicolas@laposte.net>
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

#include "xr/session.h"
#include "xr/swapchain.h"
#include <unordered_map>
#include <vulkan/vulkan_raii.hpp>
#include <openxr/openxr.h>

class renderer
{
	vk::raii::Device & device;
	xr::session & session;
	vk::Extent2D image_size;
	const vk::Format format;

	vk::raii::PipelineLayout layout;
	vk::raii::RenderPass renderpass;
	vk::raii::Pipeline pipeline;

	xr::space space_local;
	xr::space space_view;
	std::array<xr::swapchain, 2> swapchains;
	std::array<XrCompositionLayerProjectionView, 2> layer_views;

	std::unordered_map<VkImage, std::pair<vk::raii::ImageView, vk::raii::Framebuffer>> image_views;

	vk::raii::Framebuffer & get_framebuffer(VkImage image);

	float t0;

public:
	renderer(vk::raii::Device & device, xr::session & session, vk::Extent2D image_size);

	XrCompositionLayerProjection render(vk::CommandBuffer command_buffer, const XrFrameState & frame_state, XrSpace space_left, XrSpace space_right);
	void end_frame();
};
