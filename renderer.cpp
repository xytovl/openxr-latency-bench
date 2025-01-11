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

#include "renderer.h"
#include "vk/pipeline.h"

#include <map>
#include <vulkan/vulkan_enums.hpp>
#include <vulkan/vulkan_raii.hpp>
#include <vulkan/vulkan_structs.hpp>
#include <openxr/openxr.h>

extern const std::map<std::string, std::vector<uint32_t>> shaders;

struct push_constants
{
	float r, g, b, t;
};

renderer::renderer(vk::raii::Device & device, xr::session & session, vk::Extent2D image_size) :
        device(device),
        session(session),
        image_size(image_size),
        layout([](vk::raii::Device & device) {
	        vk::PushConstantRange push_constant_range{
	                .stageFlags = vk::ShaderStageFlagBits::eFragment,
	                .offset = 0,
	                .size = sizeof(push_constants),
	        };
	        return device.createPipelineLayout({
	                .pushConstantRangeCount = 1,
	                .pPushConstantRanges = &push_constant_range,
	        });
        }(device)),
        renderpass([](vk::raii::Device & device, vk::Format format) {
	        vk::AttachmentReference color_ref{
	                .attachment = 0,
	                .layout = vk::ImageLayout::eColorAttachmentOptimal,
	        };

	        vk::AttachmentDescription attachment{
	                .format = format,
	                .samples = vk::SampleCountFlagBits::e1,
	                .loadOp = vk::AttachmentLoadOp::eDontCare,
	                .storeOp = vk::AttachmentStoreOp::eStore,
	                .initialLayout = vk::ImageLayout::eColorAttachmentOptimal,
	                .finalLayout = vk::ImageLayout::eColorAttachmentOptimal,
	        };

	        vk::SubpassDescription subpass{
	                .pipelineBindPoint = vk::PipelineBindPoint::eGraphics,
	        };
	        subpass.setColorAttachments(color_ref);

	        vk::RenderPassCreateInfo renderpass_info;
	        renderpass_info.setAttachments(attachment);
	        renderpass_info.setSubpasses(subpass);

	        return vk::raii::RenderPass(device, renderpass_info);
        }(device, format)),
        pipeline([](vk::raii::Device & device, vk::raii::PipelineLayout & layout, vk::raii::RenderPass & renderpass, vk::Extent2D extent) {
	        auto & vert_spirv = shaders.at("noise.vert");
	        auto & frag_spirv = shaders.at("noise.frag");

	        vk::raii::ShaderModule vert_shader(
	                device,
	                {
	                        .codeSize = vert_spirv.size() * sizeof(uint32_t),
	                        .pCode = vert_spirv.data(),
	                });

	        vk::raii::ShaderModule frag_shader(
	                device,
	                {
	                        .codeSize = frag_spirv.size() * sizeof(uint32_t),
	                        .pCode = frag_spirv.data(),
	                });

	        vk::pipeline_builder pipeline_info{
	                .flags = {},
	                .Stages = {
	                        {
	                                .stage = vk::ShaderStageFlagBits::eVertex,
	                                .module = *vert_shader,
	                                .pName = "main",
	                        },
	                        {
	                                .stage = vk::ShaderStageFlagBits::eFragment,
	                                .module = *frag_shader,
	                                .pName = "main",
	                        },
	                },
	                .VertexBindingDescriptions = {},
	                .VertexAttributeDescriptions = {},
	                .InputAssemblyState = {{
	                        .topology = vk::PrimitiveTopology::eTriangleList,
	                }},
	                .Viewports = {{
	                        .x = 0,
	                        .y = 0,
	                        .width = (float)extent.width,
	                        .height = (float)extent.height,
	                        .minDepth = 0,
	                        .maxDepth = 1,
	                }},
	                .Scissors = {{
	                        .offset = {.x = 0, .y = 0},
	                        .extent = extent,
	                }},
	                .RasterizationState = {{
	                        .polygonMode = vk::PolygonMode::eFill,
	                        .lineWidth = 1,
	                }},
	                .MultisampleState = {{
	                        .rasterizationSamples = vk::SampleCountFlagBits::e1,
	                }},
	                .ColorBlendState = {.flags = {}},
	                .ColorBlendAttachments = {{
	                        .colorWriteMask = vk::ColorComponentFlagBits::eR | vk::ColorComponentFlagBits::eG | vk::ColorComponentFlagBits::eB | vk::ColorComponentFlagBits::eA,
	                }},
	                .layout = *layout,
	                .renderPass = *renderpass,
	                .subpass = 0,
	        };

	        return vk::raii::Pipeline{device, nullptr, pipeline_info};
        }(device, layout, renderpass, image_size)),
        space_local(session.create_reference_space(XR_REFERENCE_SPACE_TYPE_LOCAL)),
        space_view(session.create_reference_space(XR_REFERENCE_SPACE_TYPE_VIEW))
{
	for (auto & swapchain: swapchains)
	{
		swapchain = xr::swapchain(session, device, format, image_size.width, image_size.height);
	}

	t0 = 0;
}

vk::raii::Framebuffer & renderer::get_framebuffer(VkImage image)
{
	if (auto it = image_views.find(image); it != image_views.end())
		return it->second.second;

	vk::ImageViewCreateInfo iv_info{
	        .image = image,
	        .viewType = vk::ImageViewType::e2D,
	        .format = format,
	        .components = {
	                .r = vk::ComponentSwizzle::eIdentity,
	                .g = vk::ComponentSwizzle::eIdentity,
	                .b = vk::ComponentSwizzle::eIdentity,
	                .a = vk::ComponentSwizzle::eIdentity,
	        },
	        .subresourceRange = {
	                .aspectMask = vk::ImageAspectFlagBits::eColor,
	                .baseMipLevel = 0,
	                .levelCount = 1,
	                .baseArrayLayer = 0,
	                .layerCount = 1,
	        },
	};

	vk::raii::ImageView iv(device, iv_info);

	vk::FramebufferCreateInfo fb_create_info{
	        .renderPass = *renderpass,
	        .width = image_size.width,
	        .height = image_size.height,
	        .layers = 1,
	};
	fb_create_info.setAttachments(*iv);

	return image_views.emplace(std::make_pair(image, std::make_pair(std::move(iv), vk::raii::Framebuffer{device, fb_create_info}))).first->second.second;
}

static XrSpaceVelocity locate_space(XrSpace space, XrSpace reference, XrTime time)
{
	XrSpaceVelocity velocity{
	        .type = XR_TYPE_SPACE_VELOCITY,
	};

	XrSpaceLocation location{
	        .type = XR_TYPE_SPACE_LOCATION,
	        .next = &velocity,
	};

	xrLocateSpace(space, reference, time, &location);

	return velocity;
}

static bool test_space(XrSpaceVelocity a, float threshold_m_s)
{
	if (a.velocityFlags & XR_SPACE_VELOCITY_LINEAR_VALID_BIT)
	{
		const auto & v = a.linearVelocity;
		auto v2 = v.x * v.x + v.y * v.y + v.z * v.z;
		if (v2 > threshold_m_s * threshold_m_s)
			return true;
	}
	return false;
}

XrCompositionLayerProjection renderer::render(vk::CommandBuffer command_buffer, const XrFrameState & frame_state, XrSpace space_left, XrSpace space_right)
{
	auto [flags, views] = session.locate_views(XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, frame_state.predictedDisplayTime, space_local);

	auto vel_view = locate_space(space_view, space_local, frame_state.predictedDisplayTime);
	auto vel_left = locate_space(space_left, space_local, frame_state.predictedDisplayTime);
	auto vel_right = locate_space(space_right, space_local, frame_state.predictedDisplayTime);

	if (t0 == 0)
		t0 = frame_state.predictedDisplayTime / 1'000'000'000.0;

	push_constants pcs{
	        .r = 0.2f,
	        .g = test_space(vel_view, 0.005) or test_space(vel_left, 0.005) or test_space(vel_right, 0.005) ? 1.f : 0.2f,
	        .b = 0.2f,
	        .t = frame_state.predictedDisplayTime / 1'000'000'000.0f - t0,
	};

	for (int swapchain_index = 0; swapchain_index < 2; swapchain_index++)
	{
		int image_index = swapchains[swapchain_index].acquire();
		swapchains[swapchain_index].wait();

		auto image = swapchains[swapchain_index].images()[image_index].img;

		vk::RenderPassBeginInfo begin_info{
		        .renderPass = *renderpass,
		        .framebuffer = *get_framebuffer(image),
		        .renderArea = {
		                .offset = {0, 0},
		                .extent = image_size,
		        },
		};

		command_buffer.pipelineBarrier(
		        vk::PipelineStageFlagBits::eTopOfPipe,
		        vk::PipelineStageFlagBits::eColorAttachmentOutput,
		        {},
		        {},
		        {},
		        vk::ImageMemoryBarrier{
		                .srcAccessMask = {},
		                .dstAccessMask = vk::AccessFlagBits::eColorAttachmentWrite,
		                .oldLayout = vk::ImageLayout::eUndefined,
		                .newLayout = vk::ImageLayout::eColorAttachmentOptimal,
		                .image = image,
		                .subresourceRange = {
		                        .aspectMask = vk::ImageAspectFlagBits::eColor,
		                        .levelCount = 1,
		                        .layerCount = 1,
		                },
		        });

		command_buffer.beginRenderPass(begin_info, vk::SubpassContents::eInline);
		command_buffer.bindPipeline(vk::PipelineBindPoint::eGraphics, *pipeline);
		command_buffer.pushConstants(*layout, vk::ShaderStageFlagBits::eFragment, 0, sizeof(pcs), &pcs);
		command_buffer.draw(6, 1, 0, 0);
		command_buffer.endRenderPass();

		layer_views[swapchain_index] =
		        {
		                .type = XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW,
		                .pose = views[swapchain_index].pose,
		                .fov = views[swapchain_index].fov,
		                .subImage = {
		                        .swapchain = swapchains[swapchain_index],
		                        .imageRect = {
		                                .offset = {0, 0},
		                                .extent = {
		                                        swapchains[swapchain_index].width(),
		                                        swapchains[swapchain_index].height(),
		                                },
		                        },
		                },
		        };
	}

	return XrCompositionLayerProjection{
	        .type = XR_TYPE_COMPOSITION_LAYER_PROJECTION,
	        .layerFlags = XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT,
	        .space = space_local,
	        .viewCount = (uint32_t)layer_views.size(),
	        .views = layer_views.data(),
	};
}

void renderer::end_frame()
{
	for (int swapchain_index = 0; swapchain_index < 2; swapchain_index++)
	{
		swapchains[swapchain_index].release();
	}
}
