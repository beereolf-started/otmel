// Путь отрисовки Vulkan 1.3 (Р73). Единственное место, где есть типы и вызовы Vulkan (Р8). Поведение - как у пути
// OpenGL (gl_backend.cpp): те же шейдеры (вторая сборка .vk.spv), те же проходы - непрозрачное, копия глубины, вода.
// Отличия:
//  - сетки лежат отрезками в больших блоках видеопамяти (64 МБ, свой учёт свободных мест), а не буфером на сетку;
//  - выгрузка - копия в общий буфер кадра и запись копирования в команды того же кадра: главный поток не ждёт драйвер
//    (в OpenGL создание буфера под сетку подвисало на драйвере - рывки при выгрузке леса);
//  - два кадра в полёте: пока видеокарта рисует один, процессор записывает следующий; удаление - через кадр.
#include <vulkan/vulkan.h>
#include <SDL3/SDL.h>
#include <SDL3/SDL_vulkan.h>

#include <algorithm>
#include <cstring>
#include <fstream>
#include <functional>
#include <iterator>
#include <map>
#include <string>
#include <vector>

#include <glm/matrix.hpp>

#include "journal.h"
#include "render/render.h"

namespace render {
namespace {

constexpr int FRAMES = 2;                     // кадров в полёте
constexpr int MAX_QUADS = 1 << 17;            // как в OpenGL: больше прямоугольников в куске не бывает
constexpr int TILE = 32, TILE_VERTS = 33;
constexpr int QUAD_INDICES = (TILE / 2) * (TILE / 2) * 6;
constexpr VkSampleCountFlagBits SAMPLES = VK_SAMPLE_COUNT_4_BIT;   // сглаживание MSAA ×4
constexpr VkFormat DEPTH = VK_FORMAT_D32_SFLOAT;                    // обратная глубина в 32-битном float (Р20)
constexpr VkDeviceSize BLOCK = 64ull << 20;   // блок видеопамяти под сетки
constexpr VkDeviceSize STAGE = 16ull << 20;   // кусок общего буфера выгрузки

bool vkOk(VkResult r, const char* what) {
    if (r == VK_SUCCESS) return true;
    journal::error("vulkan", "{}: ошибка {}", what, static_cast<int>(r));
    return false;
}

// Сообщения слоя проверки (OTMEL_VK_VALIDATION=1) - в наш журнал
VKAPI_ATTR VkBool32 VKAPI_CALL onVkMessage(VkDebugUtilsMessageSeverityFlagBitsEXT sev, VkDebugUtilsMessageTypeFlagsEXT,
                                           const VkDebugUtilsMessengerCallbackDataEXT* d, void*) {
    if (sev >= VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT) journal::error("vulkan", "{}", d->pMessage);
    else if (sev >= VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT) journal::warn("vulkan", "{}", d->pMessage);
    return VK_FALSE;
}

// std140: всё по 16 байт (тот же блок Frame, что у OpenGL)
struct FrameUbo {
    float viewProj[16];
    float sunDir[4];
    float eye[4];
    float voxelZone[4];
    float fade[4];
    float sky[4];
    float flow[4];
    float time[4];
    float simArea[4];
    float invViewProj[16];
    int32_t originV[4];
    float originOld[4];
};
// push-константы шейдеров (#ifdef VULKAN): кусок и плитка - по 48 байт, мелкая вода - 16
struct PushChunk { int32_t chunk[4], fineOff[4], flags[4]; };
struct PushTile { int32_t i0[4]; float f0[4]; int32_t i1[4]; };
constexpr uint32_t PUSH_BYTES = 64;
constexpr VkShaderStageFlags PUSH_STAGES = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;

struct Image {
    VkImage img = VK_NULL_HANDLE;
    VkDeviceMemory mem = VK_NULL_HANDLE;
    VkImageView view = VK_NULL_HANDLE;
    int w = 0, h = 0;
};
// буфер, видимый процессору (выгрузка, кадр, брызги), - отображён в память насовсем
struct HostBuf {
    VkBuffer buf = VK_NULL_HANDLE;
    VkDeviceMemory mem = VK_NULL_HANDLE;
    uint8_t* map = nullptr;
    VkDeviceSize cap = 0, used = 0;
};
// отрезок блока видеопамяти
struct Alloc { uint32_t block = UINT32_MAX; VkDeviceSize offset = 0, size = 0; bool valid() const { return block != UINT32_MAX; } };

class VKBackend final : public Backend {
public:
    explicit VKBackend(SDL_Window* w) : window_(w) {}
    ~VKBackend() override { shutdown(); }

    bool init(const std::string& shaderDir) {
        if (!createDevice()) return false;
        if (!createSwapchain()) return false;
        if (!createLayouts()) return false;
        if (!createPipelines(shaderDir)) return false;
        for (Slot& s : slots_) {
            VkCommandPoolCreateInfo pci{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
            pci.queueFamilyIndex = family_;
            if (!vkOk(vkCreateCommandPool(dev_, &pci, nullptr, &s.pool), "vkCreateCommandPool")) return false;
            VkCommandBufferAllocateInfo ai{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
            ai.commandPool = s.pool;
            ai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
            ai.commandBufferCount = 1;
            vkAllocateCommandBuffers(dev_, &ai, &s.cmd);
            VkFenceCreateInfo fci{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
            fci.flags = VK_FENCE_CREATE_SIGNALED_BIT;
            vkCreateFence(dev_, &fci, nullptr, &s.fence);
            VkSemaphoreCreateInfo sci{VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO};
            vkCreateSemaphore(dev_, &sci, nullptr, &s.acquired);
            s.ubo = makeHost(sizeof(FrameUbo), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);
            VkDescriptorSetLayout layouts[2] = {setLayout_, setLayout_};
            VkDescriptorSetAllocateInfo dai{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
            dai.descriptorPool = descPool_;
            dai.descriptorSetCount = 2;
            dai.pSetLayouts = layouts;
            VkDescriptorSet sets[2];
            if (!vkOk(vkAllocateDescriptorSets(dev_, &dai, sets), "vkAllocateDescriptorSets")) return false;
            s.setOpaque = sets[0];
            s.setWater = sets[1];
        }
        for (Dyn* d : {&spray_, &dyn_, &fineBlocks_})
            for (HostBuf& b : d->per) b = makeHost(64 << 10, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);

        VkSamplerCreateInfo smp{VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};
        smp.magFilter = smp.minFilter = VK_FILTER_LINEAR;
        smp.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
        smp.addressModeU = smp.addressModeV = smp.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        vkCreateSampler(dev_, &smp, nullptr, &linear_);
        smp.magFilter = smp.minFilter = VK_FILTER_NEAREST;
        vkCreateSampler(dev_, &smp, nullptr, &nearest_);

        ensureFrame();
        // Индексы общие для всех кусков: прямоугольник k - вершины 4k..4k+3, два треугольника
        std::vector<uint32_t> idx(static_cast<size_t>(MAX_QUADS) * 6);
        for (uint32_t q = 0; q < MAX_QUADS; ++q) {
            const uint32_t b = q * 4;
            uint32_t* i = &idx[q * 6];
            i[0] = b; i[1] = b + 1; i[2] = b + 2; i[3] = b; i[4] = b + 2; i[5] = b + 3;
        }
        quadIdx_ = uploadDevice(idx.data(), idx.size() * 4);
        // Индексы плитки - по четвертям подряд, диагональ (i, j)-(i+1, j+1), обход как в OpenGL
        std::vector<uint16_t> tidx;
        tidx.reserve(4 * QUAD_INDICES);
        for (int q = 0; q < 4; ++q)
            for (int j = (q >> 1) * 16; j < (q >> 1) * 16 + 16; ++j)
                for (int i = (q & 1) * 16; i < (q & 1) * 16 + 16; ++i) {
                    const auto v = [](int a, int b) { return uint16_t(b * TILE_VERTS + a); };
                    const uint16_t a = v(i, j), b = v(i, j + 1), c = v(i + 1, j + 1), d = v(i + 1, j);
                    tidx.insert(tidx.end(), {a, b, c, a, c, d});
                }
        tileIdx_ = uploadDevice(tidx.data(), tidx.size() * 2);

        // заглушки: пока нет мелкой воды - текстуры 1×1 из нуля (у шейдера привязки должны быть всегда)
        const float zf[2] = {0, 0};
        const uint8_t zu = 0;
        dummyFloat_ = makeImage(1, 1, VK_FORMAT_R32G32_SFLOAT, VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT,
                                VK_SAMPLE_COUNT_1_BIT, VK_IMAGE_ASPECT_COLOR_BIT);
        dummyUint_ = makeImage(1, 1, VK_FORMAT_R8_UINT, VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT,
                               VK_SAMPLE_COUNT_1_BIT, VK_IMAGE_ASPECT_COLOR_BIT);
        uploadImage(dummyFloat_, zf, sizeof(zf));
        uploadImage(dummyUint_, &zu, 1);
        meshes_.push_back({});   // номер 0 - «нет сетки»
        updateVoxelMask(1, {&zu, 1});
        updateFlowField(1, {zf, 2});
        return true;
    }

    MeshId uploadChunkMesh(std::span<const uint32_t> v, int waterQuads) override {
        if (v.empty()) return NO_MESH;
        return store(v, static_cast<uint32_t>(v.size() / 4), static_cast<uint32_t>(waterQuads));
    }
    MeshId uploadTile(std::span<const uint32_t> words, bool water) override { return store(words, 0, water ? 1 : 0); }

    void destroyMesh(MeshId id) override {
        if (id == NO_MESH || id >= meshes_.size() || !meshes_[id].a.valid()) return;
        ensureFrame();
        const Alloc a = meshes_[id].a;
        later([this, a] { freeDevice(a); });
        meshes_[id] = {};
        free_.push_back(id);
    }

    void setDebug(int mode) override { debug_ = mode; }
    void setVsync(bool on) override { if (vsync_ != on) { vsync_ = on; swapDirty_ = true; } }

    void updateVoxelMask(int side, std::span<const uint8_t> mask) override {
        ensureFrame();
        if (side != maskSide_) {
            drop(maskTex_);
            maskTex_ = makeImage(side, side, VK_FORMAT_R8_UINT, VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT,
                                 VK_SAMPLE_COUNT_1_BIT, VK_IMAGE_ASPECT_COLOR_BIT);
            maskSide_ = side;
        }
        uploadImage(maskTex_, mask.data(), static_cast<size_t>(side) * side);
    }

    void updateFlowField(int side, std::span<const float> v) override {
        ensureFrame();
        if (side != flowSide_) {
            drop(flowTex_);
            flowTex_ = makeImage(side, side, VK_FORMAT_R32G32_SFLOAT, VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT,
                                 VK_SAMPLE_COUNT_1_BIT, VK_IMAGE_ASPECT_COLOR_BIT);
            flowSide_ = side;
        }
        uploadImage(flowTex_, v.data(), static_cast<size_t>(side) * side * 8);
    }

    void setFineWater(glm::ivec2 origin, int side, std::span<const float> data, std::span<const uint8_t> mask,
                      std::span<const int32_t> blocks) override {
        ensureFrame();
        if (side != fineSide_) {
            drop(fineTex_);
            drop(fineMask_);
            fineTex_ = makeImage(side + 2, side + 2, VK_FORMAT_R32G32_SFLOAT, VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT,
                                 VK_SAMPLE_COUNT_1_BIT, VK_IMAGE_ASPECT_COLOR_BIT);
            fineMask_ = makeImage(side, side, VK_FORMAT_R8_UINT, VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT,
                                  VK_SAMPLE_COUNT_1_BIT, VK_IMAGE_ASPECT_COLOR_BIT);
            fineSide_ = side;
        }
        uploadImage(fineTex_, data.data(), static_cast<size_t>(side + 2) * (side + 2) * 8);
        uploadImage(fineMask_, mask.data(), static_cast<size_t>(side) * side);
        fineCount_ = static_cast<uint32_t>(blocks.size() / 2);
        fineBlocks_.set(blocks.data(), blocks.size_bytes());
        fineOrigin_ = origin;
    }

    void setTreeSprites(std::span<const float> v) override {
        ensureFrame();
        if (treeAlloc_.valid()) { const Alloc a = treeAlloc_; later([this, a] { freeDevice(a); }); treeAlloc_ = {}; }
        treeCount_ = static_cast<uint32_t>(v.size() / 8);
        if (treeCount_ > 0) treeAlloc_ = uploadDevice(v.data(), v.size_bytes());
    }
    void setSpray(std::span<const float> v) override {
        sprayCount_ = static_cast<uint32_t>(v.size() / 8);
        spray_.set(v.data(), v.size_bytes());
    }
    void setSimWater(std::span<const float> v) override {
        dynCount_ = static_cast<uint32_t>(v.size() / 4);
        dyn_.set(v.data(), v.size_bytes());
    }

    void execute(const CommandList& list, FrameStats& stats) override {
        stats = {};
        water_.clear();
        ensureFrame();
        if (!acquire()) return;   // окно свёрнуто или цепочка кадров пересоздаётся
        Slot& s = slots_[slot_];
        const VkCommandBuffer c = s.cmd;
        Color clear{0, 0, 0, 1};
        for (const Command& cmd : list.commands) {
            if (auto* cl = std::get_if<CmdClear>(&cmd)) clear = cl->color;
            if (auto* cam = std::get_if<CmdCamera>(&cmd)) writeCamera(*cam, s);
        }
        for (Dyn* d : {&spray_, &dyn_, &fineBlocks_}) d->sync(*this, slot_);
        writeSets(s);

        // всё выгруженное (копии в видеопамять) - видно вершинам и шейдерам; первый охват - и прошлые отправки
        memoryBarrier(c, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT,
                      VK_PIPELINE_STAGE_2_VERTEX_INPUT_BIT | VK_PIPELINE_STAGE_2_VERTEX_SHADER_BIT | VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                      VK_ACCESS_2_VERTEX_ATTRIBUTE_READ_BIT | VK_ACCESS_2_INDEX_READ_BIT | VK_ACCESS_2_SHADER_READ_BIT);
        // цели кадра общие для кадров в полёте: ждём, пока прошлый кадр их дочитает
        imageBarrier(c, msaaColor_.img, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                     VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
                     VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_2_COLOR_ATTACHMENT_READ_BIT);
        imageBarrier(c, msaaDepth_.img, VK_IMAGE_ASPECT_DEPTH_BIT, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL,
                     VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT, VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT,
                     VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT,
                     VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_READ_BIT | VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT);
        imageBarrier(c, depthCopy_.img, VK_IMAGE_ASPECT_DEPTH_BIT, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL,
                     VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT, 0,
                     VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT,
                     VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT);

        // Проход 1 - непрозрачное. Глубина в конце сводится (образец 0, как blit GL_NEAREST) в копию для воды (Р51)
        VkRenderingAttachmentInfo col{VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO};
        col.imageView = msaaColor_.view;
        col.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        col.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        col.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        col.clearValue.color = {{clear.r, clear.g, clear.b, clear.a}};
        VkRenderingAttachmentInfo dep{VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO};
        dep.imageView = msaaDepth_.view;
        dep.imageLayout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
        dep.resolveMode = VK_RESOLVE_MODE_SAMPLE_ZERO_BIT;
        dep.resolveImageView = depthCopy_.view;
        dep.resolveImageLayout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
        dep.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        dep.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        dep.clearValue.depthStencil = {0.0f, 0};   // обратная глубина: «бесконечно далеко» - ноль
        beginRendering(c, col, dep);
        vkCmdBindDescriptorSets(c, VK_PIPELINE_BIND_POINT_GRAPHICS, layout_, 0, 1, &s.setOpaque, 0, nullptr);
        VkPipeline current = VK_NULL_HANDLE;
        for (const Command& cmd : list.commands) {
            if (auto* d = std::get_if<CmdDrawChunk>(&cmd)) {
                const Mesh& m = meshes_[d->mesh];
                if (!m.a.valid()) continue;
                if (m.water) water_.push_back(&cmd);
                stats.quads += m.quads;
                if (m.quads == m.water) continue;
                bindTerrain(c, current, pTerrain_);
                drawChunk(c, *d, m, 0, m.quads - m.water);
                ++stats.drawCalls;
            } else if (auto* t = std::get_if<CmdDrawTile>(&cmd)) {
                const Mesh& m = meshes_[t->mesh];
                if (!m.a.valid()) continue;
                if (m.water) water_.push_back(&cmd);
                bindTile(c, current, pTile_);
                drawTile(c, *t, m, 0, stats);
                ++stats.tiles;
            }
        }
        // Р65: спрайты дубов вдали - непрозрачные, до воды
        if (treeCount_ > 0) {
            vkCmdBindPipeline(c, VK_PIPELINE_BIND_POINT_GRAPHICS, pTree_);
            bindVertex(c, treeAlloc_);
            vkCmdDraw(c, 6, treeCount_, 0, 0);
            ++stats.drawCalls;
        }
        vkCmdEndRendering(c);

        // копия глубины - в текстуру; цвет и глубина MSAA - дальше рисует вода; окно - под сведение цвета
        imageBarrier(c, depthCopy_.img, VK_IMAGE_ASPECT_DEPTH_BIT, VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                     VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT,
                     VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT,
                     VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT, VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
        memoryBarrier(c, VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT,
                      VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT,
                      VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT,
                      VK_ACCESS_2_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT |
                          VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_READ_BIT | VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT);
        imageBarrier(c, swapImages_[image_], VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                     VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, 0,
                     VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);

        // Проход 2 - вода (Р34): смешение «поверх», глубину проверяет, но не пишет; в конце цвет сводится в окно
        col.loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
        col.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        col.resolveMode = VK_RESOLVE_MODE_AVERAGE_BIT;
        col.resolveImageView = swapViews_[image_];
        col.resolveImageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        dep.loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
        dep.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        dep.resolveMode = VK_RESOLVE_MODE_NONE;
        dep.resolveImageView = VK_NULL_HANDLE;
        beginRendering(c, col, dep);
        vkCmdBindDescriptorSets(c, VK_PIPELINE_BIND_POINT_GRAPHICS, layout_, 0, 1, &s.setWater, 0, nullptr);
        current = VK_NULL_HANDLE;
        for (const Command* cmd : water_) {
            if (auto* d = std::get_if<CmdDrawChunk>(cmd)) {
                const Mesh& m = meshes_[d->mesh];
                bindTerrain(c, current, pTerrainWater_);
                drawChunk(c, *d, m, m.quads - m.water, m.water);
                ++stats.drawCalls;
            } else if (auto* t = std::get_if<CmdDrawTile>(cmd)) {
                bindTile(c, current, pTileWater_);
                drawTile(c, *t, meshes_[t->mesh], 1, stats);
            }
        }
        // Р58: мелкая вода у камеры - столбики в тексель, строит вершинный шейдер
        if (fineCount_ > 0) {
            vkCmdBindPipeline(c, VK_PIPELINE_BIND_POINT_GRAPHICS, pFine_);
            const int32_t o[4] = {fineOrigin_.x, fineOrigin_.y, 0, 0};
            vkCmdPushConstants(c, layout_, PUSH_STAGES, 0, sizeof(o), o);
            vkCmdDraw(c, 256 * 30, fineCount_, 0, 0);
            ++stats.drawCalls;
        }
        // Р59: брызги и пыль - экземпляры-кубики
        if (sprayCount_ > 0) {
            vkCmdBindPipeline(c, VK_PIPELINE_BIND_POINT_GRAPHICS, pSpray_);
            const VkDeviceSize off = 0;
            vkCmdBindVertexBuffers(c, 0, 1, &spray_.per[slot_].buf, &off);
            vkCmdDraw(c, 18, sprayCount_, 0, 0);
            ++stats.drawCalls;
        }
        // Р45: вода автомата
        if (dynCount_ > 0) {
            vkCmdBindPipeline(c, VK_PIPELINE_BIND_POINT_GRAPHICS, pDyn_);
            const VkDeviceSize off = 0;
            vkCmdBindVertexBuffers(c, 0, 1, &dyn_.per[slot_].buf, &off);
            vkCmdDraw(c, dynCount_, 1, 0, 0);
            ++stats.drawCalls;
        }
        vkCmdEndRendering(c);
        imageBarrier(c, swapImages_[image_], VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
                     VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
                     VK_PIPELINE_STAGE_2_NONE, 0);
    }

    void present() override {
        ensureFrame();
        submit();
        if (acquired_) {
            VkPresentInfoKHR pi{VK_STRUCTURE_TYPE_PRESENT_INFO_KHR};
            pi.waitSemaphoreCount = 1;
            pi.pWaitSemaphores = &renderDone_[image_];
            pi.swapchainCount = 1;
            pi.pSwapchains = &swapchain_;
            pi.pImageIndices = &image_;
            const VkResult r = vkQueuePresentKHR(queue_, &pi);
            if (r == VK_ERROR_OUT_OF_DATE_KHR || r == VK_SUBOPTIMAL_KHR) swapDirty_ = true;
            else vkOk(r, "vkQueuePresentKHR");
        }
        acquired_ = waited_ = false;
        slot_ = (slot_ + 1) % FRAMES;
    }

    void resize(int, int) override { swapDirty_ = true; }

    // снимок: кадр из окна в буфер процессора; строки - снизу вверх, как у glReadPixels
    void readPixels(int w, int h, std::vector<uint8_t>& rgb) override {
        rgb.assign(static_cast<size_t>(w) * h * 3, 0);
        ensureFrame();
        if (!acquired_) return;
        const VkDeviceSize bytes = VkDeviceSize(extent_.width) * extent_.height * 4;
        if (readback_.cap < bytes) {
            if (readback_.buf) { vkDeviceWaitIdle(dev_); destroyHost(readback_); }
            readback_ = makeHost(bytes, VK_BUFFER_USAGE_TRANSFER_DST_BIT, true);
        }
        const VkCommandBuffer c = slots_[slot_].cmd;
        imageBarrier(c, swapImages_[image_], VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                     VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
                     VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_READ_BIT);
        VkBufferImageCopy r{};
        r.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
        r.imageExtent = {extent_.width, extent_.height, 1};
        vkCmdCopyImageToBuffer(c, swapImages_[image_], VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, readback_.buf, 1, &r);
        imageBarrier(c, swapImages_[image_], VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
                     VK_PIPELINE_STAGE_2_TRANSFER_BIT, 0, VK_PIPELINE_STAGE_2_NONE, 0);
        memoryBarrier(c, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_2_HOST_BIT, VK_ACCESS_2_HOST_READ_BIT);
        submit();
        ensureFrame();   // ждёт эту отправку и снова открывает команды кадра (present допишет остальное)
        VkMappedMemoryRange mr{VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE};
        mr.memory = readback_.mem;
        mr.size = VK_WHOLE_SIZE;
        vkInvalidateMappedMemoryRanges(dev_, 1, &mr);
        const int cw = std::min<int>(w, int(extent_.width)), ch = std::min<int>(h, int(extent_.height));
        for (int y = 0; y < ch; ++y) {
            const uint8_t* src = readback_.map + (size_t(y) * extent_.width) * 4;
            uint8_t* dst = &rgb[(size_t(h - 1 - y) * w) * 3];
            for (int x = 0; x < cw; ++x) {
                dst[x * 3 + 0] = src[x * 4 + (bgr_ ? 2 : 0)];
                dst[x * 3 + 1] = src[x * 4 + 1];
                dst[x * 3 + 2] = src[x * 4 + (bgr_ ? 0 : 2)];
            }
        }
    }

private:
    // water: у куска - сколько последних прямоугольников - вода; у плитки - 1, если в ней есть вода (Р34)
    struct Mesh { Alloc a; uint32_t quads = 0, water = 0; };
    struct Slot {
        VkCommandPool pool = VK_NULL_HANDLE;
        VkCommandBuffer cmd = VK_NULL_HANDLE;
        VkFence fence = VK_NULL_HANDLE;
        VkSemaphore acquired = VK_NULL_HANDLE;
        VkDescriptorSet setOpaque = VK_NULL_HANDLE, setWater = VK_NULL_HANDLE;
        HostBuf ubo;
        std::vector<HostBuf> stages;                 // общий буфер выгрузки этого кадра
        std::vector<std::function<void()>> deletes;  // освободить, когда кадр отработает
    };
    // данные, что меняются почти каждый кадр (брызги, вода автомата, список блоков мелкой воды): копия у процессора
    // и свой буфер у каждого кадра в полёте - кадр берёт последнюю версию
    struct Dyn {
        std::vector<uint8_t> cpu;
        uint64_t ver = 0;
        HostBuf per[FRAMES];
        uint64_t perVer[FRAMES] = {};
        void set(const void* p, size_t n) {
            cpu.resize(n);
            if (n) std::memcpy(cpu.data(), p, n);
            ++ver;
        }
        void sync(VKBackend& b, int slot) {
            if (perVer[slot] == ver) return;
            HostBuf& h = per[slot];
            if (h.cap < cpu.size()) {
                HostBuf old = h;
                b.later([&b, old]() mutable { b.destroyHost(old); });
                h = b.makeHost(std::max<VkDeviceSize>(cpu.size() + cpu.size() / 2, 64 << 10),
                               VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
            }
            if (!cpu.empty()) std::memcpy(h.map, cpu.data(), cpu.size());
            perVer[slot] = ver;
        }
    };
    struct Block { VkBuffer buf = VK_NULL_HANDLE; VkDeviceMemory mem = VK_NULL_HANDLE; VkDeviceSize size = 0; std::map<VkDeviceSize, VkDeviceSize> free; };

    // ---------- устройство ----------
    bool createDevice() {
        uint32_t n = 0;
        const char* const* sdlExt = SDL_Vulkan_GetInstanceExtensions(&n);
        if (!sdlExt) { journal::error("vulkan", "SDL: нет расширений Vulkan: {}", SDL_GetError()); return false; }
        std::vector<const char*> ext(sdlExt, sdlExt + n);
        const char* env = SDL_getenv("OTMEL_VK_VALIDATION");
        validation_ = env && env[0] == '1';
        std::vector<const char*> layers;
        if (validation_) { ext.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME); layers.push_back("VK_LAYER_KHRONOS_validation"); }
        VkApplicationInfo app{VK_STRUCTURE_TYPE_APPLICATION_INFO};
        app.pApplicationName = "Otmel";
        app.pEngineName = "Otmel";
        app.apiVersion = VK_API_VERSION_1_3;
        VkInstanceCreateInfo ici{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
        ici.pApplicationInfo = &app;
        ici.enabledExtensionCount = uint32_t(ext.size());
        ici.ppEnabledExtensionNames = ext.data();
        ici.enabledLayerCount = uint32_t(layers.size());
        ici.ppEnabledLayerNames = layers.data();
        if (!vkOk(vkCreateInstance(&ici, nullptr, &instance_), "vkCreateInstance")) return false;
        if (validation_) {
            auto create = reinterpret_cast<PFN_vkCreateDebugUtilsMessengerEXT>(vkGetInstanceProcAddr(instance_, "vkCreateDebugUtilsMessengerEXT"));
            VkDebugUtilsMessengerCreateInfoEXT mci{VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT};
            mci.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
            mci.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT |
                              VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
            mci.pfnUserCallback = onVkMessage;
            if (create) create(instance_, &mci, nullptr, &messenger_);
            journal::info("vulkan", "слой проверки включён");
        }
        if (!SDL_Vulkan_CreateSurface(window_, instance_, nullptr, &surface_)) {
            journal::error("vulkan", "поверхность окна не создана: {}", SDL_GetError());
            return false;
        }
        uint32_t count = 0;
        vkEnumeratePhysicalDevices(instance_, &count, nullptr);
        std::vector<VkPhysicalDevice> devs(count);
        vkEnumeratePhysicalDevices(instance_, &count, devs.data());
        int best = -1;
        for (VkPhysicalDevice d : devs) {
            VkPhysicalDeviceProperties p;
            vkGetPhysicalDeviceProperties(d, &p);
            if (p.apiVersion < VK_API_VERSION_1_3) continue;
            uint32_t qn = 0;
            vkGetPhysicalDeviceQueueFamilyProperties(d, &qn, nullptr);
            std::vector<VkQueueFamilyProperties> qs(qn);
            vkGetPhysicalDeviceQueueFamilyProperties(d, &qn, qs.data());
            for (uint32_t i = 0; i < qn; ++i) {
                VkBool32 present = VK_FALSE;
                vkGetPhysicalDeviceSurfaceSupportKHR(d, i, surface_, &present);
                if (!(qs[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) || !present) continue;
                const int score = p.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU ? 2 : 1;   // дискретная - лучше
                if (score > best) { best = score; phys_ = d; family_ = i; }
                break;
            }
        }
        if (!phys_) { journal::error("vulkan", "нет видеокарты с Vulkan 1.3 и выводом в окно"); return false; }
        VkPhysicalDeviceProperties props;
        vkGetPhysicalDeviceProperties(phys_, &props);
        vkGetPhysicalDeviceMemoryProperties(phys_, &memProps_);
        journal::info("vulkan", "Vulkan {}.{}.{}: {}", VK_API_VERSION_MAJOR(props.apiVersion), VK_API_VERSION_MINOR(props.apiVersion),
                      VK_API_VERSION_PATCH(props.apiVersion), props.deviceName);

        VkPhysicalDeviceVulkan13Features has13{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES};
        VkPhysicalDeviceFeatures2 has{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2};
        has.pNext = &has13;
        vkGetPhysicalDeviceFeatures2(phys_, &has);
        VkPhysicalDeviceVulkan13Features f13{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES};
        f13.dynamicRendering = VK_TRUE;
        f13.synchronization2 = VK_TRUE;
        f13.robustImageAccess = has13.robustImageAccess;
        f13.shaderDemoteToHelperInvocation = VK_TRUE;   // discard в шейдерах Vulkan 1.3 - «понижение до помощника»   // выборка за краем маски - ноль, как у OpenGL без текстуры
        VkPhysicalDeviceFeatures2 f2{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2};
        f2.pNext = &f13;
        const float prio = 1.0f;
        VkDeviceQueueCreateInfo qci{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
        qci.queueFamilyIndex = family_;
        qci.queueCount = 1;
        qci.pQueuePriorities = &prio;
        const char* dext[] = {VK_KHR_SWAPCHAIN_EXTENSION_NAME};
        VkDeviceCreateInfo dci{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
        dci.pNext = &f2;
        dci.queueCreateInfoCount = 1;
        dci.pQueueCreateInfos = &qci;
        dci.enabledExtensionCount = 1;
        dci.ppEnabledExtensionNames = dext;
        if (!vkOk(vkCreateDevice(phys_, &dci, nullptr, &dev_), "vkCreateDevice")) return false;
        vkGetDeviceQueue(dev_, family_, 0, &queue_);

        uint32_t fn = 0;
        vkGetPhysicalDeviceSurfaceFormatsKHR(phys_, surface_, &fn, nullptr);
        std::vector<VkSurfaceFormatKHR> fmts(fn);
        vkGetPhysicalDeviceSurfaceFormatsKHR(phys_, surface_, &fn, fmts.data());
        surfaceFormat_ = fmts[0];
        for (const VkSurfaceFormatKHR& f : fmts)   // как у OpenGL: 8 бит на канал без sRGB-перевода
            if (f.format == VK_FORMAT_B8G8R8A8_UNORM || f.format == VK_FORMAT_R8G8B8A8_UNORM) { surfaceFormat_ = f; break; }
        bgr_ = surfaceFormat_.format == VK_FORMAT_B8G8R8A8_UNORM;
        return true;
    }

    // ---------- цепочка кадров и цели ----------
    bool createSwapchain() {
        VkSurfaceCapabilitiesKHR caps;
        vkGetPhysicalDeviceSurfaceCapabilitiesKHR(phys_, surface_, &caps);
        VkExtent2D e = caps.currentExtent;
        if (e.width == UINT32_MAX) {
            int w = 1, h = 1;
            SDL_GetWindowSizeInPixels(window_, &w, &h);
            e.width = std::clamp(uint32_t(w), caps.minImageExtent.width, caps.maxImageExtent.width);
            e.height = std::clamp(uint32_t(h), caps.minImageExtent.height, caps.maxImageExtent.height);
        }
        if (e.width == 0 || e.height == 0) return false;   // свёрнуто
        uint32_t pn = 0;
        vkGetPhysicalDeviceSurfacePresentModesKHR(phys_, surface_, &pn, nullptr);
        std::vector<VkPresentModeKHR> modes(pn);
        vkGetPhysicalDeviceSurfacePresentModesKHR(phys_, surface_, &pn, modes.data());
        VkPresentModeKHR mode = VK_PRESENT_MODE_FIFO_KHR;   // ждать обновления экрана (как SwapInterval 1)
        if (!vsync_) {
            const auto has = [&](VkPresentModeKHR m) { return std::find(modes.begin(), modes.end(), m) != modes.end(); };
            mode = has(VK_PRESENT_MODE_IMMEDIATE_KHR) ? VK_PRESENT_MODE_IMMEDIATE_KHR : has(VK_PRESENT_MODE_MAILBOX_KHR) ? VK_PRESENT_MODE_MAILBOX_KHR : mode;
        }
        uint32_t images = caps.minImageCount + 1;
        if (caps.maxImageCount) images = std::min(images, caps.maxImageCount);
        VkSwapchainCreateInfoKHR sci{VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR};
        sci.surface = surface_;
        sci.minImageCount = images;
        sci.imageFormat = surfaceFormat_.format;
        sci.imageColorSpace = surfaceFormat_.colorSpace;
        sci.imageExtent = e;
        sci.imageArrayLayers = 1;
        sci.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
        sci.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
        sci.preTransform = caps.currentTransform;
        sci.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
        sci.presentMode = mode;
        sci.clipped = VK_TRUE;
        sci.oldSwapchain = swapchain_;
        VkSwapchainKHR sc = VK_NULL_HANDLE;
        if (!vkOk(vkCreateSwapchainKHR(dev_, &sci, nullptr, &sc), "vkCreateSwapchainKHR")) return false;
        destroySwapchainParts();
        if (swapchain_) vkDestroySwapchainKHR(dev_, swapchain_, nullptr);
        swapchain_ = sc;
        extent_ = e;
        uint32_t n = 0;
        vkGetSwapchainImagesKHR(dev_, swapchain_, &n, nullptr);
        swapImages_.resize(n);
        vkGetSwapchainImagesKHR(dev_, swapchain_, &n, swapImages_.data());
        for (VkImage img : swapImages_) {
            swapViews_.push_back(makeView(img, surfaceFormat_.format, VK_IMAGE_ASPECT_COLOR_BIT));
            VkSemaphoreCreateInfo s{VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO};
            VkSemaphore sem;
            vkCreateSemaphore(dev_, &s, nullptr, &sem);
            renderDone_.push_back(sem);
        }
        // свой буфер кадра: цвет и глубина MSAA ×4 (Р20), копия глубины без MSAA для воды (Р51)
        const int w = int(e.width), h = int(e.height);
        msaaColor_ = makeImage(w, h, surfaceFormat_.format, VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT, SAMPLES, VK_IMAGE_ASPECT_COLOR_BIT);
        msaaDepth_ = makeImage(w, h, DEPTH, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT, SAMPLES, VK_IMAGE_ASPECT_DEPTH_BIT);
        depthCopy_ = makeImage(w, h, DEPTH, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
                               VK_SAMPLE_COUNT_1_BIT, VK_IMAGE_ASPECT_DEPTH_BIT);
        swapDirty_ = false;
        journal::info("vulkan", "цепочка кадров {}×{}, {} изображений, {}", w, h, n, vsync_ ? "с ожиданием экрана" : "без ожидания экрана");
        return true;
    }
    void destroySwapchainParts() {
        for (VkImageView v : swapViews_) vkDestroyImageView(dev_, v, nullptr);
        for (VkSemaphore s : renderDone_) vkDestroySemaphore(dev_, s, nullptr);
        swapViews_.clear();
        renderDone_.clear();
        swapImages_.clear();
        destroyImage(msaaColor_);
        destroyImage(msaaDepth_);
        destroyImage(depthCopy_);
    }
    bool acquire() {
        for (int attempt = 0; attempt < 2; ++attempt) {
            if (swapDirty_) {
                vkDeviceWaitIdle(dev_);
                if (!createSwapchain()) return false;
            }
            const VkResult r = vkAcquireNextImageKHR(dev_, swapchain_, UINT64_MAX, slots_[slot_].acquired, VK_NULL_HANDLE, &image_);
            if (r == VK_SUCCESS || r == VK_SUBOPTIMAL_KHR) { acquired_ = true; return true; }
            if (r == VK_ERROR_OUT_OF_DATE_KHR) { swapDirty_ = true; continue; }
            vkOk(r, "vkAcquireNextImageKHR");
            return false;
        }
        return false;
    }

    // ---------- кадр ----------
    // открыть команды кадра: дождаться, пока видеокарта отработает прошлый кадр этого места, освободить отложенное
    void ensureFrame() {
        if (recording_) return;
        Slot& s = slots_[slot_];
        vkWaitForFences(dev_, 1, &s.fence, VK_TRUE, UINT64_MAX);
        vkResetFences(dev_, 1, &s.fence);
        for (auto& f : s.deletes) f();
        s.deletes.clear();
        while (s.stages.size() > 4) { destroyHost(s.stages.back()); s.stages.pop_back(); }   // после всплеска выгрузки
        for (HostBuf& h : s.stages) h.used = 0;
        vkResetCommandPool(dev_, s.pool, 0);
        VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
        bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
        vkBeginCommandBuffer(s.cmd, &bi);
        recording_ = true;
    }
    void submit() {
        Slot& s = slots_[slot_];
        vkEndCommandBuffer(s.cmd);
        VkSemaphoreSubmitInfo wait{VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO};
        wait.semaphore = s.acquired;
        wait.stageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_2_TRANSFER_BIT;
        VkSemaphoreSubmitInfo signal{VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO};
        signal.semaphore = acquired_ ? renderDone_[image_] : VK_NULL_HANDLE;
        signal.stageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
        VkCommandBufferSubmitInfo cb{VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO};
        cb.commandBuffer = s.cmd;
        VkSubmitInfo2 si{VK_STRUCTURE_TYPE_SUBMIT_INFO_2};
        const bool first = acquired_ && !waited_;   // изображение окна ждём и отмечаем готовым один раз за кадр
        si.waitSemaphoreInfoCount = first ? 1 : 0;
        si.pWaitSemaphoreInfos = &wait;
        si.signalSemaphoreInfoCount = first ? 1 : 0;
        si.pSignalSemaphoreInfos = &signal;
        si.commandBufferInfoCount = 1;
        si.pCommandBufferInfos = &cb;
        vkOk(vkQueueSubmit2(queue_, 1, &si, s.fence), "vkQueueSubmit2");
        if (first) waited_ = true;
        recording_ = false;
    }
    void later(std::function<void()> f) { ensureFrame(); slots_[slot_].deletes.push_back(std::move(f)); }

    void writeCamera(const CmdCamera& cam, Slot& s) {
        FrameUbo u{};
        const float* m = &cam.viewProj[0][0];
        for (int i = 0; i < 16; ++i) u.viewProj[i] = m[i];
        u.sunDir[0] = cam.sunDir.x; u.sunDir[1] = cam.sunDir.y; u.sunDir[2] = cam.sunDir.z;
        u.eye[0] = cam.eye.x; u.eye[1] = cam.eye.y; u.eye[2] = cam.eye.z;
        for (int i = 0; i < 4; ++i) u.voxelZone[i] = cam.voxelZone[i];
        u.fade[0] = cam.fadeStart; u.fade[1] = cam.fadeEnd;
        u.fade[2] = cam.hazeDistance; u.fade[3] = cam.hazeMax;
        u.sky[0] = cam.sky.r; u.sky[1] = cam.sky.g; u.sky[2] = cam.sky.b; u.sky[3] = cam.sky.a;
        for (int i = 0; i < 4; ++i) u.flow[i] = cam.flowArea[i];
        u.time[0] = cam.time;
        for (int i = 0; i < 4; ++i) u.simArea[i] = cam.simArea[i];
        const glm::mat4 inv = glm::inverse(cam.viewProj);
        const float* mi = &inv[0][0];
        for (int i = 0; i < 16; ++i) u.invViewProj[i] = mi[i];
        for (int i = 0; i < 3; ++i) { u.originV[i] = cam.originV[i]; u.originOld[i] = float(cam.originV[i]) / 16.0f; }
        std::memcpy(s.ubo.map, &u, sizeof(u));
    }

    // наборы привязок кадра: в непрозрачном проходе на месте глубины сцены - заглушка (копия ещё пишется)
    void writeSets(Slot& s) {
        VkDescriptorBufferInfo ubo{s.ubo.buf, 0, sizeof(FrameUbo)};
        VkDescriptorBufferInfo ssbo{fineBlocks_.per[slot_].buf, 0, VK_WHOLE_SIZE};
        VkWriteDescriptorSet w[14];
        VkDescriptorImageInfo img[2][5];
        for (int k = 0; k < 2; ++k) {
            const VkDescriptorSet set = k ? s.setWater : s.setOpaque;
            const VkImageLayout ro = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
            img[k][0] = {linear_, flowTex_.view, ro};
            img[k][1] = {nearest_, k ? depthCopy_.view : dummyFloat_.view, ro};
            img[k][2] = {nearest_, fineSide_ ? fineTex_.view : dummyFloat_.view, ro};
            img[k][3] = {nearest_, fineSide_ ? fineMask_.view : dummyUint_.view, ro};
            img[k][4] = {nearest_, maskTex_.view, ro};
            const uint32_t imgBinding[5] = {1, 2, 4, 5, 6};
            VkWriteDescriptorSet* o = &w[k * 7];
            for (int i = 0; i < 7; ++i) o[i] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
            o[0].dstSet = set; o[0].dstBinding = 0; o[0].descriptorCount = 1;
            o[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER; o[0].pBufferInfo = &ubo;
            o[1].dstSet = set; o[1].dstBinding = 3; o[1].descriptorCount = 1;
            o[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER; o[1].pBufferInfo = &ssbo;
            for (int i = 0; i < 5; ++i) {
                o[2 + i].dstSet = set; o[2 + i].dstBinding = imgBinding[i]; o[2 + i].descriptorCount = 1;
                o[2 + i].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER; o[2 + i].pImageInfo = &img[k][i];
            }
        }
        vkUpdateDescriptorSets(dev_, 14, w, 0, nullptr);
    }

    void beginRendering(VkCommandBuffer c, const VkRenderingAttachmentInfo& col, const VkRenderingAttachmentInfo& dep) {
        VkRenderingInfo ri{VK_STRUCTURE_TYPE_RENDERING_INFO};
        ri.renderArea = {{0, 0}, extent_};
        ri.layerCount = 1;
        ri.colorAttachmentCount = 1;
        ri.pColorAttachments = &col;
        ri.pDepthAttachment = &dep;
        vkCmdBeginRendering(c, &ri);
        // ось Y - вверх, как у OpenGL: окно перевёрнуто отрицательной высотой (обход граней тот же - против часовой)
        const VkViewport vp{0.0f, float(extent_.height), float(extent_.width), -float(extent_.height), 0.0f, 1.0f};
        const VkRect2D sc{{0, 0}, extent_};
        vkCmdSetViewport(c, 0, 1, &vp);
        vkCmdSetScissor(c, 0, 1, &sc);
    }
    void bindVertex(VkCommandBuffer c, const Alloc& a) { vkCmdBindVertexBuffers(c, 0, 1, &blocks_[a.block].buf, &a.offset); }
    void bindTerrain(VkCommandBuffer c, VkPipeline& current, VkPipeline p) {
        if (current == p) return;
        vkCmdBindPipeline(c, VK_PIPELINE_BIND_POINT_GRAPHICS, p);
        vkCmdBindIndexBuffer(c, blocks_[quadIdx_.block].buf, quadIdx_.offset, VK_INDEX_TYPE_UINT32);
        current = p;
    }
    void bindTile(VkCommandBuffer c, VkPipeline& current, VkPipeline p) {
        if (current == p) return;
        vkCmdBindPipeline(c, VK_PIPELINE_BIND_POINT_GRAPHICS, p);
        vkCmdBindIndexBuffer(c, blocks_[tileIdx_.block].buf, tileIdx_.offset, VK_INDEX_TYPE_UINT16);
        current = p;
    }
    void drawChunk(VkCommandBuffer c, const CmdDrawChunk& d, const Mesh& m, uint32_t firstQuad, uint32_t quads) {
        const PushChunk pc{{d.originVoxels.x, d.originVoxels.y, d.originVoxels.z, 0},
                           {d.fineOff.x, d.fineOff.y, d.fineOff.z, 0},
                           {d.fine ? 1 : 0, d.fineScale, 0, 0}};
        vkCmdPushConstants(c, layout_, PUSH_STAGES, 0, sizeof(pc), &pc);
        bindVertex(c, m.a);
        vkCmdDrawIndexed(c, quads * 6, 1, firstQuad * 6, 0, 0);
    }
    void drawTile(VkCommandBuffer c, const CmdDrawTile& t, const Mesh& m, int water, FrameStats& stats) {
        const PushTile pc{{t.originBlocks.x, t.originBlocks.y, t.spacing, t.level}, {t.morphStart, t.morphEnd, 0, 0}, {debug_, water, 0, 0}};
        vkCmdPushConstants(c, layout_, PUSH_STAGES, 0, sizeof(pc), &pc);
        bindVertex(c, m.a);
        if (t.quadrants == 0xF) {
            vkCmdDrawIndexed(c, 4 * QUAD_INDICES, 1, 0, 0, 0);
            ++stats.drawCalls;
            stats.tileTriangles += 4 * QUAD_INDICES / 3;
        } else {
            for (int q = 0; q < 4; ++q)
                if (t.quadrants >> q & 1) {
                    vkCmdDrawIndexed(c, QUAD_INDICES, 1, q * QUAD_INDICES, 0, 0);
                    ++stats.drawCalls;
                    stats.tileTriangles += QUAD_INDICES / 3;
                }
        }
    }

    static void imageBarrier(VkCommandBuffer c, VkImage img, VkImageAspectFlags aspect, VkImageLayout from, VkImageLayout to,
                             VkPipelineStageFlags2 srcStage, VkAccessFlags2 srcAccess, VkPipelineStageFlags2 dstStage, VkAccessFlags2 dstAccess) {
        VkImageMemoryBarrier2 b{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2};
        b.srcStageMask = srcStage;
        b.srcAccessMask = srcAccess;
        b.dstStageMask = dstStage;
        b.dstAccessMask = dstAccess;
        b.oldLayout = from;
        b.newLayout = to;
        b.srcQueueFamilyIndex = b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        b.image = img;
        b.subresourceRange = {aspect, 0, 1, 0, 1};
        VkDependencyInfo di{VK_STRUCTURE_TYPE_DEPENDENCY_INFO};
        di.imageMemoryBarrierCount = 1;
        di.pImageMemoryBarriers = &b;
        vkCmdPipelineBarrier2(c, &di);
    }
    static void memoryBarrier(VkCommandBuffer c, VkPipelineStageFlags2 srcStage, VkAccessFlags2 srcAccess,
                              VkPipelineStageFlags2 dstStage, VkAccessFlags2 dstAccess) {
        VkMemoryBarrier2 b{VK_STRUCTURE_TYPE_MEMORY_BARRIER_2};
        b.srcStageMask = srcStage;
        b.srcAccessMask = srcAccess;
        b.dstStageMask = dstStage;
        b.dstAccessMask = dstAccess;
        VkDependencyInfo di{VK_STRUCTURE_TYPE_DEPENDENCY_INFO};
        di.memoryBarrierCount = 1;
        di.pMemoryBarriers = &b;
        vkCmdPipelineBarrier2(c, &di);
    }

    // ---------- память ----------
    uint32_t memoryType(uint32_t bits, VkMemoryPropertyFlags want, VkMemoryPropertyFlags fallback = 0) const {
        for (VkMemoryPropertyFlags w : {want, fallback}) {
            if (!w) continue;
            for (uint32_t i = 0; i < memProps_.memoryTypeCount; ++i)
                if ((bits >> i & 1) && (memProps_.memoryTypes[i].propertyFlags & w) == w) return i;
        }
        journal::error("vulkan", "нет памяти нужного вида");
        return 0;
    }
    HostBuf makeHost(VkDeviceSize size, VkBufferUsageFlags usage, bool cached = false) {
        HostBuf h;
        VkBufferCreateInfo bci{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
        bci.size = size;
        bci.usage = usage;
        vkCreateBuffer(dev_, &bci, nullptr, &h.buf);
        VkMemoryRequirements req;
        vkGetBufferMemoryRequirements(dev_, h.buf, &req);
        VkMemoryAllocateInfo mai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
        mai.allocationSize = req.size;
        const VkMemoryPropertyFlags base = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
        mai.memoryTypeIndex = cached ? memoryType(req.memoryTypeBits, base | VK_MEMORY_PROPERTY_HOST_CACHED_BIT, base)
                                     : memoryType(req.memoryTypeBits, base);
        if (!vkOk(vkAllocateMemory(dev_, &mai, nullptr, &h.mem), "vkAllocateMemory (буфер процессора)")) return h;
        vkBindBufferMemory(dev_, h.buf, h.mem, 0);
        vkMapMemory(dev_, h.mem, 0, VK_WHOLE_SIZE, 0, reinterpret_cast<void**>(&h.map));
        h.cap = size;
        return h;
    }
    void destroyHost(HostBuf& h) {
        if (h.buf) vkDestroyBuffer(dev_, h.buf, nullptr);
        if (h.mem) vkFreeMemory(dev_, h.mem, nullptr);
        h = {};
    }
    // данные - в общий буфер выгрузки кадра (растёт кусками по 16 МБ)
    std::pair<VkBuffer, VkDeviceSize> stage(const void* data, VkDeviceSize size) {
        Slot& s = slots_[slot_];
        for (HostBuf& h : s.stages) {
            const VkDeviceSize off = (h.used + 15) & ~VkDeviceSize(15);
            if (off + size > h.cap) continue;
            std::memcpy(h.map + off, data, size);
            h.used = off + size;
            return {h.buf, off};
        }
        HostBuf h = makeHost(std::max(STAGE, size), VK_BUFFER_USAGE_TRANSFER_SRC_BIT);
        std::memcpy(h.map, data, size);
        h.used = size;
        s.stages.push_back(h);
        return {h.buf, 0};
    }
    // отрезок блока видеопамяти: первый подходящий свободный, иначе новый блок
    Alloc allocDevice(VkDeviceSize size) {
        size = (size + 255) & ~VkDeviceSize(255);
        for (uint32_t b = 0; b < blocks_.size(); ++b) {
            auto& f = blocks_[b].free;
            for (auto it = f.begin(); it != f.end(); ++it) {
                if (it->second < size) continue;
                const VkDeviceSize off = it->first, rest = it->second - size;
                f.erase(it);
                if (rest) f.emplace(off + size, rest);
                return {b, off, size};
            }
        }
        Block blk;
        blk.size = std::max(BLOCK, size);
        VkBufferCreateInfo bci{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
        bci.size = blk.size;
        bci.usage = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
        vkCreateBuffer(dev_, &bci, nullptr, &blk.buf);
        VkMemoryRequirements req;
        vkGetBufferMemoryRequirements(dev_, blk.buf, &req);
        VkMemoryAllocateInfo mai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
        mai.allocationSize = req.size;
        mai.memoryTypeIndex = memoryType(req.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        if (!vkOk(vkAllocateMemory(dev_, &mai, nullptr, &blk.mem), "vkAllocateMemory (блок сеток)")) return {};
        vkBindBufferMemory(dev_, blk.buf, blk.mem, 0);
        if (blk.size > size) blk.free.emplace(size, blk.size - size);
        blocks_.push_back(std::move(blk));
        journal::info("vulkan", "блок видеопамяти под сетки №{}: {} МБ", blocks_.size(), blocks_.back().size >> 20);
        return {uint32_t(blocks_.size() - 1), 0, size};
    }
    void freeDevice(const Alloc& a) {
        auto& f = blocks_[a.block].free;
        auto it = f.emplace(a.offset, a.size).first;
        if (auto nx = std::next(it); nx != f.end() && it->first + it->second == nx->first) { it->second += nx->second; f.erase(nx); }
        if (it != f.begin()) {
            auto pv = std::prev(it);
            if (pv->first + pv->second == it->first) { pv->second += it->second; f.erase(it); }
        }
    }
    Alloc uploadDevice(const void* data, VkDeviceSize size) {
        ensureFrame();
        const Alloc a = allocDevice(size);
        if (!a.valid()) return a;
        const auto [buf, off] = stage(data, size);
        const VkBufferCopy r{off, a.offset, size};
        vkCmdCopyBuffer(slots_[slot_].cmd, buf, blocks_[a.block].buf, 1, &r);
        return a;
    }
    MeshId store(std::span<const uint32_t> v, uint32_t quads, uint32_t water) {
        Mesh m;
        m.a = uploadDevice(v.data(), v.size_bytes());
        if (!m.a.valid()) return NO_MESH;
        m.quads = quads;
        m.water = water;
        if (!free_.empty()) {
            const MeshId id = free_.back();
            free_.pop_back();
            meshes_[id] = m;
            return id;
        }
        meshes_.push_back(m);
        return static_cast<MeshId>(meshes_.size() - 1);
    }

    VkImageView makeView(VkImage img, VkFormat f, VkImageAspectFlags aspect) {
        VkImageViewCreateInfo vci{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
        vci.image = img;
        vci.viewType = VK_IMAGE_VIEW_TYPE_2D;
        vci.format = f;
        vci.subresourceRange = {aspect, 0, 1, 0, 1};
        VkImageView v = VK_NULL_HANDLE;
        vkCreateImageView(dev_, &vci, nullptr, &v);
        return v;
    }
    Image makeImage(int w, int h, VkFormat f, VkImageUsageFlags usage, VkSampleCountFlagBits samples, VkImageAspectFlags aspect) {
        Image im;
        im.w = w;
        im.h = h;
        VkImageCreateInfo ci{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
        ci.imageType = VK_IMAGE_TYPE_2D;
        ci.format = f;
        ci.extent = {uint32_t(w), uint32_t(h), 1};
        ci.mipLevels = 1;
        ci.arrayLayers = 1;
        ci.samples = samples;
        ci.tiling = VK_IMAGE_TILING_OPTIMAL;
        ci.usage = usage;
        ci.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        if (!vkOk(vkCreateImage(dev_, &ci, nullptr, &im.img), "vkCreateImage")) return im;
        VkMemoryRequirements req;
        vkGetImageMemoryRequirements(dev_, im.img, &req);
        VkMemoryAllocateInfo mai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
        mai.allocationSize = req.size;
        mai.memoryTypeIndex = memoryType(req.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        vkOk(vkAllocateMemory(dev_, &mai, nullptr, &im.mem), "vkAllocateMemory (изображение)");
        vkBindImageMemory(dev_, im.img, im.mem, 0);
        im.view = makeView(im.img, f, aspect);
        return im;
    }
    void destroyImage(Image& im) {
        if (im.view) vkDestroyImageView(dev_, im.view, nullptr);
        if (im.img) vkDestroyImage(dev_, im.img, nullptr);
        if (im.mem) vkFreeMemory(dev_, im.mem, nullptr);
        im = {};
    }
    void drop(Image& im) {   // удалить, когда кадры, что могли его читать, отработают
        if (!im.img) return;
        Image old = im;
        later([this, old]() mutable { destroyImage(old); });
        im = {};
    }
    // изображение целиком - из общего буфера выгрузки; прошлые чтения дожидаются (первый охват - и прошлые кадры)
    void uploadImage(Image& im, const void* data, size_t bytes) {
        ensureFrame();
        const auto [buf, off] = stage(data, bytes);
        const VkCommandBuffer c = slots_[slot_].cmd;
        imageBarrier(c, im.img, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                     VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT, 0, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);
        VkBufferImageCopy r{};
        r.bufferOffset = off;
        r.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
        r.imageExtent = {uint32_t(im.w), uint32_t(im.h), 1};
        vkCmdCopyBufferToImage(c, buf, im.img, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &r);
        imageBarrier(c, im.img, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                     VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT,
                     VK_PIPELINE_STAGE_2_VERTEX_SHADER_BIT | VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT, VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
    }

    // ---------- конвейеры ----------
    bool createLayouts() {
        const VkShaderStageFlags st = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
        // 0 кадр, 1 течения, 2 глубина сцены, 3 блоки мелкой воды, 4 мелкая вода, 5 её маска, 6 маска вокселей
        VkDescriptorSetLayoutBinding b[7] = {
            {0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, st, nullptr},
            {1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, st, nullptr},
            {2, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, st, nullptr},
            {3, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, st, nullptr},
            {4, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, st, nullptr},
            {5, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, st, nullptr},
            {6, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, st, nullptr},
        };
        VkDescriptorSetLayoutCreateInfo lci{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
        lci.bindingCount = 7;
        lci.pBindings = b;
        if (!vkOk(vkCreateDescriptorSetLayout(dev_, &lci, nullptr, &setLayout_), "vkCreateDescriptorSetLayout")) return false;
        const VkPushConstantRange pr{PUSH_STAGES, 0, PUSH_BYTES};
        VkPipelineLayoutCreateInfo plci{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
        plci.setLayoutCount = 1;
        plci.pSetLayouts = &setLayout_;
        plci.pushConstantRangeCount = 1;
        plci.pPushConstantRanges = &pr;
        if (!vkOk(vkCreatePipelineLayout(dev_, &plci, nullptr, &layout_), "vkCreatePipelineLayout")) return false;
        const VkDescriptorPoolSize sizes[3] = {{VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, FRAMES * 2},
                                               {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, FRAMES * 2 * 5},
                                               {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, FRAMES * 2}};
        VkDescriptorPoolCreateInfo dpci{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
        dpci.maxSets = FRAMES * 2;
        dpci.poolSizeCount = 3;
        dpci.pPoolSizes = sizes;
        return vkOk(vkCreateDescriptorPool(dev_, &dpci, nullptr, &descPool_), "vkCreateDescriptorPool");
    }
    VkShaderModule loadModule(const std::string& path) {
        std::ifstream f(path, std::ios::binary);
        std::vector<char> code((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
        if (code.empty() || code.size() % 4) { journal::error("vulkan", "нет шейдера {}", path); return VK_NULL_HANDLE; }
        VkShaderModuleCreateInfo ci{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
        ci.codeSize = code.size();
        ci.pCode = reinterpret_cast<const uint32_t*>(code.data());
        VkShaderModule m = VK_NULL_HANDLE;
        vkOk(vkCreateShaderModule(dev_, &ci, nullptr, &m), path.c_str());
        return m;
    }
    enum Input { NONE, CHUNK, TILE_IN, INSTANCED, DYN };
    VkPipeline makePipeline(const std::string& dir, const char* vs, const char* fs, Input input, bool blend, bool depthWrite, bool cull) {
        const VkShaderModule v = loadModule(dir + "/" + vs + ".vert.vk.spv");
        const VkShaderModule f = loadModule(dir + "/" + fs + ".frag.vk.spv");
        if (!v || !f) return VK_NULL_HANDLE;
        VkPipelineShaderStageCreateInfo stages[2] = {{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO}, {VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO}};
        stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
        stages[0].module = v;
        stages[0].pName = "main";
        stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
        stages[1].module = f;
        stages[1].pName = "main";
        VkVertexInputBindingDescription bind{0, 0, VK_VERTEX_INPUT_RATE_VERTEX};
        VkVertexInputAttributeDescription attr[2] = {};
        uint32_t attrs = 0;
        switch (input) {
            case NONE: break;
            case CHUNK: bind.stride = 4; attr[0] = {0, 0, VK_FORMAT_R32_UINT, 0}; attrs = 1; break;   // вершина куска - одно целое
            case TILE_IN:   // узел плитки - пять целых: четыре - рельеф, пятое - вода (Р34)
                bind.stride = 20; attr[0] = {0, 0, VK_FORMAT_R32G32B32A32_UINT, 0}; attr[1] = {1, 0, VK_FORMAT_R32_UINT, 16}; attrs = 2; break;
            case INSTANCED:   // брызги и спрайты: по экземпляру два vec4
                bind.stride = 32; bind.inputRate = VK_VERTEX_INPUT_RATE_INSTANCE;
                attr[0] = {0, 0, VK_FORMAT_R32G32B32A32_SFLOAT, 0}; attr[1] = {1, 0, VK_FORMAT_R32G32B32A32_SFLOAT, 16}; attrs = 2; break;
            case DYN: bind.stride = 16; attr[0] = {0, 0, VK_FORMAT_R32G32B32A32_SFLOAT, 0}; attrs = 1; break;
        }
        VkPipelineVertexInputStateCreateInfo vi{VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
        vi.vertexBindingDescriptionCount = input == NONE ? 0 : 1;
        vi.pVertexBindingDescriptions = &bind;
        vi.vertexAttributeDescriptionCount = attrs;
        vi.pVertexAttributeDescriptions = attr;
        VkPipelineInputAssemblyStateCreateInfo ia{VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
        ia.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
        VkPipelineViewportStateCreateInfo vp{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
        vp.viewportCount = 1;
        vp.scissorCount = 1;
        VkPipelineRasterizationStateCreateInfo rs{VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
        rs.polygonMode = VK_POLYGON_MODE_FILL;
        rs.cullMode = cull ? VK_CULL_MODE_BACK_BIT : VK_CULL_MODE_NONE;   // задние грани не рисуем
        rs.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
        rs.lineWidth = 1.0f;
        VkPipelineMultisampleStateCreateInfo ms{VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};
        ms.rasterizationSamples = SAMPLES;
        VkPipelineDepthStencilStateCreateInfo ds{VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO};
        ds.depthTestEnable = VK_TRUE;
        ds.depthWriteEnable = depthWrite ? VK_TRUE : VK_FALSE;
        ds.depthCompareOp = VK_COMPARE_OP_GREATER;   // обратная глубина: ближе - больше
        VkPipelineColorBlendAttachmentState ba{};
        ba.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
        if (blend) {   // a·вода + (1 - a)·что за ней
            ba.blendEnable = VK_TRUE;
            ba.srcColorBlendFactor = ba.srcAlphaBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
            ba.dstColorBlendFactor = ba.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
            ba.colorBlendOp = ba.alphaBlendOp = VK_BLEND_OP_ADD;
        }
        VkPipelineColorBlendStateCreateInfo cb{VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
        cb.attachmentCount = 1;
        cb.pAttachments = &ba;
        const VkDynamicState dyn[2] = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
        VkPipelineDynamicStateCreateInfo dy{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
        dy.dynamicStateCount = 2;
        dy.pDynamicStates = dyn;
        VkPipelineRenderingCreateInfo rci{VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO};
        rci.colorAttachmentCount = 1;
        rci.pColorAttachmentFormats = &surfaceFormat_.format;
        rci.depthAttachmentFormat = DEPTH;
        VkGraphicsPipelineCreateInfo pci{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
        pci.pNext = &rci;
        pci.stageCount = 2;
        pci.pStages = stages;
        pci.pVertexInputState = &vi;
        pci.pInputAssemblyState = &ia;
        pci.pViewportState = &vp;
        pci.pRasterizationState = &rs;
        pci.pMultisampleState = &ms;
        pci.pDepthStencilState = &ds;
        pci.pColorBlendState = &cb;
        pci.pDynamicState = &dy;
        pci.layout = layout_;
        VkPipeline p = VK_NULL_HANDLE;
        vkOk(vkCreateGraphicsPipelines(dev_, VK_NULL_HANDLE, 1, &pci, nullptr, &p), vs);
        vkDestroyShaderModule(dev_, v, nullptr);
        vkDestroyShaderModule(dev_, f, nullptr);
        return p;
    }
    bool createPipelines(const std::string& dir) {
        pTerrain_ = makePipeline(dir, "terrain", "terrain", CHUNK, false, true, true);
        pTerrainWater_ = makePipeline(dir, "terrain", "terrain", CHUNK, true, false, true);
        pTile_ = makePipeline(dir, "tile", "tile", TILE_IN, false, true, true);
        pTileWater_ = makePipeline(dir, "tile", "tile", TILE_IN, true, false, true);
        pTree_ = makePipeline(dir, "treesprite", "treesprite", INSTANCED, false, true, false);   // Р65: вырез по рисунку
        pFine_ = makePipeline(dir, "finewater", "terrain", NONE, true, false, false);            // Р58
        pSpray_ = makePipeline(dir, "spray", "terrain", INSTANCED, true, false, false);          // Р59
        pDyn_ = makePipeline(dir, "dynwater", "terrain", DYN, true, false, false);               // Р45
        return pTerrain_ && pTerrainWater_ && pTile_ && pTileWater_ && pTree_ && pFine_ && pSpray_ && pDyn_;
    }

    void shutdown() {
        if (!dev_) {
            if (surface_) vkDestroySurfaceKHR(instance_, surface_, nullptr);
            if (instance_) vkDestroyInstance(instance_, nullptr);
            return;
        }
        vkDeviceWaitIdle(dev_);
        for (Slot& s : slots_) {
            for (auto& f : s.deletes) f();
            for (HostBuf& h : s.stages) destroyHost(h);
            destroyHost(s.ubo);
            if (s.fence) vkDestroyFence(dev_, s.fence, nullptr);
            if (s.acquired) vkDestroySemaphore(dev_, s.acquired, nullptr);
            if (s.pool) vkDestroyCommandPool(dev_, s.pool, nullptr);
        }
        for (Dyn* d : {&spray_, &dyn_, &fineBlocks_})
            for (HostBuf& b : d->per) destroyHost(b);
        destroyHost(readback_);
        for (Block& b : blocks_) { vkDestroyBuffer(dev_, b.buf, nullptr); vkFreeMemory(dev_, b.mem, nullptr); }
        for (Image* im : {&maskTex_, &flowTex_, &fineTex_, &fineMask_, &dummyFloat_, &dummyUint_}) destroyImage(*im);
        destroySwapchainParts();
        if (swapchain_) vkDestroySwapchainKHR(dev_, swapchain_, nullptr);
        for (VkPipeline p : {pTerrain_, pTerrainWater_, pTile_, pTileWater_, pTree_, pFine_, pSpray_, pDyn_})
            if (p) vkDestroyPipeline(dev_, p, nullptr);
        if (layout_) vkDestroyPipelineLayout(dev_, layout_, nullptr);
        if (descPool_) vkDestroyDescriptorPool(dev_, descPool_, nullptr);
        if (setLayout_) vkDestroyDescriptorSetLayout(dev_, setLayout_, nullptr);
        if (linear_) vkDestroySampler(dev_, linear_, nullptr);
        if (nearest_) vkDestroySampler(dev_, nearest_, nullptr);
        vkDestroyDevice(dev_, nullptr);
        vkDestroySurfaceKHR(instance_, surface_, nullptr);
        if (messenger_) {
            auto destroy = reinterpret_cast<PFN_vkDestroyDebugUtilsMessengerEXT>(vkGetInstanceProcAddr(instance_, "vkDestroyDebugUtilsMessengerEXT"));
            if (destroy) destroy(instance_, messenger_, nullptr);
        }
        vkDestroyInstance(instance_, nullptr);
    }

    SDL_Window* window_;
    bool validation_ = false;
    VkInstance instance_ = VK_NULL_HANDLE;
    VkDebugUtilsMessengerEXT messenger_ = VK_NULL_HANDLE;
    VkSurfaceKHR surface_ = VK_NULL_HANDLE;
    VkPhysicalDevice phys_ = VK_NULL_HANDLE;
    VkPhysicalDeviceMemoryProperties memProps_{};
    VkDevice dev_ = VK_NULL_HANDLE;
    VkQueue queue_ = VK_NULL_HANDLE;
    uint32_t family_ = 0;
    VkSurfaceFormatKHR surfaceFormat_{};
    bool bgr_ = true;
    VkSwapchainKHR swapchain_ = VK_NULL_HANDLE;
    VkExtent2D extent_{1, 1};
    std::vector<VkImage> swapImages_;
    std::vector<VkImageView> swapViews_;
    std::vector<VkSemaphore> renderDone_;   // по изображению окна: показ ждёт отрисовку
    bool vsync_ = true, swapDirty_ = false;
    Image msaaColor_, msaaDepth_, depthCopy_;
    Image maskTex_, flowTex_, fineTex_, fineMask_, dummyFloat_, dummyUint_;
    int maskSide_ = 0, flowSide_ = 0, fineSide_ = 0;
    VkSampler linear_ = VK_NULL_HANDLE, nearest_ = VK_NULL_HANDLE;
    VkDescriptorSetLayout setLayout_ = VK_NULL_HANDLE;
    VkPipelineLayout layout_ = VK_NULL_HANDLE;
    VkDescriptorPool descPool_ = VK_NULL_HANDLE;
    VkPipeline pTerrain_ = VK_NULL_HANDLE, pTerrainWater_ = VK_NULL_HANDLE, pTile_ = VK_NULL_HANDLE, pTileWater_ = VK_NULL_HANDLE;
    VkPipeline pTree_ = VK_NULL_HANDLE, pFine_ = VK_NULL_HANDLE, pSpray_ = VK_NULL_HANDLE, pDyn_ = VK_NULL_HANDLE;
    Slot slots_[FRAMES];
    int slot_ = 0;
    bool recording_ = false, acquired_ = false, waited_ = false;
    uint32_t image_ = 0;
    std::vector<Block> blocks_;
    Alloc quadIdx_, tileIdx_, treeAlloc_;
    std::vector<Mesh> meshes_;
    std::vector<MeshId> free_;
    Dyn spray_, dyn_, fineBlocks_;
    uint32_t treeCount_ = 0, sprayCount_ = 0, dynCount_ = 0, fineCount_ = 0;
    glm::ivec2 fineOrigin_{0};
    int debug_ = 0;
    HostBuf readback_;
    std::vector<const Command*> water_;   // кадр: что рисовать вторым, прозрачным проходом (Р34)
};

}  // namespace

std::unique_ptr<Backend> createVulkan(SDL_Window* window, const char* shaderDir) {
    auto backend = std::make_unique<VKBackend>(window);
    if (!backend->init(shaderDir)) return nullptr;
    return backend;
}

}  // namespace render
