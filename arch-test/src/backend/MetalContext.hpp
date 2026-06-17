#pragma once
#include "Backend.hpp"

#include <Metal/Metal.hpp>
#include "Foundation/NSString.hpp"
#include "ysim_paths.hpp"

#include <iostream>
#include <string>
#include <unordered_map>

// VectorBase is defined later (VectorBase.hpp); setBuffer only needs its
// METAL members (pool/offset), so a forward decl suffices here.
template <typename BE, typename PR>
struct VectorBase;

struct MetalGlobalContext {
    static MTL::Device* getDevice() {
        static MTL::Device* device = MTL::CreateSystemDefaultDevice();
        return device;
    }
    static MTL::CommandQueue* getCommandQueue() {
        static MTL::CommandQueue* commandQueue = getDevice()->newCommandQueue();
        return commandQueue;
    }
    inline static MTL::CommandBuffer* commandBuffer = nullptr;
    inline static MTL::ComputeCommandEncoder* computeCommandEncoder = nullptr;
    static MTL::ComputeCommandEncoder* getComputeCommandEncoder() {
        if (computeCommandEncoder) return computeCommandEncoder;
        commandBuffer = getCommandQueue()->commandBuffer();
        computeCommandEncoder = commandBuffer->computeCommandEncoder();
        return computeCommandEncoder;
    }
    template <typename BE, typename PR>
    static void setBuffer(const VectorBase<BE, PR>& vec, Index index) {
        getComputeCommandEncoder()->setBuffer(vec.pool, vec.offset, index);
    }
    template <typename PR>
    static void setBytes(const PR& data, Index index) {
        getComputeCommandEncoder()->setBytes(&data, sizeof(PR), index);
    }
    static void dispatchThreads(MTL::ComputePipelineState* pso, Index numThreads) {
        Index tg = std::min<Index>(pso->maxTotalThreadsPerThreadgroup(), numThreads);
        dispatchThreads(pso, numThreads, tg);
    }
    static void dispatchThreads(MTL::ComputePipelineState* pso, Index numThreads,
                                Index threadsPerThreadgroup) {
        Index maxTG = (Index)pso->maxTotalThreadsPerThreadgroup();
        if (threadsPerThreadgroup == 0 || threadsPerThreadgroup > maxTG) {
            std::cerr << "[MetalGlobalContext] invalid threadsPerThreadgroup: "
                      << threadsPerThreadgroup << ", max = " << maxTG << std::endl;
            return;
        }
        MTL::Size gridSize(numThreads, 1, 1);
        MTL::Size groupSize(threadsPerThreadgroup, 1, 1);
        getComputeCommandEncoder()->setComputePipelineState(pso);
        getComputeCommandEncoder()->dispatchThreads(gridSize, groupSize);
    }
    static void commitAndWait() {
        // No-op when no encoder is open: a pure-CPU part between two GPU
        // parts must not crash (DECISIONS C1 / D-030).
        if (!computeCommandEncoder) return;
        computeCommandEncoder->endEncoding();
        commandBuffer->commit();
        commandBuffer->waitUntilCompleted();
        commandBuffer = nullptr;
        computeCommandEncoder = nullptr;
    }
};

struct MetalKernelContext {
    static MTL::Library* getLibrary() {
        static MTL::Library* library = nullptr;
        if (!library) {
            NS::Error* error = nullptr;
            std::string libPath = ysim_paths::runtimeFile("default.metallib");
            auto path = NS::String::string(libPath.c_str(), NS::UTF8StringEncoding);
            library = MetalGlobalContext::getDevice()->newLibrary(path, &error);
            if (!library) {
                std::cout << "[Metal Error] Failed to load " << path->utf8String() << "!\n";
                if (error) std::cout << error->localizedDescription()->utf8String() << std::endl;
                exit(1);
            }
            path->release();
        }
        return library;
    }
    static MTL::Function* getFunction(const char* name) {
        static std::unordered_map<std::string, MTL::Function*> cache;
        auto it = cache.find(name);
        if (it != cache.end()) return it->second;
        auto nsName = NS::String::string(name, NS::UTF8StringEncoding);
        auto func = getLibrary()->newFunction(nsName);
        if (!func) {
            std::cout << "[Metal Error] Failed to load function: " << name << "\n";
            exit(1);
        }
        cache[name] = func;
        nsName->release();
        return func;
    }
    static MTL::ComputePipelineState* getPSO(const char* name) {
        static std::unordered_map<std::string, MTL::ComputePipelineState*> cache;
        auto it = cache.find(name);
        if (it != cache.end()) return it->second;
        NS::Error* error = nullptr;
        auto func = getFunction(name);
        auto pso = MetalGlobalContext::getDevice()->newComputePipelineState(func, &error);
        if (!pso) {
            std::cout << "[Metal Error] PSO creation failed: " << name << "\n";
            exit(1);
        }
        cache[name] = pso;
        return pso;
    }
};
