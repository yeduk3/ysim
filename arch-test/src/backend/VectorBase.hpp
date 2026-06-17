#pragma once
#include "Backend.hpp"
#include "MemoryPool.hpp"

#include <Eigen/Dense>
#include <Eigen/Sparse>

template <typename PR>
struct VectorBase<CPU, PR> {
    PR* ptr;
    size_t size;
    VectorBase() : ptr(nullptr), size(0) {}
    VectorBase(size_t size) {
        auto block = GlobalAutoAllocator<CPU>::template alloc<PR>(size);
        this->ptr = block.ptr;
        this->size = block.size;
    }
    VectorBase(size_t size, PR fill) {
        MemoryBlock<CPU, PR> block;
        if (fill == 0) block = GlobalAutoAllocator<CPU>::template zeros<PR>(size);
        else block = GlobalAutoAllocator<CPU>::template allocFill<PR>(size, fill);
        this->ptr = block.ptr;
        this->size = block.size;
    }
    VectorBase(const MemoryBlock<CPU, PR>& block) : ptr(block.ptr), size(block.size) {}
    auto map() { return Eigen::Map<Eigen::VectorX<PR>>(ptr, size); }
    PR& operator[](Index index) { return ptr[index]; }
};

template <typename PR>
struct VectorBase<METAL, PR> {
    MTL::Buffer* pool;
    size_t offset;
    PR* ptr;
    size_t size;
    VectorBase() : pool(nullptr), offset(0), ptr(nullptr), size(0) {}
    VectorBase(size_t size) {
        auto block = GlobalAutoAllocator<METAL>::template alloc<PR>(size);
        this->pool = block.pool;
        this->offset = block.offset;
        this->ptr = block.ptr;
        this->size = block.size;
    }
    VectorBase(size_t size, PR fill) {
        MemoryBlock<METAL, PR> block;
        if (fill == 0) block = GlobalAutoAllocator<METAL>::template zeros<PR>(size);
        else block = GlobalAutoAllocator<METAL>::template allocFill<PR>(size, fill);
        this->pool = block.pool;
        this->offset = block.offset;
        this->ptr = block.ptr;
        this->size = block.size;
    }
    VectorBase(const MemoryBlock<METAL, PR>& block)
        : pool(block.pool), offset(block.offset), ptr(block.ptr), size(block.size) {}
    // Sub-view ctor (pack() slicing into a parent buffer). METAL only;
    // CPU lacks this — engine is METAL-first this pass (DECISIONS A6 / D-A6).
    VectorBase(const VectorBase<METAL, PR>& v, size_t start, size_t size) {
        this->pool = v.pool;
        this->offset = v.offset + start * sizeof(PR);
        this->ptr = v.ptr + start;
        this->size = size;
    }
    auto map() { return Eigen::Map<Eigen::VectorX<PR>>(ptr, size); }
    PR& operator[](Index index) { return ptr[index]; }
};

template <typename BE, typename PR>
struct Matrix {};
template <typename PR>
struct Matrix<CPU, PR> {
    PR* ptr;
    size_t rows, cols;
    Matrix(PR* ptr, size_t rows, size_t cols) : ptr(ptr), rows(rows), cols(cols) {}
    auto map() { return Eigen::Map<Eigen::MatrixX<PR>>(ptr, rows, cols); }
};

template <typename BE, typename PR>
struct SparseMatrix {};
template <typename PR>
struct SparseMatrix<CPU, PR> {
    Eigen::SparseMatrix<PR> data;
    size_t rows, cols;
    SparseMatrix(std::vector<Eigen::Triplet<PR>>& triplets, size_t rows, size_t cols)
        : rows(rows), cols(cols) {
        data = Eigen::SparseMatrix<PR>(rows, cols);
        data.setFromTriplets(triplets.begin(), triplets.end());
    }
    auto& map() { return data; }
};
