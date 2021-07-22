// MIT License

// Copyright (c) 2019 Erin Catto

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef B2_BLOCK_ALLOCATOR_H
#define B2_BLOCK_ALLOCATOR_H

#include "b2_api.h"
#include "b2_settings.h"

// 可以申请块子节点大小的类型总数
const int32 b2_blockSizeCount = 14;

// 块子节点结构体[链表实现]声明
struct b2Block;
// 块结构体声明
struct b2Chunk;

/// This is a small object allocator used for allocating small
/// objects that persist for more than one time step.
/// See: http://www.codeproject.com/useritems/Small_Block_Allocator.asp
class B2_API b2BlockAllocator
{
public:
	b2BlockAllocator();
	~b2BlockAllocator();

	/// Allocate memory. This will use b2Alloc if the size is larger than b2_maxBlockSize.
	void* Allocate(int32 size);

	/// Free memory. This will use b2Free if the size is larger than b2_maxBlockSize.
	void Free(void* p, int32 size);

	void Clear();

private:
    // 当前块的头指针(数组)
	b2Chunk* m_chunks;
    // 当前已使用的块空间节点总数，通过m_chunks+小于m_chunkCount的偏移量i就可以找到第i+1个内存块
	int32 m_chunkCount;
    // 当前已申请的块空间节点总数
	int32 m_chunkSpace;
    // 保存了已经分配的页/块中还未被使用的块的开始地址的数组; 数组指针，每一个元素都是bBlock指针
	b2Block* m_freeLists[b2_blockSizeCount];
};

#endif
