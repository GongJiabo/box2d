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

#ifndef B2_STACK_ALLOCATOR_H
#define B2_STACK_ALLOCATOR_H

#include "b2_api.h"
#include "b2_settings.h"

// 栈内存池的大小 100KB
const int32 b2_stackSize = 100 * 1024;	// 100k
// 栈元素的最大数量
const int32 b2_maxStackEntries = 32;

// 栈实体定义
struct B2_API b2StackEntry
{
	char* data;        // 头指针
	int32 size;        // 大小
	bool usedMalloc;   // 是否使用
};

// This is a stack allocator used for fast per step allocations.
// You must nest allocate/free pairs. The code will assert
// if you try to interleave multiple allocate/free pairs.
class B2_API b2StackAllocator
{
public:
	b2StackAllocator();
	~b2StackAllocator();

	void* Allocate(int32 size);
	void Free(void* p);

	int32 GetMaxAllocation() const;

private:

    // 栈的内存池，用于栈子节点的内存开辟
	char m_data[b2_stackSize];
    // 在栈的内存池中，已使用的内存大小
	int32 m_index;
    // 栈中所有元素使用内存大小
	int32 m_allocation;
    // 占中的所有元素[不管现在是否出栈]使用的内存容量
    // 注意该变量在对象销毁之前只增不减
	int32 m_maxAllocation;
    // 栈实体的数组
	b2StackEntry m_entries[b2_maxStackEntries];
    // 栈中元素的数量
	int32 m_entryCount;
};

#endif
