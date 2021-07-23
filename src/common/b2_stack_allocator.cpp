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

#include "box2d/b2_stack_allocator.h"
#include "box2d/b2_math.h"

b2StackAllocator::b2StackAllocator()
{
	m_index = 0;
	m_allocation = 0;
	m_maxAllocation = 0;
	m_entryCount = 0;
}

b2StackAllocator::~b2StackAllocator()
{
	b2Assert(m_index == 0);
	b2Assert(m_entryCount == 0);
}

void* b2StackAllocator::Allocate(int32 size)
{
    //验证栈中元素的有效性，防止内存溢出
    b2Assert(m_entryCount < b2_maxStackEntries);
    //获取栈实体头指针
    b2StackEntry* entry = m_entries + m_entryCount;
    //实体大小
    entry->size = size;
    //当内存池m_data已使用的大小加需要申请的大小大于内存池的总容量时，则在堆上申请
    if (m_index + size > b2_stackSize)
    {
        //申请大小为size的内存，并标记是在堆上申请的
        entry->data = (char*)b2Alloc(size);
        entry->usedMalloc = true;
    }
    else
    {
        //从m_data中获取内存，并标记不是在堆上申请的
        //同时修改m_index的值
        entry->data = m_data + m_index;
        entry->usedMalloc = false;
        m_index += size;
    }
    //增加栈中的所有元素使用内存大小
    m_allocation += size;
    //修改内存容量的最大值
    m_maxAllocation = b2Max(m_maxAllocation, m_allocation);
    //增加栈中元素的数量
    ++m_entryCount;
    //返回栈中元素的内存头指针
    return entry->data;
}


void b2StackAllocator::Free(void* p)
{
	b2Assert(m_entryCount > 0);
	b2StackEntry* entry = m_entries + m_entryCount - 1;
	b2Assert(p == entry->data);
	if (entry->usedMalloc)
	{
		b2Free(p);
	}
	else
	{
		m_index -= entry->size;
	}
	m_allocation -= entry->size;
	--m_entryCount;

	p = nullptr;
}

int32 b2StackAllocator::GetMaxAllocation() const
{
	return m_maxAllocation;
}
