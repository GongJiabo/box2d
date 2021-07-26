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

#include "box2d/b2_broad_phase.h"
#include <string.h>

b2BroadPhase::b2BroadPhase()
{
	m_proxyCount = 0;

	m_pairCapacity = 16;
	m_pairCount = 0;
	m_pairBuffer = (b2Pair*)b2Alloc(m_pairCapacity * sizeof(b2Pair));

	m_moveCapacity = 16;
	m_moveCount = 0;
	m_moveBuffer = (int32*)b2Alloc(m_moveCapacity * sizeof(int32));
}

b2BroadPhase::~b2BroadPhase()
{
	b2Free(m_moveBuffer);
	b2Free(m_pairBuffer);
}

// 创建一个代理
int32 b2BroadPhase::CreateProxy(const b2AABB& aabb, void* userData)
{
    // 获取代理id并插入到动态树m_tree中
	int32 proxyId = m_tree.CreateProxy(aabb, userData);
    // 代理数量自增
	++m_proxyCount;
    // 添加代理到移动缓冲区中
	BufferMove(proxyId);
	return proxyId;
}

// 销毁一个代理
void b2BroadPhase::DestroyProxy(int32 proxyId)
{
	UnBufferMove(proxyId);
	--m_proxyCount;
	m_tree.DestroyProxy(proxyId);
}

// 移动一个代理(先操作动态树的代理id节点后，添加代理到移动缓冲区)
void b2BroadPhase::MoveProxy(int32 proxyId, const b2AABB& aabb, const b2Vec2& displacement)
{
	bool buffer = m_tree.MoveProxy(proxyId, aabb, displacement);
	if (buffer)
	{
		BufferMove(proxyId);
	}
}

// 在下次调用UpdatePiars时，调用一个触发器触发它的pairs
void b2BroadPhase::TouchProxy(int32 proxyId)
{
	BufferMove(proxyId);
}

// 根据代理id添加代理到移动缓冲区中
void b2BroadPhase::BufferMove(int32 proxyId)
{
    // 移动缓冲区达到上限，扩容
	if (m_moveCount == m_moveCapacity)
	{
		int32* oldBuffer = m_moveBuffer;
		m_moveCapacity *= 2;
		m_moveBuffer = (int32*)b2Alloc(m_moveCapacity * sizeof(int32));
		memcpy(m_moveBuffer, oldBuffer, m_moveCount * sizeof(int32));
		b2Free(oldBuffer);
	}
    // 添加代理id到移动缓冲区中
	m_moveBuffer[m_moveCount] = proxyId;
	++m_moveCount;
}

// 移除移动缓冲区
void b2BroadPhase::UnBufferMove(int32 proxyId)
{
	for (int32 i = 0; i < m_moveCount; ++i)
	{
		if (m_moveBuffer[i] == proxyId)
		{
			m_moveBuffer[i] = e_nullProxy;
		}
	}
}

// This is called from b2DynamicTree::Query when we are gathering pairs.
// 在动态树的Query函数中会进行回调该函数, proxyId为需要查询的aabb在动态树节点数组中的位置
bool b2BroadPhase::QueryCallback(int32 proxyId)
{
	// A proxy cannot form a pair with itself.
    // 一个代理不需要自己pair更新自己的pair
	if (proxyId == m_queryProxyId)
	{
		return true;
	}

    // 如果当前代理移动了
	const bool moved = m_tree.WasMoved(proxyId);
	if (moved && proxyId > m_queryProxyId)
	{
		// Both proxies are moving. Avoid duplicate pairs.
        // 当前piar中的两个代理一起移动，避免重复pairs(在pair中前一个代理处理???)
		return true;
	}

	// Grow the pair buffer as needed.
	if (m_pairCount == m_pairCapacity)
	{
		b2Pair* oldBuffer = m_pairBuffer;
		m_pairCapacity = m_pairCapacity + (m_pairCapacity >> 1);
		m_pairBuffer = (b2Pair*)b2Alloc(m_pairCapacity * sizeof(b2Pair));
		memcpy(m_pairBuffer, oldBuffer, m_pairCount * sizeof(b2Pair));
		b2Free(oldBuffer);
	}

    // 设置最新的pair并自增pair数量
	m_pairBuffer[m_pairCount].proxyIdA = b2Min(proxyId, m_queryProxyId);
	m_pairBuffer[m_pairCount].proxyIdB = b2Max(proxyId, m_queryProxyId);
	++m_pairCount;

	return true;
}
