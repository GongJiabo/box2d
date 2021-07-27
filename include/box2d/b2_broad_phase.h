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

#ifndef B2_BROAD_PHASE_H
#define B2_BROAD_PHASE_H

#include "b2_api.h"
#include "b2_settings.h"
#include "b2_collision.h"
#include "b2_dynamic_tree.h"

// piar定义
struct B2_API b2Pair
{
	int32 proxyIdA;     // 代理A
	int32 proxyIdB;     // 代理B
};

/// The broad-phase is used for computing pairs and performing volume queries and ray casts.
/// This broad-phase does not persist pairs. Instead, this reports potentially new pairs.
/// It is up to the client to consume the new pairs and to track subsequent overlap.
class B2_API b2BroadPhase
{
public:
    // 空节点代理
	enum
	{
		e_nullProxy = -1
	};

	b2BroadPhase();
	~b2BroadPhase();

	/// Create a proxy with an initial AABB. Pairs are not reported until
	/// UpdatePairs is called.
    /// 创建一个代理，并用aabb初始化  pairs不会汇报直到UpdatePairs被调用
	int32 CreateProxy(const b2AABB& aabb, void* userData);

	/// Destroy a proxy. It is up to the client to remove any pairs.
    /// 销毁一个代理，任何pairs的删除都取决于客户端
	void DestroyProxy(int32 proxyId);

	/// Call MoveProxy as many times as you like, then when you are done
	/// call UpdatePairs to finalized the proxy pairs (for your time step).
    /// 移动一个代理，只要你喜欢可以多次调用MoveProxy
    /// 点那个你完成后调用UpdatePairs用于完成代理pairs(在时间步内)
	void MoveProxy(int32 proxyId, const b2AABB& aabb, const b2Vec2& displacement);

	/// Call to trigger a re-processing of it's pairs on the next call to UpdatePairs.
    /// 在下次调用UpdatePairs时，调用一个触发器触发它的pairs
	void TouchProxy(int32 proxyId);

	/// Get the fat AABB for a proxy.
	const b2AABB& GetFatAABB(int32 proxyId) const;

	/// Get user data from a proxy. Returns nullptr if the id is invalid.
	void* GetUserData(int32 proxyId) const;

	/// Test overlap of fat AABBs.
	bool TestOverlap(int32 proxyIdA, int32 proxyIdB) const;

	/// Get the number of proxies.
	int32 GetProxyCount() const;

	/// Update the pairs. This results in pair callbacks. This can only add pairs.
    /***********************************************************************
        * 功能描述： 更新pairs.这会对pair进行回调。只能添加pairs
        * 参数说明： callback ：回调对象
        * 返 回 值： (void)
    ***************************************************************************/
	template <typename T>
	void UpdatePairs(T* callback);

	/// Query an AABB for overlapping proxies. The callback class
	/// is called for each proxy that overlaps the supplied AABB.
    /// 在重叠代理中查询一个aabb 每个提供aabb重叠的代理将会被回调类调用
	template <typename T>
	void Query(T* callback, const b2AABB& aabb) const;

	/// Ray-cast against the proxies in the tree. This relies on the callback
	/// to perform a exact ray-cast in the case were the proxy contains a shape.
	/// The callback also performs the any collision filtering. This has performance
	/// roughly equal to k * log(n), where k is the number of collisions and n is the
	/// number of proxies in the tree.
	/// @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
	/// @param callback a callback class that is called for each proxy that is hit by the ray.
    /**************************************************************************
        * 功能描述： 光线投射在树上的代理。
                 这依赖于回调被执行一个精确的光线投射在一个代理包含一个形状
        * 参数说明： callback : 一个回调对象类，当被调用时，光线将会撒到每个代理中。
                 input    ：光线投射输入数据。这个光线从p1扩展到p1+maxFraction *(p2 - p1)
        * 返 回 值： (void)
    ***************************************************************************/
	template <typename T>
	void RayCast(T* callback, const b2RayCastInput& input) const;

	/// Get the height of the embedded tree.
	int32 GetTreeHeight() const;

	/// Get the balance of the embedded tree.
	int32 GetTreeBalance() const;

	/// Get the quality metric of the embedded tree.
	float GetTreeQuality() const;

	/// Shift the world origin. Useful for large worlds.
	/// The shift formula is: position -= newOrigin
	/// @param newOrigin the new origin with respect to the old origin
	void ShiftOrigin(const b2Vec2& newOrigin);

private:
    // 友元类
	friend class b2DynamicTree;
    // 根据代理id添加代理到移动缓冲区中
	void BufferMove(int32 proxyId);
    // 将代理移出缓冲区
	void UnBufferMove(int32 proxyId);
    // 查询回调函数
	bool QueryCallback(int32 proxyId);

	b2DynamicTree m_tree;       // 动态树声明, m_tree并不能访问b2DynamicTree中的私有变量

	int32 m_proxyCount;         // 代理数量

	int32* m_moveBuffer;        // 移动的缓冲区
	int32 m_moveCapacity;       // 移动缓冲区的总容量
	int32 m_moveCount;          // 需要移动的代理数量

	b2Pair* m_pairBuffer;       // pair缓冲区
	int32 m_pairCapacity;       // pair缓冲区的总容量
	int32 m_pairCount;          // pair数量

	int32 m_queryProxyId;       // 查询代理id
};

inline void* b2BroadPhase::GetUserData(int32 proxyId) const
{
	return m_tree.GetUserData(proxyId);
}

inline bool b2BroadPhase::TestOverlap(int32 proxyIdA, int32 proxyIdB) const
{
	const b2AABB& aabbA = m_tree.GetFatAABB(proxyIdA);
	const b2AABB& aabbB = m_tree.GetFatAABB(proxyIdB);
	return b2TestOverlap(aabbA, aabbB);
}

inline const b2AABB& b2BroadPhase::GetFatAABB(int32 proxyId) const
{
	return m_tree.GetFatAABB(proxyId);
}

inline int32 b2BroadPhase::GetProxyCount() const
{
	return m_proxyCount;
}

inline int32 b2BroadPhase::GetTreeHeight() const
{
	return m_tree.GetHeight();
}

inline int32 b2BroadPhase::GetTreeBalance() const
{
	return m_tree.GetMaxBalance();
}

inline float b2BroadPhase::GetTreeQuality() const
{
	return m_tree.GetAreaRatio();
}

// !!!! 更新pair !!!
template <typename T>
void b2BroadPhase::UpdatePairs(T* callback)
{
	// Reset pair buffer
    // 重制pair缓存区
	m_pairCount = 0;

	// Perform tree queries for all moving proxies.
    // 执行查询树上所有需要移动的代理, 遍历顺序其实就是创建时的物体顺序(每个物体的aabb已经保存在树的叶子节点)
	for (int32 i = 0; i < m_moveCount; ++i)
	{
        // 获取在移动缓冲区的代理id(即在动态树节点的索引m_nodes中 -- leafnode)
		m_queryProxyId = m_moveBuffer[i];
		if (m_queryProxyId == e_nullProxy)
		{
			continue;
		}

		// We have to query the tree with the fat AABB so that
		// we don't fail to create a pair that may touch later.
        // 获取我们需要查询树的fatAABB, 以便当我们创建pair失败时, 可以再次创建
		const b2AABB& fatAABB = m_tree.GetFatAABB(m_queryProxyId);

		// Query tree, create pairs and add them pair buffer.
        // 查询树, 创建多个pair并将他们添加到pair缓冲区中
		m_tree.Query(this, fatAABB);
	}

	// Send pairs to caller 发送pairs到客户端
	for (int32 i = 0; i < m_pairCount; ++i)
	{
        // 在pair缓冲区中获取当前的pair
		b2Pair* primaryPair = m_pairBuffer + i;
        // 根据相交记录
		void* userDataA = m_tree.GetUserData(primaryPair->proxyIdA);
		void* userDataB = m_tree.GetUserData(primaryPair->proxyIdB);

		callback->AddPair(userDataA, userDataB);
	}

	// Clear move flags
	for (int32 i = 0; i < m_moveCount; ++i)
	{
		int32 proxyId = m_moveBuffer[i];
		if (proxyId == e_nullProxy)
		{
			continue;
		}

		m_tree.ClearMoved(proxyId);
	}

	// Reset move buffer 重制移动缓冲区
	m_moveCount = 0;
}

// 区域查询
template <typename T>
inline void b2BroadPhase::Query(T* callback, const b2AABB& aabb) const
{
	m_tree.Query(callback, aabb);
}

// 光线投射
template <typename T>
inline void b2BroadPhase::RayCast(T* callback, const b2RayCastInput& input) const
{
	m_tree.RayCast(callback, input);
}

inline void b2BroadPhase::ShiftOrigin(const b2Vec2& newOrigin)
{
	m_tree.ShiftOrigin(newOrigin);
}

#endif
