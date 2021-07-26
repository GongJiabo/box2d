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
#include "box2d/b2_dynamic_tree.h"
#include <string.h>

b2DynamicTree::b2DynamicTree()
{
	m_root = b2_nullNode;

	m_nodeCapacity = 16;
	m_nodeCount = 0;
    // 申请一块内存，创建m_nodeCapacity子节点，并清空内存中的内容（不是在自己定义的SOA上实现）
	m_nodes = (b2TreeNode*)b2Alloc(m_nodeCapacity * sizeof(b2TreeNode));
	memset(m_nodes, 0, m_nodeCapacity * sizeof(b2TreeNode));

	// Build a linked list for the free list.
    // 创建一个空闲链表
	for (int32 i = 0; i < m_nodeCapacity - 1; ++i)
	{
		m_nodes[i].next = i + 1;
		m_nodes[i].height = -1;
	}
    // 链表的最后一个字节点和孩子指针、高度都置为初始值
	m_nodes[m_nodeCapacity-1].next = b2_nullNode;
	m_nodes[m_nodeCapacity-1].height = -1;
	m_freeList = 0;

	m_insertionCount = 0;
}

b2DynamicTree::~b2DynamicTree()
{
	// This frees the entire tree in one shot.
	b2Free(m_nodes);
}

// Allocate a node from the pool. Grow the pool if necessary.
// 从内存池中申请余个节点，如果必要增大内存池
int32 b2DynamicTree::AllocateNode()
{
	// Expand the node pool as needed.
	if (m_freeList == b2_nullNode)
	{
		b2Assert(m_nodeCount == m_nodeCapacity);

		// The free list is empty. Rebuild a bigger pool.
		b2TreeNode* oldNodes = m_nodes;
		m_nodeCapacity *= 2;
		m_nodes = (b2TreeNode*)b2Alloc(m_nodeCapacity * sizeof(b2TreeNode));
		memcpy(m_nodes, oldNodes, m_nodeCount * sizeof(b2TreeNode));
		b2Free(oldNodes);

		// Build a linked list for the free list. The parent
		// pointer becomes the "next" pointer.
        // 创建一个空闲链表，父节点成为下一个指针
		for (int32 i = m_nodeCount; i < m_nodeCapacity - 1; ++i)
		{
			m_nodes[i].next = i + 1;
			m_nodes[i].height = -1;
		}
		m_nodes[m_nodeCapacity-1].next = b2_nullNode;
		m_nodes[m_nodeCapacity-1].height = -1;
		m_freeList = m_nodeCount;
	}

	// Peel a node off the free list.
    // 从空闲链表中去下一个节点，初始化该节点，
    // 同时将空闲链表头指针m_freeList指向下一个(并没有给新节点分配数据)
	int32 nodeId = m_freeList;
	m_freeList = m_nodes[nodeId].next;
	m_nodes[nodeId].parent = b2_nullNode;
	m_nodes[nodeId].child1 = b2_nullNode;
	m_nodes[nodeId].child2 = b2_nullNode;
	m_nodes[nodeId].height = 0;
	m_nodes[nodeId].userData = nullptr;
	m_nodes[nodeId].moved = false;
    // 增加节点数量
	++m_nodeCount;
	return nodeId;
}

// Return a node to the pool.
// 从内存池中申请一个节点，根据nodeid将一个节点内存返回到内存池中
void b2DynamicTree::FreeNode(int32 nodeId)
{
	b2Assert(0 <= nodeId && nodeId < m_nodeCapacity);
	b2Assert(0 < m_nodeCount);
	m_nodes[nodeId].next = m_freeList;
	m_nodes[nodeId].height = -1;
	m_freeList = nodeId;
	--m_nodeCount;
}

// Create a proxy in the tree as a leaf node. We return the index
// of the node instead of a pointer so that we can grow
// the node pool. 在树上创建一个叶子节点代理
int32 b2DynamicTree::CreateProxy(const b2AABB& aabb, void* userData)
{
    // 申请代理节点id
	int32 proxyId = AllocateNode();

	// Fatten the aabb.
    // 填充aabb，为节点赋值
	b2Vec2 r(b2_aabbExtension, b2_aabbExtension);
    // 保存fattenAABB
	m_nodes[proxyId].aabb.lowerBound = aabb.lowerBound - r;
	m_nodes[proxyId].aabb.upperBound = aabb.upperBound + r;
	m_nodes[proxyId].userData = userData;
	m_nodes[proxyId].height = 0;
	m_nodes[proxyId].moved = true;
    // 插入叶子节点
	InsertLeaf(proxyId);

	return proxyId;
}

// 销毁叶子节点代理
void b2DynamicTree::DestroyProxy(int32 proxyId)
{
	b2Assert(0 <= proxyId && proxyId < m_nodeCapacity);
	b2Assert(m_nodes[proxyId].IsLeaf());

	RemoveLeaf(proxyId);
	FreeNode(proxyId);
}

// 移动叶子代理
bool b2DynamicTree::MoveProxy(int32 proxyId, const b2AABB& aabb, const b2Vec2& displacement)
{
    // 验证proxyid的有效性
	b2Assert(0 <= proxyId && proxyId < m_nodeCapacity);
    // 验证是否是叶子节点
	b2Assert(m_nodes[proxyId].IsLeaf());

	// Extend AABB 扩大aabb
	b2AABB fatAABB;
	b2Vec2 r(b2_aabbExtension, b2_aabbExtension);
	fatAABB.lowerBound = aabb.lowerBound - r;
	fatAABB.upperBound = aabb.upperBound + r;

	// Predict AABB movement 预测aabb的位移
	b2Vec2 d = b2_aabbMultiplier * displacement;

    // 扩大下限
	if (d.x < 0.0f)
	{
		fatAABB.lowerBound.x += d.x;
	}
	else
	{
		fatAABB.upperBound.x += d.x;
	}

	if (d.y < 0.0f)
	{
		fatAABB.lowerBound.y += d.y;
	}
	else
	{
		fatAABB.upperBound.y += d.y;
	}

    // 如果原来节点的aabb包含了移动后的aabb(缩小了)
	const b2AABB& treeAABB = m_nodes[proxyId].aabb;
	if (treeAABB.Contains(aabb))
	{
		// The tree AABB still contains the object, but it might be too large.
		// Perhaps the object was moving fast but has since gone to sleep.
		// The huge AABB is larger than the new fat AABB.
		b2AABB hugeAABB;
		hugeAABB.lowerBound = fatAABB.lowerBound - 4.0f * r;
		hugeAABB.upperBound = fatAABB.upperBound + 4.0f * r;
        // 将新aabb且fatten后，并扩展了一圈，如果还宝包含则返回false
		if (hugeAABB.Contains(treeAABB))
		{
			// The tree AABB contains the object AABB and the tree AABB is
			// not too large. No tree update needed.
			return false;
		}

		// Otherwise the tree AABB is huge and needs to be shrunk
	}

	RemoveLeaf(proxyId);
    // 重新设置aabb
	m_nodes[proxyId].aabb = fatAABB;
    // 插入叶子节点
	InsertLeaf(proxyId);

	m_nodes[proxyId].moved = true;

	return true;
}

//　box2d是采用surface area heuristic 划分场景的。 虽然采用的是getperimter, 获取aabb的周长，然后计算代价。
// 但可以从变量名上可以找到area, totalarea，newarea,oldarea. 这可能是因为采用周长的代价更低的原因。
// 插入思路如下：计算左右子树的代价，选择代价低的节点，如果此节点是叶子节点，将新节点插入。 如果不是，重复此步操作。
// 尽量保证每个子树内aabb的周长接近
void b2DynamicTree::InsertLeaf(int32 leaf)
{
    // 插入叶子节点总数自增
	++m_insertionCount;
    // 判断该树是否为空
	if (m_root == b2_nullNode)
	{
		m_root = leaf;
		m_nodes[m_root].parent = b2_nullNode;
		return;
	}

	// Find the best sibling for this node
    // 为该节点找到最好的兄弟姐妹节点
    // 获取leaf的aabb
	b2AABB leafAABB = m_nodes[leaf].aabb;
	int32 index = m_root;
    // a) 通过遍历树上的节点，对比aabb找到代价最小的节点A
	while (m_nodes[index].IsLeaf() == false)
	{
		int32 child1 = m_nodes[index].child1;
		int32 child2 = m_nodes[index].child2;
        // 获取当前index节点的aabb周长
		float area = m_nodes[index].aabb.GetPerimeter();
        // 获取插入的leaf节点和index节点的aabb和
		b2AABB combinedAABB;
		combinedAABB.Combine(m_nodes[index].aabb, leafAABB);
		float combinedArea = combinedAABB.GetPerimeter();

		// Cost of creating a new parent for this node and the new leaf
        // 为该节点创建一个父新的父节点（一个新叶子节点）的代价
		float cost = 2.0f * combinedArea;

		// Minimum cost of pushing the leaf further down the tree
        // 把leaf往树的下一层插入的最小cost
		float inheritanceCost = 2.0f * (combinedArea - area);

		// Cost of descending into child1 把叶子节点降级到child1的cost
		float cost1;
		if (m_nodes[child1].IsLeaf())
		{
			b2AABB aabb;
			aabb.Combine(leafAABB, m_nodes[child1].aabb);
			cost1 = aabb.GetPerimeter() + inheritanceCost;
		}
		else
		{
            // 当前index节点的child1节点不是叶子节点时
			b2AABB aabb;
			aabb.Combine(leafAABB, m_nodes[child1].aabb);
			float oldArea = m_nodes[child1].aabb.GetPerimeter();
			float newArea = aabb.GetPerimeter();        // 相比child1是叶子节点的时候cost减去了m_nodes[child1]的aabb周长
			cost1 = (newArea - oldArea) + inheritanceCost;
		}

		// Cost of descending into child2 把叶子节点降级到child2的cost
		float cost2;
		if (m_nodes[child2].IsLeaf())
		{
			b2AABB aabb;
			aabb.Combine(leafAABB, m_nodes[child2].aabb);
			cost2 = aabb.GetPerimeter() + inheritanceCost;
		}
		else
		{
			b2AABB aabb;
			aabb.Combine(leafAABB, m_nodes[child2].aabb);
			float oldArea = m_nodes[child2].aabb.GetPerimeter();
			float newArea = aabb.GetPerimeter();
			cost2 = newArea - oldArea + inheritanceCost;
		}

		// Descend according to the minimum cost.
		if (cost < cost1 && cost < cost2)
		{
			break;
		}

		// Descend
		if (cost1 < cost2)
		{
			index = child1;
		}
		else
		{
			index = child2;
		}
	}

    // b) 将A作为兄弟节点，先建一个父节点N，将A的父节点的孩子指针指向N，同时N将A和要插入的叶子节点L作为左右孩子节点连接起来
    // Quote: https://blog.csdn.net/cg0206/article/details/8293049
	int32 sibling = index;

	// Create a new parent. 创建一个新的父节点并初始化
	int32 oldParent = m_nodes[sibling].parent;
	int32 newParent = AllocateNode();
	m_nodes[newParent].parent = oldParent;
	m_nodes[newParent].userData = nullptr;
	m_nodes[newParent].aabb.Combine(leafAABB, m_nodes[sibling].aabb);
	m_nodes[newParent].height = m_nodes[sibling].height + 1;

	if (oldParent != b2_nullNode)
	{
		// The sibling was not the root. 兄弟节点不是根节点
		if (m_nodes[oldParent].child1 == sibling)
		{
			m_nodes[oldParent].child1 = newParent;
		}
		else
		{
			m_nodes[oldParent].child2 = newParent;
		}

		m_nodes[newParent].child1 = sibling;
		m_nodes[newParent].child2 = leaf;
		m_nodes[sibling].parent = newParent;
		m_nodes[leaf].parent = newParent;
	}
	else
	{
		// The sibling was the root. 兄弟节点是跟节点
		m_nodes[newParent].child1 = sibling;
		m_nodes[newParent].child2 = leaf;
		m_nodes[sibling].parent = newParent;
		m_nodes[leaf].parent = newParent;
		m_root = newParent;
	}

	// Walk back up the tree fixing heights and AABBs
    // 向后走修复树的高度和aabb
    // c) 如果出现树不平衡，旋转动态树，使它成为新的平衡二叉树
	index = m_nodes[leaf].parent;
	while (index != b2_nullNode)
	{
        // 平衡
		index = Balance(index);
        // 左右孩子节点
		int32 child1 = m_nodes[index].child1;
		int32 child2 = m_nodes[index].child2;

		b2Assert(child1 != b2_nullNode);
		b2Assert(child2 != b2_nullNode);
        // 获取高度和aabb
		m_nodes[index].height = 1 + b2Max(m_nodes[child1].height, m_nodes[child2].height);
		m_nodes[index].aabb.Combine(m_nodes[child1].aabb, m_nodes[child2].aabb);
        // 获取parent节点
		index = m_nodes[index].parent;
	}

	//Validate();
}

// 关于删除函数，主要步骤是：
// a)、根据索引找到父节点、祖父节点、和兄弟节点
// b)、将祖父节点的原本指向父节点的孩子指针指向兄弟节点，并释放父亲节点
// c)、如果出现树不平衡，旋转动态树，使它成为新的平衡二叉树。
// 还要注意一点的就是我们插入的有用的信息都是叶子节点，剩下的节点全部都是辅助节点
// ----- 与树的旋转有关 ------ //
void b2DynamicTree::RemoveLeaf(int32 leaf)
{
    // leaf是根节点 只有该一个节点
	if (leaf == m_root)
	{
		m_root = b2_nullNode;
		return;
	}
    // 获取A的父节点和祖父节点
	int32 parent = m_nodes[leaf].parent;
	int32 grandParent = m_nodes[parent].parent;
	int32 sibling;
    // 寻找兄弟节点
	if (m_nodes[parent].child1 == leaf)
	{
		sibling = m_nodes[parent].child2;
	}
	else
	{
		sibling = m_nodes[parent].child1;
	}
    
    // 如果祖父节点不为空，即父节点不是跟节点
	if (grandParent != b2_nullNode)
	{
		// Destroy parent and connect sibling to grandParent.
        // 销毁父节点并将兄弟节点连接到祖父节点中(兄弟节点作为祖父节点的原父节点对应的child)
		if (m_nodes[grandParent].child1 == parent)
		{
			m_nodes[grandParent].child1 = sibling;
		}
		else
		{
			m_nodes[grandParent].child2 = sibling;
		}
        // 将兄弟节点的父节点设置为祖父节点
		m_nodes[sibling].parent = grandParent;
        // 释放父节点的空间到空闲内存池
		FreeNode(parent);

		// Adjust ancestor bounds.  调整祖父界限
		int32 index = grandParent;
		while (index != b2_nullNode)
		{
            // 平衡
			index = Balance(index);
            // 获取左右孩子
			int32 child1 = m_nodes[index].child1;
			int32 child2 = m_nodes[index].child2;
            // 合并aabb并获取高度
			m_nodes[index].aabb.Combine(m_nodes[child1].aabb, m_nodes[child2].aabb);
			m_nodes[index].height = 1 + b2Max(m_nodes[child1].height, m_nodes[child2].height);
            // 更新index
			index = m_nodes[index].parent;
		}
	}
    //  如果祖父节点为空，即父节点为根节点，将兄弟节点置为根节点
	else
	{
		m_root = sibling;
		m_nodes[sibling].parent = b2_nullNode;
		FreeNode(parent);
	}

	//Validate();
}

// Perform a left or right rotation if node A is imbalanced.
// Returns the new root index.
// 如果子树A不平衡，则执行一个向左或向右旋转， 具体步骤：
//a) 获取当前节点的两个孩子节点，比对孩子子树B、C的高度，求的高度差b
//b) 若b不在[-1,1]之间，则上旋孩子子树较高的（这里假设是B），将父节点作为子树B的一个孩子，同时树的根节点指针m_root指向B节点
//c) 然后在B子树上进行a、b操作，直到叶子节点。同时返回新的根节点
int32 b2DynamicTree::Balance(int32 iA)
{
	b2Assert(iA != b2_nullNode);

	b2TreeNode* A = m_nodes + iA;
	if (A->IsLeaf() || A->height < 2)
	{
		return iA;
	}

	int32 iB = A->child1;
	int32 iC = A->child2;
	b2Assert(0 <= iB && iB < m_nodeCapacity);
	b2Assert(0 <= iC && iC < m_nodeCapacity);

	b2TreeNode* B = m_nodes + iB;
	b2TreeNode* C = m_nodes + iC;

	int32 balance = C->height - B->height;

	// Rotate C up
	if (balance > 1)
	{
		int32 iF = C->child1;
		int32 iG = C->child2;
		b2TreeNode* F = m_nodes + iF;
		b2TreeNode* G = m_nodes + iG;
		b2Assert(0 <= iF && iF < m_nodeCapacity);
		b2Assert(0 <= iG && iG < m_nodeCapacity);

		// Swap A and C
		C->child1 = iA;
		C->parent = A->parent;
		A->parent = iC;

		// A's old parent should point to C
		if (C->parent != b2_nullNode)
		{
			if (m_nodes[C->parent].child1 == iA)
			{
				m_nodes[C->parent].child1 = iC;
			}
			else
			{
				b2Assert(m_nodes[C->parent].child2 == iA);
				m_nodes[C->parent].child2 = iC;
			}
		}
		else
		{
			m_root = iC;
		}

		// Rotate
		if (F->height > G->height)
		{
			C->child2 = iF;
			A->child2 = iG;
			G->parent = iA;
			A->aabb.Combine(B->aabb, G->aabb);
			C->aabb.Combine(A->aabb, F->aabb);

			A->height = 1 + b2Max(B->height, G->height);
			C->height = 1 + b2Max(A->height, F->height);
		}
		else
		{
			C->child2 = iG;
			A->child2 = iF;
			F->parent = iA;
			A->aabb.Combine(B->aabb, F->aabb);
			C->aabb.Combine(A->aabb, G->aabb);

			A->height = 1 + b2Max(B->height, F->height);
			C->height = 1 + b2Max(A->height, G->height);
		}

		return iC;
	}
	
	// Rotate B up
	if (balance < -1)
	{
		int32 iD = B->child1;
		int32 iE = B->child2;
		b2TreeNode* D = m_nodes + iD;
		b2TreeNode* E = m_nodes + iE;
		b2Assert(0 <= iD && iD < m_nodeCapacity);
		b2Assert(0 <= iE && iE < m_nodeCapacity);

		// Swap A and B
		B->child1 = iA;
		B->parent = A->parent;
		A->parent = iB;

		// A's old parent should point to B
		if (B->parent != b2_nullNode)
		{
			if (m_nodes[B->parent].child1 == iA)
			{
				m_nodes[B->parent].child1 = iB;
			}
			else
			{
				b2Assert(m_nodes[B->parent].child2 == iA);
				m_nodes[B->parent].child2 = iB;
			}
		}
		else
		{
			m_root = iB;
		}

		// Rotate
		if (D->height > E->height)
		{
			B->child2 = iD;
			A->child1 = iE;
			E->parent = iA;
			A->aabb.Combine(C->aabb, E->aabb);
			B->aabb.Combine(A->aabb, D->aabb);

			A->height = 1 + b2Max(C->height, E->height);
			B->height = 1 + b2Max(A->height, D->height);
		}
		else
		{
			B->child2 = iE;
			A->child1 = iD;
			D->parent = iA;
			A->aabb.Combine(C->aabb, D->aabb);
			B->aabb.Combine(A->aabb, E->aabb);

			A->height = 1 + b2Max(C->height, D->height);
			B->height = 1 + b2Max(A->height, E->height);
		}

		return iB;
	}

	return iA;
}

int32 b2DynamicTree::GetHeight() const
{
	if (m_root == b2_nullNode)
	{
		return 0;
	}

	return m_nodes[m_root].height;
}

//
float b2DynamicTree::GetAreaRatio() const
{
	if (m_root == b2_nullNode)
	{
		return 0.0f;
	}

	const b2TreeNode* root = m_nodes + m_root;
	float rootArea = root->aabb.GetPerimeter();

	float totalArea = 0.0f;
	for (int32 i = 0; i < m_nodeCapacity; ++i)
	{
		const b2TreeNode* node = m_nodes + i;
		if (node->height < 0)
		{
			// Free node in pool
			continue;
		}

		totalArea += node->aabb.GetPerimeter();
	}

	return totalArea / rootArea;
}

// Compute the height of a sub-tree.
int32 b2DynamicTree::ComputeHeight(int32 nodeId) const
{
	b2Assert(0 <= nodeId && nodeId < m_nodeCapacity);
	b2TreeNode* node = m_nodes + nodeId;

	if (node->IsLeaf())
	{
		return 0;
	}

	int32 height1 = ComputeHeight(node->child1);
	int32 height2 = ComputeHeight(node->child2);
	return 1 + b2Max(height1, height2);
}

int32 b2DynamicTree::ComputeHeight() const
{
	int32 height = ComputeHeight(m_root);
	return height;
}

void b2DynamicTree::ValidateStructure(int32 index) const
{
	if (index == b2_nullNode)
	{
		return;
	}

	if (index == m_root)
	{
		b2Assert(m_nodes[index].parent == b2_nullNode);
	}

	const b2TreeNode* node = m_nodes + index;

	int32 child1 = node->child1;
	int32 child2 = node->child2;

	if (node->IsLeaf())
	{
		b2Assert(child1 == b2_nullNode);
		b2Assert(child2 == b2_nullNode);
		b2Assert(node->height == 0);
		return;
	}

	b2Assert(0 <= child1 && child1 < m_nodeCapacity);
	b2Assert(0 <= child2 && child2 < m_nodeCapacity);

	b2Assert(m_nodes[child1].parent == index);
	b2Assert(m_nodes[child2].parent == index);

	ValidateStructure(child1);
	ValidateStructure(child2);
}

void b2DynamicTree::ValidateMetrics(int32 index) const
{
	if (index == b2_nullNode)
	{
		return;
	}

	const b2TreeNode* node = m_nodes + index;

	int32 child1 = node->child1;
	int32 child2 = node->child2;

	if (node->IsLeaf())
	{
		b2Assert(child1 == b2_nullNode);
		b2Assert(child2 == b2_nullNode);
		b2Assert(node->height == 0);
		return;
	}

	b2Assert(0 <= child1 && child1 < m_nodeCapacity);
	b2Assert(0 <= child2 && child2 < m_nodeCapacity);

	int32 height1 = m_nodes[child1].height;
	int32 height2 = m_nodes[child2].height;
	int32 height;
	height = 1 + b2Max(height1, height2);
	b2Assert(node->height == height);

	b2AABB aabb;
	aabb.Combine(m_nodes[child1].aabb, m_nodes[child2].aabb);

	b2Assert(aabb.lowerBound == node->aabb.lowerBound);
	b2Assert(aabb.upperBound == node->aabb.upperBound);

	ValidateMetrics(child1);
	ValidateMetrics(child2);
}

void b2DynamicTree::Validate() const
{
#if defined(b2DEBUG)
	ValidateStructure(m_root);
	ValidateMetrics(m_root);

	int32 freeCount = 0;
	int32 freeIndex = m_freeList;
	while (freeIndex != b2_nullNode)
	{
		b2Assert(0 <= freeIndex && freeIndex < m_nodeCapacity);
		freeIndex = m_nodes[freeIndex].next;
		++freeCount;
	}

	b2Assert(GetHeight() == ComputeHeight());

	b2Assert(m_nodeCount + freeCount == m_nodeCapacity);
#endif
}

int32 b2DynamicTree::GetMaxBalance() const
{
	int32 maxBalance = 0;
	for (int32 i = 0; i < m_nodeCapacity; ++i)
	{
		const b2TreeNode* node = m_nodes + i;
		if (node->height <= 1)
		{
			continue;
		}

		b2Assert(node->IsLeaf() == false);

		int32 child1 = node->child1;
		int32 child2 = node->child2;
		int32 balance = b2Abs(m_nodes[child2].height - m_nodes[child1].height);
		maxBalance = b2Max(maxBalance, balance);
	}

	return maxBalance;
}

// 重新构建一棵树
// a) 获取所有叶子节点放入动态数组中，其他释放到叶子节点内存池中
// b) 获取所有叶子节点中aabb最小的两个，申请一个节点，作为它们的父节点，形成一个新的子树
// c) 用动态数组最后一个节点覆盖最小aabb的节点，用父节点放入动态数组aabb第二小的位置上。同时将动态数组中的节点个数减少一个
// d) 重复a、b、c步骤，直到动态数组中的节点个数为1个
// e) 获取头指针
void b2DynamicTree::RebuildBottomUp()
{
	int32* nodes = (int32*)b2Alloc(m_nodeCount * sizeof(int32));
	int32 count = 0;

	// Build array of leaves. Free the rest.
	for (int32 i = 0; i < m_nodeCapacity; ++i)
	{
		if (m_nodes[i].height < 0)
		{
			// free node in pool
			continue;
		}

		if (m_nodes[i].IsLeaf())
		{
			m_nodes[i].parent = b2_nullNode;
			nodes[count] = i;
			++count;
		}
		else
		{
			FreeNode(i);
		}
	}

	while (count > 1)
	{
		float minCost = b2_maxFloat;
		int32 iMin = -1, jMin = -1;
		for (int32 i = 0; i < count; ++i)
		{
			b2AABB aabbi = m_nodes[nodes[i]].aabb;

			for (int32 j = i + 1; j < count; ++j)
			{
				b2AABB aabbj = m_nodes[nodes[j]].aabb;
				b2AABB b;
				b.Combine(aabbi, aabbj);
				float cost = b.GetPerimeter();
				if (cost < minCost)
				{
					iMin = i;
					jMin = j;
					minCost = cost;
				}
			}
		}

		int32 index1 = nodes[iMin];
		int32 index2 = nodes[jMin];
		b2TreeNode* child1 = m_nodes + index1;
		b2TreeNode* child2 = m_nodes + index2;

		int32 parentIndex = AllocateNode();
		b2TreeNode* parent = m_nodes + parentIndex;
		parent->child1 = index1;
		parent->child2 = index2;
		parent->height = 1 + b2Max(child1->height, child2->height);
		parent->aabb.Combine(child1->aabb, child2->aabb);
		parent->parent = b2_nullNode;

		child1->parent = parentIndex;
		child2->parent = parentIndex;

		nodes[jMin] = nodes[count-1];
		nodes[iMin] = parentIndex;
		--count;
	}

	m_root = nodes[0];
	b2Free(nodes);

	Validate();
}

void b2DynamicTree::ShiftOrigin(const b2Vec2& newOrigin)
{
	// Build array of leaves. Free the rest.
	for (int32 i = 0; i < m_nodeCapacity; ++i)
	{
		m_nodes[i].aabb.lowerBound -= newOrigin;
		m_nodes[i].aabb.upperBound -= newOrigin;
	}
}
