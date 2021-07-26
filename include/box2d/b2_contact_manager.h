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

#ifndef B2_CONTACT_MANAGER_H
#define B2_CONTACT_MANAGER_H

#include "b2_api.h"
#include "b2_broad_phase.h"

class b2Contact;
class b2ContactFilter;
class b2ContactListener;
class b2BlockAllocator;

// Delegate of b2World.
class B2_API b2ContactManager
{
public:
	b2ContactManager();

	// Broad-phase callback.
    // AABB框开始重叠(fixture自身并没有发生重叠，但此时相关物体的b2Contact对象被创建并添加到接触链表中)
    // 结果: 接触对象存在与接触链表中，但IsTouching()返回false
	void AddPair(void* proxyUserDataA, void* proxyUserDataB);

	void FindNewContacts();

	void Destroy(b2Contact* c);

	void Collide();

	b2BroadPhase m_broadPhase;
	b2Contact* m_contactList;       // core members!!
	int32 m_contactCount;
	b2ContactFilter* m_contactFilter;
	b2ContactListener* m_contactListener;
	b2BlockAllocator* m_allocator;
};

#endif
