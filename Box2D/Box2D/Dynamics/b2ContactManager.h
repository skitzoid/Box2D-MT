/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef B2_CONTACT_MANAGER_H
#define B2_CONTACT_MANAGER_H

#include <Box2D/Collision/b2BroadPhase.h>
#include <Box2D/Common/b2GrowableArray.h>

class b2Contact;
class b2ContactFilter;
class b2ContactListener;
class b2BlockAllocator;
class b2Body;
struct b2FixtureProxy;

struct b2DeferredContactCreate
{
	b2FixtureProxy* proxyA;
	b2FixtureProxy* proxyB;
};

struct b2DeferredMoveProxy
{
	b2FixtureProxy* proxy;
	b2Vec2 displacement;
};

struct b2ContactManagerPerThreadData
{
	b2ContactManagerPerThreadData();

	b2GrowableArray<b2Contact*> m_deferredAwakes;
	b2GrowableArray<b2Contact*> m_deferredDestroys;
	b2GrowableArray<b2DeferredContactCreate> m_deferredCreates;
	b2GrowableArray<b2DeferredMoveProxy> m_deferredMoveProxies;
	b2GrowableArray<int32> m_tempProxyIds;

	char m_padding[b2_cacheLineSize];
};

// Delegate of b2World.
class b2ContactManager
{
public:
	b2ContactManager();

	// Broad-phase callback.
	void AddPair(void* proxyUserDataA, void* proxyUserDataB);

	void FindNewContacts(int32 moveBegin, int32 moveEnd);

	void Collide(b2Contact** contacts, int32 count);

	void Destroy(b2Contact* c);

	void GenerateDeferredMoveProxies(b2Body** bodies, int32 count);

	void ConsumeDeferredAwakes();
	void ConsumeDeferredDestroys();
	void ConsumeDeferredCreates();
	void ConsumeDeferredMoveProxies();

	void OnContactCreate(b2Contact* c);

	int32 GetContactCount() const;
            
	b2BroadPhase m_broadPhase;
	b2Contact* m_contactList;
	b2ContactFilter* m_contactFilter;
	b2ContactListener* m_contactListener;
	b2BlockAllocator* m_allocator;

	b2GrowableArray<b2Contact*> m_contactsNonTOI;
	b2GrowableArray<b2Contact*> m_contactsTOI;

	b2ContactManagerPerThreadData m_perThreadData[b2_maxThreads];

	bool m_deferAwakenings;
	bool m_deferDestroys;
	bool m_deferCreates;
};

inline int32 b2ContactManager::GetContactCount() const
{
	return m_contactsNonTOI.GetCount() + m_contactsTOI.GetCount();
}

#endif
