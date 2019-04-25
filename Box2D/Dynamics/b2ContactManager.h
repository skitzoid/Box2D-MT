/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
* Copyright (c) 2015 Justin Hoffman https://github.com/jhoffman0x/Box2D-MT
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

#include "Box2D/Collision/b2BroadPhase.h"
#include "Box2D/Common/b2GrowableArray.h"
#include "Box2D/Dynamics/Contacts/b2Contact.h"
#include "Box2D/Dynamics/b2WorldCallbacks.h"
#include "Box2D/Dynamics/b2TimeStep.h"

class b2ContactFilter;
class b2ContactListener;
class b2BlockAllocator;
class b2Body;
struct b2FixtureProxy;

struct b2DeferredContactCreate
{
	b2Fixture* fixtureA;
	b2Fixture* fixtureB;
	int32 indexA;
	int32 indexB;
	b2ContactProxyIds proxyIds;
};

struct b2DeferredMoveProxy
{
	b2AABB aabb;
	b2Vec2 displacement;
	int32 proxyId;
};

struct b2DeferredPreSolve
{
	b2Contact* contact;
	b2Manifold oldManifold;
};

struct b2DeferredPostSolve
{
	b2Contact* contact;
	b2ContactImpulse impulse;
};

/// These are used to sort deferred events so their effects are applied in a deterministic order.
bool b2ContactPointerLessThan(const b2Contact* l, const b2Contact* r);
bool b2DeferredContactCreateLessThan(const b2DeferredContactCreate& l, const b2DeferredContactCreate& r);
bool b2DeferredMoveProxyLessThan(const b2DeferredMoveProxy& l, const b2DeferredMoveProxy& r);
bool b2DeferredPreSolveLessThan(const b2DeferredPreSolve& l, const b2DeferredPreSolve& r);
bool b2DeferredPostSolveLessThan(const b2DeferredPostSolve& l, const b2DeferredPostSolve& r);

struct b2ContactManagerPerThreadData
{
	b2GrowableArray<b2Contact*> m_deferredBeginContacts;
	b2GrowableArray<b2Contact*> m_deferredEndContacts;
	b2GrowableArray<b2DeferredPreSolve> m_deferredPreSolves;
	b2GrowableArray<b2DeferredPostSolve> m_deferredPostSolves;
	b2GrowableArray<b2Contact*> m_deferredAwakes;
	b2GrowableArray<b2Contact*> m_deferredDestroys;
	b2GrowableArray<b2DeferredContactCreate> m_deferredCreates;
	b2GrowableArray<b2DeferredMoveProxy> m_deferredMoveProxies;
	b2Profile m_profile;

	uint8 m_padding[b2_cacheLineSize];
};

// Delegate of b2World.
class b2ContactManager
{
public:
	b2ContactManager();

	// Broad-phase callback.
	void AddPair(void* proxyUserDataA, void* proxyUserDataB, uint32 threadId);

	void FindNewContacts(uint32 moveBegin, uint32 moveEnd, uint32 threadId);

	void Collide(uint32 contactsBegin, uint32 contactsEnd, uint32 threadId);

	void Destroy(b2Contact* contact);

	void GenerateDeferredMoveProxies(b2Body** bodies, uint32 count, uint32 threadId);

	void ConsumeDeferredBeginContacts(uint32 threadCount);
	void ConsumeDeferredEndContacts(uint32 threadCount);
	void ConsumeDeferredPreSolves(uint32 threadCount);
	void ConsumeDeferredPostSolves(uint32 threadCount);

	void ConsumeDeferredAwakes(uint32 threadCount);
	void ConsumeDeferredDestroys(uint32 threadCount);
	void ConsumeDeferredCreates(uint32 threadCount);
	void ConsumeDeferredMoveProxies(uint32 threadCount);

	// Contacts are partitioned, with TOI eligible contacts ordered before TOI ineligible
	// contacts. This speeds up traversal during TOI solving.
	b2Contact** GetToiBegin();
	b2Contact** GetNonToiBegin();
	uint32 GetNonToiCount();

	// Reorder contacts when TOI eligibility changes.
	void RecalculateToiCandidacy(b2Body* body);
	void RecalculateToiCandidacy(b2Fixture* fixture);

	b2BroadPhase m_broadPhase;
	b2Contact* m_contactList;
	b2ContactFilter* m_contactFilter;
	b2ContactListener* m_contactListener;
	b2BlockAllocator* m_allocator;

	// This contacts array makes it easier to assign ranges of contacts to different tasks.
	// Noe: TOI partitioning is also done in this array rather than in the world's contact
	// list, but it might be better to do that in the contact list.
	b2GrowableArray<b2Contact*> m_contacts;
	int32 m_toiCount;

	b2ContactManagerPerThreadData m_perThreadData[b2_maxThreads];

	bool m_deferCreates;

private:
	void RecalculateToiCandidacy(b2Contact* contact);
	inline void OnContactCreate(b2Contact* contact, b2ContactProxyIds proxyIds);
	inline void PushContact(b2Contact* contact);
	inline void RemoveContact(b2Contact* contact);
};

inline b2Contact** b2ContactManager::GetToiBegin()
{
	return m_contacts.Data();
}

inline b2Contact** b2ContactManager::GetNonToiBegin()
{
	return m_contacts.Data() + m_toiCount;
}

inline uint32 b2ContactManager::GetNonToiCount()
{
	return m_contacts.GetCount() - m_toiCount;
}

#endif
