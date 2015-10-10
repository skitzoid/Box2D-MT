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

#include <Box2D/Dynamics/b2ContactManager.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2Fixture.h>
#include <Box2D/Dynamics/b2WorldCallbacks.h>
#include <Box2D/Dynamics/Contacts/b2Contact.h>

// TODO_JUSTIN: Settings?
const int32 b2_initialContactCapacity = 1024;
const int32 b2_initialTOIContactCapacity = 1024;
const int32 b2_initialDeferredAwakesCapacity = 1024;
const int32 b2_initialDeferredDestroysCapacity = 1024;
const int32 b2_initialDeferredCreatesCapacity = 1024;
const int32 b2_initialDeferredMoveProxiesCapacity = 1024;
const int32 b2_initialTempProxyIdsCapacity = 1024;

b2ContactFilter b2_defaultFilter;
b2ContactListener b2_defaultListener;

/// This is used to sort contacts in a deterministic order.
bool b2ContactPointerLessThan(const b2Contact* l, const b2Contact* r)
{
	int32 c1A = l->GetFixtureA()->m_proxies[l->GetChildIndexA()].proxyId;
	int32 c1B = l->GetFixtureB()->m_proxies[l->GetChildIndexB()].proxyId;
	int32 c2A = r->GetFixtureA()->m_proxies[r->GetChildIndexA()].proxyId;
	int32 c2B = r->GetFixtureB()->m_proxies[r->GetChildIndexB()].proxyId;

	if (c1A < c2A)
	{
		return true;
	}

	if (c1A == c2A)
	{
		return c1B < c2B;
	}

	return false;
}

/// This is used to sort deferred contact creations in a deterministic order.
bool b2DeferredContactCreateLessThan(const b2DeferredContactCreate& l, const b2DeferredContactCreate& r)
{
	if (l.proxyA->proxyId < r.proxyA->proxyId)
	{
		return true;
	}

	if (l.proxyA->proxyId == r.proxyA->proxyId)
	{
		return l.proxyB->proxyId < r.proxyB->proxyId;
	}

	return false;
}

bool b2DeferredMoveProxyLessThan(const b2DeferredMoveProxy& l, const b2DeferredMoveProxy& r)
{
	return l.proxy->proxyId < r.proxy->proxyId;
}

b2ContactManagerPerThreadData::b2ContactManagerPerThreadData()
: m_deferredAwakes(b2_initialDeferredAwakesCapacity)
, m_deferredDestroys(b2_initialDeferredDestroysCapacity)
, m_deferredCreates(b2_initialDeferredCreatesCapacity)
, m_deferredMoveProxies(b2_initialDeferredMoveProxiesCapacity)
, m_tempProxyIds(b2_initialTempProxyIdsCapacity)
{

}

b2ContactManager::b2ContactManager()
: m_contactsNonTOI(b2_initialContactCapacity)
, m_contactsTOI(b2_initialTOIContactCapacity)
{
	m_contactList = NULL;
	m_contactFilter = &b2_defaultFilter;
	m_contactListener = &b2_defaultListener;
	m_allocator = NULL;
	m_deferAwakenings = false;
	m_deferDestroys = false;
	m_deferCreates = false;
}

void b2ContactManager::Destroy(b2Contact* c)
{
	if (m_deferDestroys)
	{
		m_perThreadData[b2GetThreadId()].m_deferredDestroys.Push(c);
		return;
	}

	b2Fixture* fixtureA = c->GetFixtureA();
	b2Fixture* fixtureB = c->GetFixtureB();
	b2Body* bodyA = fixtureA->GetBody();
	b2Body* bodyB = fixtureB->GetBody();

	if (m_contactListener && c->IsTouching())
	{
		m_contactListener->EndContact(c);
	}

	// Remove from the world.
	if (c->m_prev)
	{
		c->m_prev->m_next = c->m_next;
	}

	if (c->m_next)
	{
		c->m_next->m_prev = c->m_prev;
	}

	if (c == m_contactList)
	{
		m_contactList = c->m_next;
	}

	if (c->m_flags & b2Contact::e_toiCandidateFlag)
	{
		m_contactsTOI.Peek()->m_managerIndex = c->m_managerIndex;
		m_contactsTOI.RemoveAndSwap(c->m_managerIndex);
	}
	else
	{
		m_contactsNonTOI.Peek()->m_managerIndex = c->m_managerIndex;
		m_contactsNonTOI.RemoveAndSwap(c->m_managerIndex);
	}

	// Remove from body 1
	if (c->m_nodeA.prev)
	{
		c->m_nodeA.prev->next = c->m_nodeA.next;
	}

	if (c->m_nodeA.next)
	{
		c->m_nodeA.next->prev = c->m_nodeA.prev;
	}

	if (&c->m_nodeA == bodyA->m_contactList)
	{
		bodyA->m_contactList = c->m_nodeA.next;
	}

	// Remove from body 2
	if (c->m_nodeB.prev)
	{
		c->m_nodeB.prev->next = c->m_nodeB.next;
	}

	if (c->m_nodeB.next)
	{
		c->m_nodeB.next->prev = c->m_nodeB.prev;
	}

	if (&c->m_nodeB == bodyB->m_contactList)
	{
		bodyB->m_contactList = c->m_nodeB.next;
	}

	// Call the factory.
	b2Contact::Destroy(c, m_allocator);
}

// This is the top level collision call for the time step. Here
// all the narrow phase collision is processed for the world
// contact list.
void b2ContactManager::Collide(b2Contact** contacts, int32 count)
{
	// Update awake contacts.
	for (int32 i = 0; i < count; ++i)
	{
		b2Contact* c = contacts[i];

		b2Fixture* fixtureA = c->GetFixtureA();
		b2Fixture* fixtureB = c->GetFixtureB();
		int32 indexA = c->GetChildIndexA();
		int32 indexB = c->GetChildIndexB();
		b2Body* bodyA = fixtureA->GetBody();
		b2Body* bodyB = fixtureB->GetBody();

		// Is this contact flagged for filtering?
		if (c->m_flags & b2Contact::e_filterFlag)
		{
			// Should these bodies collide?
			if (bodyB->ShouldCollide(bodyA) == false)
			{
				b2Contact* cNuke = c;
				c = cNuke->GetNext();
				Destroy(cNuke);
				continue;
			}

			// Check user filtering.
			if (m_contactFilter && m_contactFilter->ShouldCollide(fixtureA, fixtureB) == false)
			{
				b2Contact* cNuke = c;
				c = cNuke->GetNext();
				Destroy(cNuke);
				continue;
			}

			// Clear the filtering flag.
			c->m_flags &= ~b2Contact::e_filterFlag;
		}

		bool activeA = bodyA->IsAwake() && bodyA->m_type != b2_staticBody;
		bool activeB = bodyB->IsAwake() && bodyB->m_type != b2_staticBody;

		// At least one body must be awake and it must be dynamic or kinematic.
		if (activeA == false && activeB == false)
		{
			c = c->GetNext();
			continue;
		}

		int32 proxyIdA = fixtureA->m_proxies[indexA].proxyId;
		int32 proxyIdB = fixtureB->m_proxies[indexB].proxyId;
		bool overlap = m_broadPhase.TestOverlap(proxyIdA, proxyIdB);

		// Here we destroy contacts that cease to overlap in the broad-phase.
		if (overlap == false)
		{
			b2Contact* cNuke = c;
			c = cNuke->GetNext();
			Destroy(cNuke);
			continue;
		}

		// Awakening might be deferred to avoid a data race on body flags.
		bool canWakeBodies = m_deferAwakenings == false;

		// The contact persists.
		bool needsAwake = c->Update(m_contactListener, canWakeBodies);
		if (needsAwake)
		{
			m_perThreadData[b2GetThreadId()].m_deferredAwakes.Push(c);
		}
		c = c->GetNext();
	}
}

void b2ContactManager::FindNewContacts(int32 moveBegin, int32 moveEnd)
{
	m_broadPhase.UpdatePairs(moveBegin, moveEnd, this);
}

void b2ContactManager::AddPair(void* proxyUserDataA, void* proxyUserDataB)
{
	b2FixtureProxy* proxyA = (b2FixtureProxy*)proxyUserDataA;
	b2FixtureProxy* proxyB = (b2FixtureProxy*)proxyUserDataB;

	b2Fixture* fixtureA = proxyA->fixture;
	b2Fixture* fixtureB = proxyB->fixture;

	int32 indexA = proxyA->childIndex;
	int32 indexB = proxyB->childIndex;

	b2Body* bodyA = fixtureA->GetBody();
	b2Body* bodyB = fixtureB->GetBody();

	// Are the fixtures on the same body?
	if (bodyA == bodyB)
	{
		return;
	}

	// TODO_ERIN use a hash table to remove a potential bottleneck when both
	// bodies have a lot of contacts.
	// Does a contact already exist?
	b2ContactEdge* edge = bodyB->GetContactList();
	while (edge)
	{
		if (edge->other == bodyA)
		{
			b2Fixture* fA = edge->contact->GetFixtureA();
			b2Fixture* fB = edge->contact->GetFixtureB();
			int32 iA = edge->contact->GetChildIndexA();
			int32 iB = edge->contact->GetChildIndexB();

			if (fA == fixtureA && fB == fixtureB && iA == indexA && iB == indexB)
			{
				// A contact already exists.
				return;
			}

			if (fA == fixtureB && fB == fixtureA && iA == indexB && iB == indexA)
			{
				// A contact already exists.
				return;
			}
		}

		edge = edge->next;
	}

	// Does a joint override collision? Is at least one body dynamic?
	if (bodyB->ShouldCollide(bodyA) == false)
	{
		return;
	}

	// Check user filtering.
	if (m_contactFilter && m_contactFilter->ShouldCollide(fixtureA, fixtureB) == false)
	{
		return;
	}

	// Defer creation?
	if (m_deferCreates)
	{
		b2DeferredContactCreate deferredCreate;
		deferredCreate.proxyA = proxyA;
		deferredCreate.proxyB = proxyB;
		m_perThreadData[b2GetThreadId()].m_deferredCreates.Push(deferredCreate);
		return;
	}

	// Call the factory.
	b2Contact* c = b2Contact::Create(fixtureA, indexA, fixtureB, indexB, m_allocator);
	if (c == NULL)
	{
		return;
	}

	// Finish creating.
	OnContactCreate(c);
}

// This allows proxy synchronization to be somewhat parallel.
void b2ContactManager::GenerateDeferredMoveProxies(b2Body** bodies, int32 count)
{
	for (int32 i = 0; i < count; ++i)
	{
		b2Body* b = bodies[i];

		b2Assert(b->GetType() != b2_staticBody);

		// If a body was not in an island then it did not move.
		if ((b->m_flags & b2Body::e_islandFlag) == 0)
		{
			continue;
		}

		b2Transform xf1;
		xf1.q.Set(b->m_sweep.a0);
		xf1.p = b->m_sweep.c0 - b2Mul(xf1.q, b->m_sweep.localCenter);

		for (b2Fixture* f = b->m_fixtureList; f; f = f->m_next)
		{
			for (int32 j = 0; j < f->m_proxyCount; ++j)
			{
				b2FixtureProxy* proxy = f->m_proxies + j;

				// Compute an AABB that covers the swept shape (may miss some rotation effect).
				b2AABB aabb1, aabb2;
				f->m_shape->ComputeAABB(&aabb1, xf1, proxy->childIndex);
				f->m_shape->ComputeAABB(&aabb2, b->m_xf, proxy->childIndex);

				proxy->aabb.Combine(aabb1, aabb2);

				// A move is required if the new AABB isn't contained by the fat AABB.
				bool requiresMove = m_broadPhase.GetFatAABB(proxy->proxyId).Contains(proxy->aabb) == false;

				if (requiresMove)
				{
					b2DeferredMoveProxy moveProxy;
					moveProxy.proxy = proxy;
					moveProxy.displacement = b->m_xf.p - xf1.p;
					m_perThreadData[b2GetThreadId()].m_deferredMoveProxies.Push(moveProxy);
				}
			}
		}
	}
}

void b2ContactManager::ConsumeDeferredAwakes()
{
	b2Assert(m_deferAwakenings);

	// Awake bodies. Order doesn't affect determinism.
	for (int32 i = 0; i < b2_maxThreads; ++i)
	{
		while (m_perThreadData[i].m_deferredAwakes.GetCount())
		{
			b2Contact* c = m_perThreadData[i].m_deferredAwakes.Pop();
			c->m_nodeA.other->SetAwake(true);
			c->m_nodeB.other->SetAwake(true);
		}
	}

	m_deferAwakenings = false;
}

void b2ContactManager::ConsumeDeferredDestroys()
{
	b2Assert(m_deferDestroys);

	b2ContactManagerPerThreadData* td0 = m_perThreadData + 0;

	// Put all contacts in a single array.
	for (int32 i = 1; i < b2_maxThreads; ++i)
	{
		while (m_perThreadData[i].m_deferredDestroys.GetCount())
		{
			b2Contact* c = m_perThreadData[i].m_deferredDestroys.Pop();
			td0->m_deferredDestroys.Push(c);
		}
	}

	// Sort to ensure determinism.
	b2Contact** deferredDestroysBegin = td0->m_deferredDestroys.Data();
	int32 deferredDestroyCount = td0->m_deferredDestroys.GetCount();
	std::sort(deferredDestroysBegin, deferredDestroysBegin + deferredDestroyCount, b2ContactPointerLessThan);

	m_deferDestroys = false;

	// Destroy contacts.
	while (td0->m_deferredDestroys.GetCount())
	{
		b2Contact* c = td0->m_deferredDestroys.Pop();
		Destroy(c);
	}
}

void b2ContactManager::ConsumeDeferredCreates()
{
	b2Assert(m_deferCreates);

	b2ContactManagerPerThreadData* td = m_perThreadData;

	// Put all contacts in a single array.
	for (int32 i = 1; i < b2_maxThreads; ++i)
	{
		while (m_perThreadData[i].m_deferredCreates.GetCount())
		{
			b2DeferredContactCreate deferredCreate = m_perThreadData[i].m_deferredCreates.Pop();

			td->m_deferredCreates.Push(deferredCreate);
		}
	}

	// Sort to ensure determinism.
	b2DeferredContactCreate* deferredCreatesBegin = td->m_deferredCreates.Data();
	int32 deferredCreateCount = td->m_deferredCreates.GetCount();
	std::sort(deferredCreatesBegin, deferredCreatesBegin + deferredCreateCount, b2DeferredContactCreateLessThan);

	m_deferCreates = false;

	b2Pair prevPair;
	prevPair.proxyIdA = b2BroadPhase::e_nullProxy;
	prevPair.proxyIdB = b2BroadPhase::e_nullProxy;

	// Finish contact creation.
	while (td->m_deferredCreates.GetCount())
	{
		b2DeferredContactCreate deferredCreate = td->m_deferredCreates.Pop();

		// Store the pair for fast lookup.
		b2Pair proxyPair;
		proxyPair.proxyIdA = deferredCreate.proxyA->proxyId;
		proxyPair.proxyIdB = deferredCreate.proxyB->proxyId;

		// Already created?
		if (proxyPair.proxyIdA == prevPair.proxyIdA && proxyPair.proxyIdB == prevPair.proxyIdB)
		{
			continue;
		}

		prevPair = proxyPair;

		b2Fixture* fixtureA = deferredCreate.proxyA->fixture;
		b2Fixture* fixtureB = deferredCreate.proxyB->fixture;

		int32 indexA = deferredCreate.proxyA->childIndex;
		int32 indexB = deferredCreate.proxyB->childIndex;

		// Call the factory.
		b2Contact* c = b2Contact::Create(fixtureA, indexA, fixtureB, indexB, m_allocator);
		if (c == NULL)
		{
			return;
		}

		// Finish creating.
		OnContactCreate(c);
	}
}

void b2ContactManager::OnContactCreate(b2Contact* c)
{
	b2Fixture* fixtureA = c->GetFixtureA();
	b2Fixture* fixtureB = c->GetFixtureB();
	b2Body* bodyA = fixtureA->GetBody();
	b2Body* bodyB = fixtureB->GetBody();

	// Mark for TOI if needed.
	if (fixtureA->IsSensor() == false && fixtureB->IsSensor() == false)
	{
		bool aNeedsTOI = bodyA->IsBullet() || (bodyA->GetType() != b2_dynamicBody);
		bool bNeedsTOI = bodyB->IsBullet() || (bodyB->GetType() != b2_dynamicBody);

		if (aNeedsTOI || bNeedsTOI)
		{
			c->m_flags |= b2Contact::e_toiCandidateFlag;
		}
	}

	if (c->m_flags & b2Contact::e_toiCandidateFlag)
	{
		// Add to TOI contacts.
		c->m_managerIndex = m_contactsTOI.GetCount();
		m_contactsTOI.Push(c);
	}
	else
	{
		// Add to non-TOI contacts.
		c->m_managerIndex = m_contactsNonTOI.GetCount();
		m_contactsNonTOI.Push(c);
	}

	// Insert into the world.
	c->m_prev = NULL;
	c->m_next = m_contactList;
	if (m_contactList != NULL)
	{
		m_contactList->m_prev = c;
	}
	m_contactList = c;

	// Connect to island graph.

	// Connect to body A
	c->m_nodeA.contact = c;
	c->m_nodeA.other = bodyB;
	c->m_nodeA.next = bodyA->m_contactList;
	if (bodyA->m_contactList != NULL)
	{
		bodyA->m_contactList->prev = &c->m_nodeA;
	}
	bodyA->m_contactList = &c->m_nodeA;

	// Connect to body B
	c->m_nodeB.contact = c;
	c->m_nodeB.other = bodyA;
	c->m_nodeB.next = bodyB->m_contactList;
	if (bodyB->m_contactList != NULL)
	{
		bodyB->m_contactList->prev = &c->m_nodeB;
	}
	bodyB->m_contactList = &c->m_nodeB;

	// Wake up the bodies
	if (fixtureA->IsSensor() == false && fixtureB->IsSensor() == false)
	{
		bodyA->SetAwake(true);
		bodyB->SetAwake(true);
	}
}

void b2ContactManager::ConsumeDeferredMoveProxies()
{
	b2ContactManagerPerThreadData* td0 = m_perThreadData + 0;

	// TODO_JUSTIN put deferred move in normal broadphase move buffers?

	// Put all proxies in a single array.
	for (int32 i = 1; i < b2_maxThreads; ++i)
	{
		while (m_perThreadData[i].m_deferredMoveProxies.GetCount())
		{
			b2DeferredMoveProxy moveProxy = m_perThreadData[i].m_deferredMoveProxies.Pop();
			td0->m_deferredMoveProxies.Push(moveProxy);
		}
	}

	// Sort to ensure determinism.
	b2DeferredMoveProxy* deferredMovesBegin = td0->m_deferredMoveProxies.Data();
	int32 deferredMoveCount = td0->m_deferredMoveProxies.GetCount();
	std::sort(deferredMovesBegin, deferredMovesBegin + deferredMoveCount, b2DeferredMoveProxyLessThan);

	// Move proxies.
	while (td0->m_deferredMoveProxies.GetCount())
	{
		b2DeferredMoveProxy moveProxy = td0->m_deferredMoveProxies.Pop();
		m_broadPhase.MoveProxy(moveProxy.proxy->proxyId, moveProxy.proxy->aabb, moveProxy.displacement);
	}
}