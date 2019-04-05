/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
* Copyright (c) 2015, Justin Hoffman https://github.com/skitzoid
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

#include "Box2D/Dynamics/b2ContactManager.h"
#include "Box2D/Dynamics/b2Body.h"
#include "Box2D/Dynamics/b2Fixture.h"
#include "Box2D/Dynamics/Contacts/b2Contact.h"

/// A do-nothing contact listener.
class b2DefaultContactListener : public b2ContactListener
{
public:
	b2ImmediateCallbackResult BeginContactImmediate(b2Contact* contact) override
	{
		B2_NOT_USED(contact);
		return b2ImmediateCallbackResult::DO_NOT_CALL_DEFERRED;
	}
	b2ImmediateCallbackResult EndContactImmediate(b2Contact* contact) override
	{
		B2_NOT_USED(contact);
		return b2ImmediateCallbackResult::DO_NOT_CALL_DEFERRED;
	}
	b2ImmediateCallbackResult PreSolveImmediate(b2Contact* contact, const b2Manifold* oldManifold) override
	{
		B2_NOT_USED(contact);
		B2_NOT_USED(oldManifold);
		return b2ImmediateCallbackResult::DO_NOT_CALL_DEFERRED;
	}
	b2ImmediateCallbackResult PostSolveImmediate(b2Contact* contact, const b2ContactImpulse* impulse) override
	{
		B2_NOT_USED(contact);
		B2_NOT_USED(impulse);
		return b2ImmediateCallbackResult::DO_NOT_CALL_DEFERRED;
	}
};

b2ContactFilter b2_defaultFilter;
b2DefaultContactListener b2_defaultListener;

/// This is used to sort contacts in a deterministic order.
bool b2ContactPointerLessThan(const b2Contact* l, const b2Contact* r)
{
	uint32 leftToi = (l->m_flags & b2Contact::e_toiCandidateFlag) ? 1 : 0;
	uint32 leftId =  (leftToi << 31) | l->m_managerIndex;

	uint32 rightToi = (r->m_flags & b2Contact::e_toiCandidateFlag) ? 1 : 0;
	uint32 rightId =  (rightToi << 31) | r->m_managerIndex;

	return leftId < rightId;
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

template<typename T, typename CompType>
b2GrowableArray<T>* b2SortPerThreadData(b2ContactManagerPerThreadData* td,
	b2GrowableArray<T> b2ContactManagerPerThreadData::*member,
	CompType compFunc)
{
	b2ContactManagerPerThreadData& td0 = td[0];

	for (int32 i = 1; i < b2_maxThreads; ++i)
	{
		while ((td[i].*member).GetCount())
		{
			T data = (td[i].*member).Pop();
			(td0.*member).Push(data);
		}
	}

	T* memberDataBegin = (td0.*member).Data();
	int32 memberDataCount = (td0.*member).GetCount();
	std::sort(memberDataBegin, memberDataBegin + memberDataCount, compFunc);

	return &(td0.*member);
}

b2ContactManager::b2ContactManager()
{
	m_contactList = nullptr;
	m_contactFilter = &b2_defaultFilter;
	m_contactListener = &b2_defaultListener;
	m_allocator = nullptr;
	m_deferCreates = false;
}

void b2ContactManager::Destroy(b2Contact* c)
{
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
				m_perThreadData[b2GetThreadId()].m_deferredDestroys.Push(c);
				continue;
			}

			// Check user filtering.
			if (m_contactFilter && m_contactFilter->ShouldCollide(fixtureA, fixtureB) == false)
			{
				m_perThreadData[b2GetThreadId()].m_deferredDestroys.Push(c);
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
			continue;
		}

		int32 proxyIdA = fixtureA->m_proxies[indexA].proxyId;
		int32 proxyIdB = fixtureB->m_proxies[indexB].proxyId;
		bool overlap = m_broadPhase.TestOverlap(proxyIdA, proxyIdB);

		// Here we destroy contacts that cease to overlap in the broad-phase.
		if (overlap == false)
		{
			m_perThreadData[b2GetThreadId()].m_deferredDestroys.Push(c);
			continue;
		}

		// The contact persists.
		c->Update(m_perThreadData[b2GetThreadId()], m_contactListener);
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

	if (m_deferCreates)
	{
		// Defer creation.
		b2DeferredContactCreate deferredCreate;
		deferredCreate.proxyA = proxyA;
		deferredCreate.proxyB = proxyB;
		m_perThreadData[b2GetThreadId()].m_deferredCreates.Push(deferredCreate);
	}
	else
	{
		// Call the factory.
		b2Contact* c = b2Contact::Create(fixtureA, indexA, fixtureB, indexB, m_allocator);
		if (c == nullptr)
		{
			return;
		}

		OnContactCreate(c);
	}
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

void b2ContactManager::ConsumeDeferredBeginContacts()
{
	b2GrowableArray<b2Contact*>* begins = b2SortPerThreadData(m_perThreadData,
		&b2ContactManagerPerThreadData::m_deferredBeginContacts, b2ContactPointerLessThan);

	while (begins->GetCount())
	{
		m_contactListener->BeginContact(begins->Pop());
	}
}

void b2ContactManager::ConsumeDeferredEndContacts()
{
	b2GrowableArray<b2Contact*>* ends = b2SortPerThreadData(m_perThreadData,
		&b2ContactManagerPerThreadData::m_deferredEndContacts, b2ContactPointerLessThan);

	while (ends->GetCount())
	{
		m_contactListener->EndContact(ends->Pop());
	}
}

void b2ContactManager::ConsumeDeferredPreSolves()
{
	b2GrowableArray<b2DeferredPreSolve>* preSolves = b2SortPerThreadData(m_perThreadData,
		&b2ContactManagerPerThreadData::m_deferredPreSolves,
		[](const b2DeferredPreSolve& l, const b2DeferredPreSolve& r)
		{
			return b2ContactPointerLessThan(l.contact, r.contact);
		});

	while (preSolves->GetCount())
	{
		const b2DeferredPreSolve& ps = preSolves->Pop();
		m_contactListener->PreSolve(ps.contact, &ps.oldManifold);
	}
}

void b2ContactManager::ConsumeDeferredPostSolves()
{
	b2GrowableArray<b2DeferredPostSolve>* postSolves = b2SortPerThreadData(m_perThreadData,
		&b2ContactManagerPerThreadData::m_deferredPostSolves,
		[](const b2DeferredPostSolve& l, const b2DeferredPostSolve& r)
		{
			return b2ContactPointerLessThan(l.contact, r.contact);
		});

	while (postSolves->GetCount())
	{
		const b2DeferredPostSolve& ps = postSolves->Pop();
		m_contactListener->PostSolve(ps.contact, &ps.impulse);
	}
}

void b2ContactManager::ConsumeDeferredAwakes()
{
	// Order doesn't affect determinism so don't bother sorting.
	for (int32 i = 0; i < b2_maxThreads; ++i)
	{
		while (m_perThreadData[i].m_deferredAwakes.GetCount())
		{
			b2Contact* c = m_perThreadData[i].m_deferredAwakes.Pop();
			c->m_nodeA.other->SetAwake(true);
			c->m_nodeB.other->SetAwake(true);
		}
	}
}

void b2ContactManager::ConsumeDeferredDestroys()
{
	b2GrowableArray<b2Contact*>* destroys = b2SortPerThreadData(m_perThreadData,
		&b2ContactManagerPerThreadData::m_deferredDestroys, b2ContactPointerLessThan);

	while (destroys->GetCount())
	{
		b2Contact* c = destroys->Pop();
		Destroy(c);
	}
}

void b2ContactManager::ConsumeDeferredCreates()
{
	b2GrowableArray<b2DeferredContactCreate>* creates = b2SortPerThreadData(m_perThreadData,
		&b2ContactManagerPerThreadData::m_deferredCreates, b2DeferredContactCreateLessThan);

	b2Pair prevPair;
	prevPair.proxyIdA = b2BroadPhase::e_nullProxy;
	prevPair.proxyIdB = b2BroadPhase::e_nullProxy;

	// Finish contact creation.
	while (creates->GetCount())
	{
		b2DeferredContactCreate deferredCreate = creates->Pop();

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
		if (c == nullptr)
		{
			return;
		}

		// Finish creating.
		OnContactCreate(c);
	}
}

void b2ContactManager::ConsumeDeferredMoveProxies()
{
	b2GrowableArray<b2DeferredMoveProxy>* moves = b2SortPerThreadData(m_perThreadData,
		&b2ContactManagerPerThreadData::m_deferredMoveProxies, b2DeferredMoveProxyLessThan);

	while (moves->GetCount())
	{
		b2DeferredMoveProxy moveProxy = moves->Pop();
		m_broadPhase.MoveProxy(moveProxy.proxy->proxyId, moveProxy.proxy->aabb, moveProxy.displacement);
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
		bool aNeedsTOI = bodyA->IsBullet() || (bodyA->GetType() != b2_dynamicBody && !bodyA->GetPreferNoCCD());
		bool bNeedsTOI = bodyB->IsBullet() || (bodyB->GetType() != b2_dynamicBody && !bodyB->GetPreferNoCCD());

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
	c->m_prev = nullptr;
	c->m_next = m_contactList;
	if (m_contactList != nullptr)
	{
		m_contactList->m_prev = c;
	}
	m_contactList = c;

	// Connect to island graph.

	// Connect to body A
	c->m_nodeA.contact = c;
	c->m_nodeA.other = bodyB;

	c->m_nodeA.prev = nullptr;
	c->m_nodeA.next = bodyA->m_contactList;
	if (bodyA->m_contactList != nullptr)
	{
		bodyA->m_contactList->prev = &c->m_nodeA;
	}
	bodyA->m_contactList = &c->m_nodeA;

	// Connect to body B
	c->m_nodeB.contact = c;
	c->m_nodeB.other = bodyA;

	c->m_nodeB.prev = nullptr;
	c->m_nodeB.next = bodyB->m_contactList;
	if (bodyB->m_contactList != nullptr)
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
