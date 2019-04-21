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

#include "Box2D/Common/b2Timer.h"
#include "Box2D/Dynamics/b2ContactManager.h"
#include "Box2D/Dynamics/b2Body.h"
#include "Box2D/Dynamics/b2Fixture.h"
#include "Box2D/Dynamics/Contacts/b2Contact.h"

/// A do-nothing contact listener.
class b2DefaultContactListener : public b2ContactListener
{
public:
	b2ImmediateCallbackResult BeginContactImmediate(b2Contact* contact, uint32 threadId) override
	{
		B2_NOT_USED(contact);
		B2_NOT_USED(threadId);
		return b2ImmediateCallbackResult::DO_NOT_CALL_DEFERRED;
	}
	b2ImmediateCallbackResult EndContactImmediate(b2Contact* contact, uint32 threadId) override
	{
		B2_NOT_USED(contact);
		B2_NOT_USED(threadId);
		return b2ImmediateCallbackResult::DO_NOT_CALL_DEFERRED;
	}
	b2ImmediateCallbackResult PreSolveImmediate(b2Contact* contact, const b2Manifold* oldManifold, uint32 threadId) override
	{
		B2_NOT_USED(contact);
		B2_NOT_USED(oldManifold);
		B2_NOT_USED(threadId);
		return b2ImmediateCallbackResult::DO_NOT_CALL_DEFERRED;
	}
	b2ImmediateCallbackResult PostSolveImmediate(b2Contact* contact, const b2ContactImpulse* impulse, uint32 threadId) override
	{
		B2_NOT_USED(contact);
		B2_NOT_USED(impulse);
		B2_NOT_USED(threadId);
		return b2ImmediateCallbackResult::DO_NOT_CALL_DEFERRED;
	}
};

b2ContactFilter b2_defaultFilter;
b2DefaultContactListener b2_defaultListener;

bool b2ContactPointerLessThan(const b2Contact* lhs, const b2Contact* rhs)
{
	return lhs->m_proxyIds < rhs->m_proxyIds;
}

bool b2DeferredContactCreateLessThan(const b2DeferredContactCreate& lhs, const b2DeferredContactCreate& rhs)
{
	return lhs.proxyIds < rhs.proxyIds;
}

bool b2DeferredMoveProxyLessThan(const b2DeferredMoveProxy& lhs, const b2DeferredMoveProxy& rhs)
{
	return lhs.proxyId < rhs.proxyId;
}

bool b2DeferredPreSolveLessThan(const b2DeferredPreSolve& l, const b2DeferredPreSolve& r)
{
	return b2ContactPointerLessThan(l.contact, r.contact);
}

bool b2DeferredPostSolveLessThan(const b2DeferredPostSolve& l, const b2DeferredPostSolve& r)
{
	return b2ContactPointerLessThan(l.contact, r.contact);
}

template<typename T, typename CompType>
bool b2PopPerThreadData(b2ContactManagerPerThreadData* td, T& out, int32 threadCount,
	b2GrowableArray<T> b2ContactManagerPerThreadData::*member,
	CompType compFunc)
{
	int32 selectedThread = -1;
	for (int32 i = 0; i < threadCount; ++i)
	{
		if ((td[i].*member).GetCount() > 0)
		{
			selectedThread = i;
			break;
		}
	}

	if (selectedThread == -1)
	{
		return false;
	}

	for (int32 i = selectedThread + 1; i < threadCount; ++i)
	{
		if ((td[i].*member).GetCount() > 0 &&
			!compFunc((td[i].*member).Peek(), (td[selectedThread].*member).Peek()))
		{
			selectedThread = i;
		}
	}

	out = (td[selectedThread].*member).Pop();

	return true;
}

b2ContactManager::b2ContactManager()
	: m_perThreadData{}
{
	m_contactList = nullptr;
	m_contactFilter = &b2_defaultFilter;
	m_contactListener = &b2_defaultListener;
	m_allocator = nullptr;
	m_deferCreates = false;
	m_toiCount = 0;
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

	RemoveContact(c);

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
void b2ContactManager::Collide(uint32 contactsBegin, uint32 contactsEnd, uint32 threadId)
{
	b2ContactManagerPerThreadData& td = m_perThreadData[threadId];

	// Update awake contacts.
	for (uint32 i = contactsBegin; i < contactsEnd; ++i)
	{
		b2Contact* c = m_contacts[i];

		b2Fixture* fixtureA = c->GetFixtureA();
		b2Fixture* fixtureB = c->GetFixtureB();
		b2Body* bodyA = fixtureA->GetBody();
		b2Body* bodyB = fixtureB->GetBody();

		// Is this contact flagged for filtering?
		if (c->m_flags & b2Contact::e_filterFlag)
		{
			// Should these bodies collide?
			if (bodyB->ShouldCollide(bodyA) == false)
			{
				td.m_deferredDestroys.Push(c);
				continue;
			}

			// Check user filtering.
			if (m_contactFilter && m_contactFilter->ShouldCollide(fixtureA, fixtureB) == false)
			{
				td.m_deferredDestroys.Push(c);
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

		bool overlap = m_broadPhase.TestOverlap(c->m_proxyIds.low, c->m_proxyIds.high);

		// Here we destroy contacts that cease to overlap in the broad-phase.
		if (overlap == false)
		{
			td.m_deferredDestroys.Push(c);
		}

		// The contact persists.
		c->Update(td, m_contactListener, threadId);
	}
}

void b2ContactManager::FindNewContacts(uint32 moveBegin, uint32 moveEnd, uint32 threadId)
{
	m_broadPhase.UpdatePairs(moveBegin, moveEnd, this, threadId);
}

void b2ContactManager::AddPair(void* proxyUserDataA, void* proxyUserDataB, uint32 threadId)
{
	b2FixtureProxy* proxyA = (b2FixtureProxy*)proxyUserDataA;
	b2FixtureProxy* proxyB = (b2FixtureProxy*)proxyUserDataB;

	b2Fixture* fixtureA = proxyA->fixture;
	b2Fixture* fixtureB = proxyB->fixture;

	b2Body* bodyA = fixtureA->GetBody();
	b2Body* bodyB = fixtureB->GetBody();

	// Are the fixtures on the same body?
	if (bodyA == bodyB)
	{
		return;
	}

	b2ContactProxyIds proxyIds(proxyA->proxyId, proxyB->proxyId);

	// TODO_ERIN use a hash table to remove a potential bottleneck when both
	// bodies have a lot of contacts.
	// Does a contact already exist?
	b2ContactEdge* edge = bodyB->GetContactList();
	while (edge)
	{
		if (edge->other == bodyA)
		{
			const b2ContactProxyIds& edgeProxyIds = edge->contact->m_proxyIds;

			if (edgeProxyIds == proxyIds)
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

	int32 indexA = proxyA->childIndex;
	int32 indexB = proxyB->childIndex;

	if (m_deferCreates)
	{
		b2DeferredContactCreate deferredCreate;
		deferredCreate.fixtureA = proxyA->fixture;
		deferredCreate.fixtureB = proxyB->fixture;
		deferredCreate.indexA = indexA;
		deferredCreate.indexB = indexB;
		deferredCreate.proxyIds = proxyIds;
		m_perThreadData[threadId].m_deferredCreates.Push(deferredCreate);
	}
	else
	{
		// Call the factory.
		b2Contact* c = b2Contact::Create(fixtureA, indexA, fixtureB, indexB, m_allocator);
		if (c == nullptr)
		{
			return;
		}

		OnContactCreate(c, proxyIds);
	}
}

// This allows proxy synchronization to be somewhat parallel.
void b2ContactManager::GenerateDeferredMoveProxies(b2Body** bodies, uint32 count, uint32 threadId)
{
	b2ContactManagerPerThreadData& td = m_perThreadData[threadId];

	for (uint32 i = 0; i < count; ++i)
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

				b2Shape* shape = f->m_shape;

				// Compute an AABB that covers the swept shape (may miss some rotation effect).
				b2AABB aabb1, aabb2;
				shape->ComputeAABB(&aabb1, xf1, proxy->childIndex);
				shape->ComputeAABB(&aabb2, b->m_xf, proxy->childIndex);

				proxy->aabb.Combine(aabb1, aabb2);

				// A move is required if the new AABB isn't contained by the fat AABB.
				bool requiresMove = m_broadPhase.GetFatAABB(proxy->proxyId).Contains(proxy->aabb) == false;

				if (requiresMove)
				{
					b2DeferredMoveProxy moveProxy;
					moveProxy.aabb = proxy->aabb;
					moveProxy.displacement = b->m_xf.p - xf1.p;
					moveProxy.proxyId = proxy->proxyId;
					td.m_deferredMoveProxies.Push(moveProxy);
				}
			}
		}
	}
}

void b2ContactManager::ConsumeDeferredBeginContacts(uint32 threadCount)
{
	b2Contact* contact = nullptr;
	while (b2PopPerThreadData(m_perThreadData, contact, threadCount,
			&b2ContactManagerPerThreadData::m_deferredBeginContacts, b2ContactPointerLessThan))
	{
		m_contactListener->BeginContact(contact);
	}
}

void b2ContactManager::ConsumeDeferredEndContacts(uint32 threadCount)
{
	b2Contact* contact = nullptr;
	while (b2PopPerThreadData(m_perThreadData, contact, threadCount,
		&b2ContactManagerPerThreadData::m_deferredEndContacts, b2ContactPointerLessThan))
	{
		m_contactListener->EndContact(contact);
	}
}

void b2ContactManager::ConsumeDeferredPreSolves(uint32 threadCount)
{
	b2DeferredPreSolve ps{};
	while (b2PopPerThreadData(m_perThreadData, ps, threadCount,
		&b2ContactManagerPerThreadData::m_deferredPreSolves, b2DeferredPreSolveLessThan))
	{
		m_contactListener->PreSolve(ps.contact, &ps.oldManifold);
	}
}

void b2ContactManager::ConsumeDeferredPostSolves(uint32 threadCount)
{
	b2DeferredPostSolve ps{};
	while (b2PopPerThreadData(m_perThreadData, ps, threadCount,
		&b2ContactManagerPerThreadData::m_deferredPostSolves, b2DeferredPostSolveLessThan))
	{
		m_contactListener->PostSolve(ps.contact, &ps.impulse);
	}
}

void b2ContactManager::ConsumeDeferredAwakes(uint32 threadCount)
{
	// Order doesn't affect determinism so we don't bother sorting.
	for (uint32 i = 0; i < threadCount; ++i)
	{
		while (m_perThreadData[i].m_deferredAwakes.GetCount())
		{
			b2Contact* c = m_perThreadData[i].m_deferredAwakes.Pop();
			c->m_nodeA.other->SetAwake(true);
			c->m_nodeB.other->SetAwake(true);
		}
	}
}

void b2ContactManager::ConsumeDeferredDestroys(uint32 threadCount)
{
	b2Contact* contact = nullptr;
	while (b2PopPerThreadData(m_perThreadData, contact, threadCount,
		&b2ContactManagerPerThreadData::m_deferredDestroys, b2ContactPointerLessThan))
	{
		Destroy(contact);
	}
}

void b2ContactManager::ConsumeDeferredCreates(uint32 threadCount)
{
	b2ContactProxyIds prevIds{};

	// Finish contact creation.
	b2DeferredContactCreate deferredCreate{};
	while (b2PopPerThreadData(m_perThreadData, deferredCreate, threadCount,
		&b2ContactManagerPerThreadData::m_deferredCreates, b2DeferredContactCreateLessThan))
	{
		// Already created?
		if (deferredCreate.proxyIds == prevIds)
		{
			continue;
		}

		prevIds = deferredCreate.proxyIds;

		b2Fixture* fixtureA = deferredCreate.fixtureA;
		b2Fixture* fixtureB = deferredCreate.fixtureB;

		int32 indexA = deferredCreate.indexA;
		int32 indexB = deferredCreate.indexB;

		// Call the factory.
		b2Contact* c = b2Contact::Create(fixtureA, indexA, fixtureB, indexB, m_allocator);
		if (c == nullptr)
		{
			return;
		}

		// Finish creating.
		OnContactCreate(c, prevIds);
	}
}

void b2ContactManager::ConsumeDeferredMoveProxies(uint32 threadCount)
{
	b2DeferredMoveProxy moveProxy{};
	while (b2PopPerThreadData(m_perThreadData,moveProxy, threadCount,
		&b2ContactManagerPerThreadData::m_deferredMoveProxies, b2DeferredMoveProxyLessThan))
	{
		m_broadPhase.MoveProxy(moveProxy.proxyId, moveProxy.aabb, moveProxy.displacement);
	}
}

void b2ContactManager::OnContactCreate(b2Contact* c, b2ContactProxyIds proxyIds)
{
	b2Fixture* fixtureA = c->GetFixtureA();
	b2Fixture* fixtureB = c->GetFixtureB();
	b2Body* bodyA = fixtureA->GetBody();
	b2Body* bodyB = fixtureB->GetBody();

	c->m_proxyIds = proxyIds;

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

	PushContact(c);
}

void b2ContactManager::RecalculateToiCandidacy(b2Body* body)
{
	for (b2ContactEdge* ce = body->GetContactList(); ce; ce = ce->next)
	{
		RecalculateToiCandidacy(ce->contact);
	}
}

void b2ContactManager::RecalculateToiCandidacy(b2Fixture* fixture)
{
	b2Body* body = fixture->GetBody();

	for (b2ContactEdge* ce = body->GetContactList(); ce; ce = ce->next)
	{
		b2Contact* contact = ce->contact;

		if (contact->GetFixtureA() == fixture || contact->GetFixtureB() == fixture)
		{
			RecalculateToiCandidacy(ce->contact);
		}
	}
}

void b2ContactManager::RecalculateToiCandidacy(b2Contact* c)
{
	b2Fixture* fixtureA = c->GetFixtureA();
	b2Fixture* fixtureB = c->GetFixtureB();

	b2Body* bodyA = fixtureA->GetBody();
	b2Body* bodyB = fixtureB->GetBody();

	uint32 flags = c->m_flags;

	if (bodyA->IsToiCandidate() && bodyB->IsToiCandidate() &&
		fixtureA->IsSensor() == false && fixtureB->IsSensor() == false)
	{
		flags |= b2Contact::e_toiCandidateFlag;
	}
	else
	{
		flags &= ~b2Contact::e_toiCandidateFlag;
	}

	if (flags == c->m_flags)
	{
		return;
	}

	c->m_flags = flags;

	if ((c->m_flags & b2Contact::e_toiCandidateFlag) == b2Contact::e_toiCandidateFlag)
	{
		b2Assert(c->m_managerIndex >= m_toiCount);
		m_contacts[m_toiCount]->m_managerIndex = c->m_managerIndex;
		m_contacts[c->m_managerIndex] = m_contacts[m_toiCount];
		m_contacts[m_toiCount] = c;
		c->m_managerIndex = m_toiCount;
		++m_toiCount;
	}
	else
	{
		b2Assert(c->m_managerIndex < m_toiCount);
		--m_toiCount;
		m_contacts[m_toiCount]->m_managerIndex = c->m_managerIndex;
		m_contacts[c->m_managerIndex] = m_contacts[m_toiCount];
		m_contacts[m_toiCount] = c;
		c->m_managerIndex = m_toiCount;
	}
}

void b2ContactManager::PushContact(b2Contact* c)
{
	if (c->m_flags & b2Contact::e_toiCandidateFlag)
	{
		if (m_toiCount < m_contacts.GetCount())
		{
			c->m_managerIndex = m_toiCount;
			b2Assert((m_contacts[m_toiCount]->m_flags & b2Contact::e_toiCandidateFlag) == 0);
			m_contacts[m_toiCount]->m_managerIndex = m_contacts.GetCount();
			m_contacts.Push(m_contacts[m_toiCount]);
			m_contacts[m_toiCount] = c;
			++m_toiCount;
		}
		else
		{
			c->m_managerIndex = m_contacts.GetCount();
			m_contacts.Push(c);
			++m_toiCount;
		}
	}
	else
	{
		c->m_managerIndex = m_contacts.GetCount();
		m_contacts.Push(c);
	}
}

void b2ContactManager::RemoveContact(b2Contact* c)
{
	if (c->m_managerIndex < m_toiCount)
	{
		b2Assert((c->m_flags & b2Contact::e_toiCandidateFlag) == b2Contact::e_toiCandidateFlag);
		--m_toiCount;
		m_contacts[m_toiCount]->m_managerIndex = c->m_managerIndex;
		m_contacts[c->m_managerIndex] = m_contacts[m_toiCount];
		b2Contact* backContact = m_contacts.Pop();
		if (m_contacts.GetCount() > m_toiCount)
		{
			m_contacts[m_toiCount] = backContact;
			m_contacts[m_toiCount]->m_managerIndex = m_toiCount;
		}
	}
	else
	{
		b2Assert((c->m_flags & b2Contact::e_toiCandidateFlag) == 0);
		m_contacts.Peek()->m_managerIndex = c->m_managerIndex;
		m_contacts.RemoveAndSwap(c->m_managerIndex);
	}
}
