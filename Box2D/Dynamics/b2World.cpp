/*
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

#include "Box2D/Dynamics/b2World.h"
#include "Box2D/Dynamics/b2Body.h"
#include "Box2D/Dynamics/b2Fixture.h"
#include "Box2D/Dynamics/b2Island.h"
#include "Box2D/Dynamics/Joints/b2PulleyJoint.h"
#include "Box2D/Dynamics/Contacts/b2Contact.h"
#include "Box2D/Dynamics/Contacts/b2ContactSolver.h"
#include "Box2D/Collision/b2Collision.h"
#include "Box2D/Collision/b2BroadPhase.h"
#include "Box2D/Collision/Shapes/b2CircleShape.h"
#include "Box2D/Collision/Shapes/b2EdgeShape.h"
#include "Box2D/Collision/Shapes/b2ChainShape.h"
#include "Box2D/Collision/Shapes/b2PolygonShape.h"
#include "Box2D/Collision/b2TimeOfImpact.h"
#include "Box2D/Common/b2Draw.h"
#include "Box2D/Common/b2Timer.h"
#include "Box2D/MT/b2TaskExecutor.h"
#include <new>

b2ThreadContext b2MainThreadCtx(b2StackAllocator& stackAllocator)
{
	b2ThreadContext threadCtx;
	threadCtx.stack = &stackAllocator;
	threadCtx.threadId = 0;
	return threadCtx;
}

class b2SolveTask : public b2Task
{
public:
	b2SolveTask(b2ContactManagerPerThreadData* td, b2ContactListener* listener, const b2TimeStep& timestep,
		b2Vec2 gravity, bool allowSleep, b2SolveTask* next)
	{
		m_td = td;
		m_contactListener = listener;
		m_timestep = &timestep;
		m_gravity = gravity;
		m_allowSleep = allowSleep;
		m_next = next;
		m_islandCount = 0;
		m_bodyCount = 0;
		m_contactCount = 0;
		m_jointCount = 0;
	}

	void AddIsland(int32 bodyCount, int32 contactCount, int32 jointCount,
		b2Body** bodies, b2Contact** contacts, b2Joint** joints,
		b2Velocity* velocities, b2Position* positions)
	{
		m_islands[m_islandCount++] = b2Island(
			bodyCount, contactCount, jointCount,
			bodies, contacts, joints,
			velocities, positions);

		m_bodyCount += bodyCount;
		m_contactCount += contactCount;
		m_jointCount += jointCount;

		SetCost(GetCost() + b2GetIslandCost(m_bodyCount, m_contactCount, m_jointCount));
	}

	int32 GetBodyCount() const { return m_bodyCount; }

	b2SolveTask* GetNext() { return m_next; }

private:

	virtual void Execute(const b2ThreadContext& threadCtx) override
	{
		b2TimeStep timestep = *m_timestep;
		for (int32 islandIndex = 0; islandIndex < m_islandCount; ++islandIndex)
		{
			b2Island& island = m_islands[islandIndex];

			island.m_td = m_td + threadCtx.threadId;

			island.Solve(&island.m_td->m_profile, timestep, m_gravity, threadCtx.stack, m_contactListener,
				threadCtx.threadId, m_allowSleep);
		}
	}

	b2Island m_islands[b2_solveBatchTargetBodyCount];
	const b2TimeStep* m_timestep;
	b2ContactManagerPerThreadData* m_td;
	b2ContactListener* m_contactListener;
	b2SolveTask* m_next;
	b2Vec2 m_gravity;
	int32 m_islandCount;
	int32 m_bodyCount;
	int32 m_contactCount;
	int32 m_jointCount;
	bool m_allowSleep;
};

class b2CollideTask : public b2RangeTask
{
public:
	b2CollideTask() {}
	b2CollideTask(const b2RangeTaskRange& range, b2ContactManager* manager)
		: b2RangeTask(range)
		, m_contactManager(manager)
	{}
private:
	virtual void Execute(const b2ThreadContext& threadCtx, const b2RangeTaskRange& range) override
	{
		m_contactManager->Collide(range.begin, range.end, threadCtx.threadId);
	}
	b2ContactManager* m_contactManager;
};

class b2GenerateDeferredMoveProxiesTask : public b2RangeTask
{
public:
	b2GenerateDeferredMoveProxiesTask() {}
	b2GenerateDeferredMoveProxiesTask(const b2RangeTaskRange& range, b2ContactManager* manager, b2Body** bodies)
		: b2RangeTask(range)
		, m_contactManager(manager)
		, m_bodies(bodies)
	{}
private:
	virtual void Execute(const b2ThreadContext& threadCtx, const b2RangeTaskRange& range) override
	{
		m_contactManager->GenerateDeferredMoveProxies(m_bodies + range.begin, range.end - range.begin, threadCtx.threadId);
	}
	b2ContactManager* m_contactManager;
	b2Body** m_bodies;
};

class b2BroadphaseFindNewContactsTask : public b2RangeTask
{
public:
	b2BroadphaseFindNewContactsTask() {}
	b2BroadphaseFindNewContactsTask(const b2RangeTaskRange& range, b2ContactManager* manager)
		: b2RangeTask(range)
		, m_contactManager(manager)
	{}
private:
	virtual void Execute(const b2ThreadContext& threadCtx, const b2RangeTaskRange& range) override
	{
		m_contactManager->FindNewContacts(range.begin, range.end, threadCtx.threadId);
	}
	b2ContactManager* m_contactManager;
};

class b2ContactPreSolveTask : public b2RangeTask
{
public:
	b2ContactPreSolveTask() {}
	b2ContactPreSolveTask(const b2RangeTaskRange& range, b2Contact** contacts, bool toiCandidates)
		: b2RangeTask(range)
		, m_contacts(contacts)
		, m_toiCandidates(toiCandidates)
	{}
private:
	virtual void Execute(const b2ThreadContext&, const b2RangeTaskRange& range) override
	{
		if (m_toiCandidates)
		{
			for (uint32 i = range.begin; i < range.end; ++i)
			{
				b2Contact* c = m_contacts[i];
				c->m_flags &= ~(b2Contact::e_toiFlag | b2Contact::e_islandFlag);
				c->m_toiCount = 0;
				c->m_toi = 1.0f;
			}
		}
		else
		{
			for (uint32 i = range.begin; i < range.end; ++i)
			{
				b2Contact* c = m_contacts[i];
				c->m_flags &= ~b2Contact::e_islandFlag;
			}
		}
	}
	b2Contact** m_contacts;
	bool m_toiCandidates;
};

class b2BodyPreSolveTask : public b2RangeTask
{
public:
	b2BodyPreSolveTask() {}
	b2BodyPreSolveTask(const b2RangeTaskRange& range, b2Body** bodies)
		: b2RangeTask(range)
		, m_bodies(bodies)
	{}
private:
	virtual void Execute(const b2ThreadContext&, const b2RangeTaskRange& range) override
	{
		for (uint32 i = range.begin; i < range.end; ++i)
		{
			m_bodies[i]->m_flags &= ~b2Body::e_islandFlag;
			m_bodies[i]->m_sweep.alpha0 = 0.0f;
		}
	}
	b2Body** m_bodies;
};

class b2SortMovesTask : public b2Task
{
public:
	b2SortMovesTask() {}
	b2SortMovesTask(b2ContactManagerPerThreadData& td)
		: m_tdIn(&td)
	{}
private:
	virtual void Execute(const b2ThreadContext&) override
	{
		std::sort(m_tdIn->m_deferredMoveProxies.Data(), m_tdIn->m_deferredMoveProxies.End(), b2DeferredMoveProxyLessThan);
	}
	b2ContactManagerPerThreadData* m_tdIn;
};

class b2SortPostSolvesTask : public b2Task
{
public:
	b2SortPostSolvesTask() {}
	b2SortPostSolvesTask(b2ContactManagerPerThreadData& td)
		: m_tdIn(&td)
	{}
private:
	virtual void Execute(const b2ThreadContext&) override
	{
		std::sort(m_tdIn->m_deferredPostSolves.Data(), m_tdIn->m_deferredPostSolves.End(), b2DeferredPostSolveLessThan);
	}
	b2ContactManagerPerThreadData* m_tdIn;
};

class b2SortCollidesTask : public b2Task
{
public:
	b2SortCollidesTask() {}
	b2SortCollidesTask(b2ContactManagerPerThreadData& td)
		: m_tdIn(&td)
	{}
private:
	virtual void Execute(const b2ThreadContext&) override
	{
		std::sort(m_tdIn->m_deferredBeginContacts.Data(), m_tdIn->m_deferredBeginContacts.End(), b2ContactPointerLessThan);
		std::sort(m_tdIn->m_deferredEndContacts.Data(), m_tdIn->m_deferredEndContacts.End(), b2ContactPointerLessThan);
		std::sort(m_tdIn->m_deferredDestroys.Data(), m_tdIn->m_deferredDestroys.End(), b2ContactPointerLessThan);
		std::sort(m_tdIn->m_deferredPreSolves.Data(), m_tdIn->m_deferredPreSolves.End(), b2DeferredPreSolveLessThan);
	}
	b2ContactManagerPerThreadData* m_tdIn;
};

class b2SortCreatesTask : public b2Task
{
public:
	b2SortCreatesTask() {}
	b2SortCreatesTask(b2ContactManagerPerThreadData& td)
		: m_tdIn(&td)
	{}
private:
	virtual void Execute(const b2ThreadContext&) override
	{
		std::sort(m_tdIn->m_deferredCreates.Data(), m_tdIn->m_deferredCreates.End(), b2DeferredContactCreateLessThan);
	}
	b2ContactManagerPerThreadData* m_tdIn;
};

template<typename TaskType>
inline void b2SubmitTasks(b2TaskExecutor& executor, b2TaskGroup taskGroup, TaskType* tasks, uint32 count)
{
	b2Task* taskPtrs[b2_partitionRangeMaxOutput];
	for (uint32 i = 0; i < count; ++i)
	{
		taskPtrs[i] = tasks + i;
		taskPtrs[i]->SetTaskGroup(taskGroup);
	}
	executor.SubmitTasks(taskGroup, taskPtrs, count);
}

inline void b2SubmitTask(b2TaskExecutor& executor, b2TaskGroup taskGroup, b2Task* task)
{
	task->SetTaskGroup(taskGroup);
	executor.SubmitTask(taskGroup, task);
}

b2World::b2World(const b2Vec2& gravity)
{
	m_destructionListener = nullptr;
	m_debugDraw = nullptr;

	m_bodyList = nullptr;
	m_jointList = nullptr;

	m_bodyCount = 0;
	m_jointCount = 0;

	m_warmStarting = true;
	m_continuousPhysics = true;
	m_subStepping = false;

	m_stepComplete = true;

	m_allowSleep = true;
	m_gravity = gravity;

	m_flags = e_clearForces;

	m_inv_dt0 = 0.0f;

	m_contactManager.m_allocator = &m_blockAllocator;

	memset(&m_profile, 0, sizeof(b2Profile));
}

b2World::~b2World()
{
	// Some shapes allocate using b2Alloc.
	b2Body* b = m_bodyList;
	while (b)
	{
		b2Body* bNext = b->m_next;

		b2Fixture* f = b->m_fixtureList;
		while (f)
		{
			b2Fixture* fNext = f->m_next;
			f->m_proxyCount = 0;
			f->Destroy(&m_blockAllocator);
			f = fNext;
		}

		b = bNext;
	}
}

void b2World::SetDestructionListener(b2DestructionListener* listener)
{
	m_destructionListener = listener;
}

void b2World::SetContactFilter(b2ContactFilter* filter)
{
	m_contactManager.m_contactFilter = filter;
}

void b2World::SetContactListener(b2ContactListener* listener)
{
	m_contactManager.m_contactListener = listener;
}

void b2World::SetDebugDraw(b2Draw* debugDraw)
{
	m_debugDraw = debugDraw;
}

b2Body* b2World::CreateBody(const b2BodyDef* def)
{
	b2Assert(IsLocked() == false);
	if (IsLocked())
	{
		return nullptr;
	}

	void* mem = m_blockAllocator.Allocate(sizeof(b2Body));
	b2Body* b = new (mem) b2Body(def, this);

	// Add to world doubly linked list.
	b->m_prev = nullptr;
	b->m_next = m_bodyList;
	if (m_bodyList)
	{
		m_bodyList->m_prev = b;
	}
	m_bodyList = b;
	++m_bodyCount;

	// Add to bodies array.
	if (def->type != b2_staticBody)
	{
		b->m_worldIndex = m_nonStaticBodies.GetCount();
		m_nonStaticBodies.Push(b);
	}
	else
	{
		b->m_worldIndex = m_staticBodies.GetCount();
		m_staticBodies.Push(b);
	}

	return b;
}

void b2World::DestroyBody(b2Body* b)
{
	b2Assert(m_bodyCount > 0);
	b2Assert(IsLocked() == false);
	if (IsLocked())
	{
		return;
	}

	// Delete the attached joints.
	b2JointEdge* je = b->m_jointList;
	while (je)
	{
		b2JointEdge* je0 = je;
		je = je->next;

		if (m_destructionListener)
		{
			m_destructionListener->SayGoodbye(je0->joint);
		}

		DestroyJoint(je0->joint);

		b->m_jointList = je;
	}
	b->m_jointList = nullptr;

	// Delete the attached contacts.
	b2ContactEdge* ce = b->m_contactList;
	while (ce)
	{
		b2ContactEdge* ce0 = ce;
		ce = ce->next;
		m_contactManager.Destroy(ce0->contact);
	}
	b->m_contactList = nullptr;

	// Delete the attached fixtures. This destroys broad-phase proxies.
	b2Fixture* f = b->m_fixtureList;
	while (f)
	{
		b2Fixture* f0 = f;
		f = f->m_next;

		if (m_destructionListener)
		{
			m_destructionListener->SayGoodbye(f0);
		}

		f0->DestroyProxies(&m_contactManager.m_broadPhase);
		f0->Destroy(&m_blockAllocator);
		f0->~b2Fixture();
		m_blockAllocator.Free(f0, sizeof(b2Fixture));

		b->m_fixtureList = f;
		b->m_fixtureCount -= 1;
	}
	b->m_fixtureList = nullptr;
	b->m_fixtureCount = 0;

	// Remove world body list.
	if (b->m_prev)
	{
		b->m_prev->m_next = b->m_next;
	}

	if (b->m_next)
	{
		b->m_next->m_prev = b->m_prev;
	}

	if (b == m_bodyList)
	{
		m_bodyList = b->m_next;
	}

	// Remove from bodies array.
	int32 index = b->m_worldIndex;
	if (b->m_type != b2_staticBody)
	{
		m_nonStaticBodies.Peek()->m_worldIndex = index;
		m_nonStaticBodies.RemoveAndSwap(index);
	}
	else
	{
		m_staticBodies.Peek()->m_worldIndex = index;
		m_staticBodies.RemoveAndSwap(index);
	}

	--m_bodyCount;
	b->~b2Body();
	m_blockAllocator.Free(b, sizeof(b2Body));
}

b2Joint* b2World::CreateJoint(const b2JointDef* def)
{
	b2Assert(IsLocked() == false);
	if (IsLocked())
	{
		return nullptr;
	}

	b2Joint* j = b2Joint::Create(def, &m_blockAllocator);

	// Connect to the world list.
	j->m_prev = nullptr;
	j->m_next = m_jointList;
	if (m_jointList)
	{
		m_jointList->m_prev = j;
	}
	m_jointList = j;
	++m_jointCount;

	// Connect to the bodies' doubly linked lists.
	j->m_edgeA.joint = j;
	j->m_edgeA.other = j->m_bodyB;
	j->m_edgeA.prev = nullptr;
	j->m_edgeA.next = j->m_bodyA->m_jointList;
	if (j->m_bodyA->m_jointList) j->m_bodyA->m_jointList->prev = &j->m_edgeA;
	j->m_bodyA->m_jointList = &j->m_edgeA;

	j->m_edgeB.joint = j;
	j->m_edgeB.other = j->m_bodyA;
	j->m_edgeB.prev = nullptr;
	j->m_edgeB.next = j->m_bodyB->m_jointList;
	if (j->m_bodyB->m_jointList) j->m_bodyB->m_jointList->prev = &j->m_edgeB;
	j->m_bodyB->m_jointList = &j->m_edgeB;

	b2Body* bodyA = def->bodyA;
	b2Body* bodyB = def->bodyB;

	// If the joint prevents collisions, then flag any contacts for filtering.
	if (def->collideConnected == false)
	{
		b2ContactEdge* edge = bodyB->GetContactList();
		while (edge)
		{
			if (edge->other == bodyA)
			{
				// Flag the contact for filtering at the next time step (where either
				// body is awake).
				edge->contact->FlagForFiltering();
			}

			edge = edge->next;
		}
	}

	// Note: creating a joint doesn't wake the bodies.

	return j;
}

void b2World::DestroyJoint(b2Joint* j)
{
	b2Assert(IsLocked() == false);
	if (IsLocked())
	{
		return;
	}

	bool collideConnected = j->m_collideConnected;

	// Remove from the doubly linked list.
	if (j->m_prev)
	{
		j->m_prev->m_next = j->m_next;
	}

	if (j->m_next)
	{
		j->m_next->m_prev = j->m_prev;
	}

	if (j == m_jointList)
	{
		m_jointList = j->m_next;
	}

	// Disconnect from island graph.
	b2Body* bodyA = j->m_bodyA;
	b2Body* bodyB = j->m_bodyB;

	// Wake up connected bodies.
	bodyA->SetAwake(true);
	bodyB->SetAwake(true);

	// Remove from body 1.
	if (j->m_edgeA.prev)
	{
		j->m_edgeA.prev->next = j->m_edgeA.next;
	}

	if (j->m_edgeA.next)
	{
		j->m_edgeA.next->prev = j->m_edgeA.prev;
	}

	if (&j->m_edgeA == bodyA->m_jointList)
	{
		bodyA->m_jointList = j->m_edgeA.next;
	}

	j->m_edgeA.prev = nullptr;
	j->m_edgeA.next = nullptr;

	// Remove from body 2
	if (j->m_edgeB.prev)
	{
		j->m_edgeB.prev->next = j->m_edgeB.next;
	}

	if (j->m_edgeB.next)
	{
		j->m_edgeB.next->prev = j->m_edgeB.prev;
	}

	if (&j->m_edgeB == bodyB->m_jointList)
	{
		bodyB->m_jointList = j->m_edgeB.next;
	}

	j->m_edgeB.prev = nullptr;
	j->m_edgeB.next = nullptr;

	b2Joint::Destroy(j, &m_blockAllocator);

	b2Assert(m_jointCount > 0);
	--m_jointCount;

	// If the joint prevents collisions, then flag any contacts for filtering.
	if (collideConnected == false)
	{
		b2ContactEdge* edge = bodyB->GetContactList();
		while (edge)
		{
			if (edge->other == bodyA)
			{
				// Flag the contact for filtering at the next time step (where either
				// body is awake).
				edge->contact->FlagForFiltering();
			}

			edge = edge->next;
		}
	}
}

//
void b2World::SetAllowSleeping(bool flag)
{
	if (flag == m_allowSleep)
	{
		return;
	}

	m_allowSleep = flag;
	if (m_allowSleep == false)
	{
		for (b2Body* b = m_bodyList; b; b = b->m_next)
		{
			b->SetAwake(true);
		}
	}
}

void b2World::SolveTOI(b2TaskExecutor& executor, b2TaskGroup taskGroup, const b2TimeStep& step)
{
	if (m_stepComplete)
	{
		SolveTOIInit(executor, taskGroup);
	}

	int32 bodyCapacity = 2 * b2_maxTOIContacts;
	int32 contactCapacity = b2_maxTOIContacts;

	b2Body** bodies = (b2Body**)m_stackAllocator.Allocate(bodyCapacity * sizeof(b2Body*));
	b2Contact** contacts = (b2Contact**)m_stackAllocator.Allocate(contactCapacity * sizeof(b2Contact*));

	b2Velocity* velocities = (b2Velocity*)m_stackAllocator.Allocate(bodyCapacity * sizeof(b2Velocity));
	b2Position* positions = (b2Position*)m_stackAllocator.Allocate(bodyCapacity * sizeof(b2Position));

	b2Island island(bodies, contacts, velocities, positions);

	// Find TOI events and solve them.
	for (;;)
	{
		// Find the first TOI.
		b2Contact* minContact = nullptr;
		float32 minAlpha = 1.0f;

		for (int32 i = 0; i < m_contactManager.m_toiCount; ++i)
		{
			b2Contact* c = m_contactManager.m_contacts[i];

			// Is this contact disabled?
			if (c->IsEnabled() == false)
			{
				continue;
			}

			// Prevent excessive sub-stepping.
			if (c->m_toiCount > b2_maxSubSteps)
			{
				continue;
			}

			float32 alpha = 1.0f;
			if (c->m_flags & b2Contact::e_toiFlag)
			{
				// This contact has a valid cached TOI.
				alpha = c->m_toi;
			}
			else
			{
				b2Fixture* fA = c->GetFixtureA();
				b2Fixture* fB = c->GetFixtureB();

				// Is there a sensor?
				if (fA->IsSensor() || fB->IsSensor())
				{
					continue;
				}

				b2Body* bA = fA->GetBody();
				b2Body* bB = fB->GetBody();

				b2BodyType typeA = bA->m_type;
				b2BodyType typeB = bB->m_type;
				b2Assert(typeA == b2_dynamicBody || typeB == b2_dynamicBody);

				bool activeA = bA->IsAwake() && typeA != b2_staticBody;
				bool activeB = bB->IsAwake() && typeB != b2_staticBody;

				// Is at least one body active (awake and dynamic or kinematic)?
				if (activeA == false && activeB == false)
				{
					continue;
				}

				// These conditions determine whether the contact is in the TOI candidate list
				// and should be true for all contacts in the list.
				{
					bool collideA = bA->IsBullet() || (typeA != b2_dynamicBody && !bA->GetPreferNoCCD());
					bool collideB = bB->IsBullet() || (typeB != b2_dynamicBody && !bB->GetPreferNoCCD());

					bool isToiCandidate = collideA || collideB;

					b2Assert(isToiCandidate);
					B2_NOT_USED(isToiCandidate);
				}

				// Compute the TOI for this contact.
				// Put the sweeps onto the same time interval.
				float32 alpha0 = bA->m_sweep.alpha0;

				if (bA->m_sweep.alpha0 < bB->m_sweep.alpha0)
				{
					alpha0 = bB->m_sweep.alpha0;
					bA->m_sweep.Advance(alpha0);
				}
				else if (bB->m_sweep.alpha0 < bA->m_sweep.alpha0)
				{
					alpha0 = bA->m_sweep.alpha0;
					bB->m_sweep.Advance(alpha0);
				}

				b2Assert(alpha0 < 1.0f);

				int32 indexA = c->GetChildIndexA();
				int32 indexB = c->GetChildIndexB();

				// Compute the time of impact in interval [0, minTOI]
				b2TOIInput input;
				input.proxyA.Set(fA->GetShape(), indexA);
				input.proxyB.Set(fB->GetShape(), indexB);
				input.sweepA = bA->m_sweep;
				input.sweepB = bB->m_sweep;
				input.tMax = 1.0f;

				b2TOIOutput output;
				b2TimeOfImpact(&output, &input);

				// Beta is the fraction of the remaining portion of the .
				float32 beta = output.t;
				if (output.state == b2TOIOutput::e_touching)
				{
					alpha = b2Min(alpha0 + (1.0f - alpha0) * beta, 1.0f);
				}
				else
				{
					alpha = 1.0f;
				}

				c->m_toi = alpha;
				c->m_flags |= b2Contact::e_toiFlag;
			}

			if (alpha < minAlpha)
			{
				// This is the minimum TOI found so far.
				minContact = c;
				minAlpha = alpha;
			}
		}

		if (minContact == nullptr || 1.0f - 10.0f * b2_epsilon < minAlpha)
		{
			// No more TOI events. Done!
			m_stepComplete = true;
			break;
		}

		// Advance the bodies to the TOI.
		b2Fixture* fA = minContact->GetFixtureA();
		b2Fixture* fB = minContact->GetFixtureB();
		b2Body* bA = fA->GetBody();
		b2Body* bB = fB->GetBody();

		b2Sweep backup1 = bA->m_sweep;
		b2Sweep backup2 = bB->m_sweep;

		bA->Advance(minAlpha);
		bB->Advance(minAlpha);

		// The TOI contact likely has some new contact points.
		minContact->Update(m_contactManager.m_contactListener);
		minContact->m_flags &= ~b2Contact::e_toiFlag;
		++minContact->m_toiCount;

		// Is the contact solid?
		if (minContact->IsEnabled() == false || minContact->IsTouching() == false)
		{
			// Restore the sweeps.
			minContact->SetEnabled(false);
			bA->m_sweep = backup1;
			bB->m_sweep = backup2;
			bA->SynchronizeTransform();
			bB->SynchronizeTransform();
			continue;
		}

		bA->SetAwake(true);
		bB->SetAwake(true);

		// Build the island
		island.Clear();
		island.Add(bA);
		island.Add(bB);
		island.Add(minContact);

		bA->m_flags |= b2Body::e_islandFlag;
		bB->m_flags |= b2Body::e_islandFlag;
		minContact->m_flags |= b2Contact::e_islandFlag;

		// Get contacts on bodyA and bodyB.
		b2Body* bodies[2] = {bA, bB};
		for (int32 i = 0; i < 2; ++i)
		{
			b2Body* body = bodies[i];
			if (body->m_type == b2_dynamicBody)
			{
				for (b2ContactEdge* ce = body->m_contactList; ce; ce = ce->next)
				{
					if (island.m_bodyCount == bodyCapacity)
					{
						break;
					}

					if (island.m_contactCount == contactCapacity)
					{
						break;
					}

					b2Contact* contact = ce->contact;

					// Has this contact already been added to the island?
					if (contact->m_flags & b2Contact::e_islandFlag)
					{
						continue;
					}

					// Only add static, kinematic, or bullet bodies.
					b2Body* other = ce->other;
					if (other->m_type == b2_dynamicBody &&
						body->IsBullet() == false && other->IsBullet() == false)
					{
						continue;
					}

					// Skip sensors.
					bool sensorA = contact->m_fixtureA->m_isSensor;
					bool sensorB = contact->m_fixtureB->m_isSensor;
					if (sensorA || sensorB)
					{
						continue;
					}

					// Tentatively advance the body to the TOI.
					b2Sweep backup = other->m_sweep;
					if ((other->m_flags & b2Body::e_islandFlag) == 0)
					{
						other->Advance(minAlpha);
					}

					// Update the contact points
					contact->Update(m_contactManager.m_contactListener);

					// Was the contact disabled by the user?
					if (contact->IsEnabled() == false)
					{
						other->m_sweep = backup;
						other->SynchronizeTransform();
						continue;
					}

					// Are there contact points?
					if (contact->IsTouching() == false)
					{
						other->m_sweep = backup;
						other->SynchronizeTransform();
						continue;
					}

					// Add the contact to the island
					contact->m_flags |= b2Contact::e_islandFlag;
					island.Add(contact);

					// Has the other body already been added to the island?
					if (other->m_flags & b2Body::e_islandFlag)
					{
						continue;
					}

					// Add the other body to the island.
					other->m_flags |= b2Body::e_islandFlag;

					if (other->m_type != b2_staticBody)
					{
						other->SetAwake(true);
					}

					island.Add(other);
				}
			}
		}

		b2TimeStep subStep;
		subStep.dt = (1.0f - minAlpha) * step.dt;
		subStep.inv_dt = 1.0f / subStep.dt;
		subStep.dtRatio = 1.0f;
		subStep.positionIterations = 20;
		subStep.velocityIterations = step.velocityIterations;
		subStep.warmStarting = false;
		island.SolveTOI(subStep, bA->GetIslandIndex(0), bB->GetIslandIndex(0), &m_stackAllocator, m_contactManager.m_contactListener);

		// Reset island flags and synchronize broad-phase proxies.
		for (int32 i = 0; i < island.m_bodyCount; ++i)
		{
			b2Body* body = island.m_bodies[i];
			body->m_flags &= ~b2Body::e_islandFlag;

			if (body->m_type != b2_dynamicBody)
			{
				continue;
			}

			body->SynchronizeFixtures();

			// Invalidate all contact TOIs on this displaced body.
			for (b2ContactEdge* ce = body->m_contactList; ce; ce = ce->next)
			{
				ce->contact->m_flags &= ~(b2Contact::e_toiFlag | b2Contact::e_islandFlag);
			}
		}

		// Commit fixture proxy movements to the broad-phase so that new contacts are created.
		// Also, some contacts can be destroyed.
		m_contactManager.FindNewContacts(0, m_contactManager.m_broadPhase.GetMoveCount(), 0);
		m_contactManager.m_broadPhase.ResetBuffers();

		if (m_subStepping)
		{
			m_stepComplete = false;
			break;
		}
	}

	m_stackAllocator.Free(positions);
	m_stackAllocator.Free(velocities);
	m_stackAllocator.Free(contacts);
	m_stackAllocator.Free(bodies);
}

void b2World::FindNewContacts(b2TaskExecutor& executor, b2TaskGroup taskGroup, uint32 threadCount)
{
	if (m_contactManager.m_broadPhase.GetMoveCount() == 0)
	{
		return;
	}

	b2BroadphaseFindNewContactsTask tasks[b2_partitionRangeMaxOutput];
	b2PartitionedRange ranges;
	executor.PartitionRange(0, m_contactManager.m_broadPhase.GetMoveCount(), ranges);
	for (uint32 i = 0; i < ranges.count; ++i)
	{
		tasks[i] = b2BroadphaseFindNewContactsTask(ranges[i], &m_contactManager);
	}
	m_contactManager.m_deferCreates = true;
	b2SubmitTasks(executor, taskGroup, tasks, ranges.count);

	executor.Wait(taskGroup, b2MainThreadCtx(m_stackAllocator));
	m_contactManager.m_deferCreates = false;

	b2SortCreatesTask sortTasks[b2_maxThreads];
	for (uint32 i = 0; i < threadCount; ++i)
	{
		sortTasks[i] = b2SortCreatesTask(m_contactManager.m_perThreadData[i]);
	}
	b2SubmitTasks(executor, taskGroup, sortTasks, threadCount);

	executor.Wait(taskGroup, b2MainThreadCtx(m_stackAllocator));

	m_contactManager.m_broadPhase.ResetBuffers();
	m_contactManager.ConsumeDeferredCreates(threadCount);
}

void b2World::Collide(b2TaskExecutor& executor, b2TaskGroup taskGroup, uint32 threadCount)
{
	if (m_contactManager.m_contacts.GetCount() == 0)
	{
		return;
	}

	b2CollideTask tasks[b2_partitionRangeMaxOutput];
	b2PartitionedRange ranges;
	executor.PartitionRange(0, m_contactManager.m_contacts.GetCount(), ranges);
	for (uint32 i = 0; i < ranges.count; ++i)
	{
		tasks[i] = b2CollideTask(ranges[i], &m_contactManager);
	}
	b2SubmitTasks(executor, taskGroup, tasks, ranges.count);

	executor.Wait(taskGroup, b2MainThreadCtx(m_stackAllocator));

	b2SortCollidesTask sortTasks[b2_maxThreads];
	for (uint32 i = 0; i < threadCount; ++i)
	{
		sortTasks[i] = b2SortCollidesTask(m_contactManager.m_perThreadData[i]);
	}
	b2SubmitTasks(executor, taskGroup, sortTasks, threadCount);

	executor.Wait(taskGroup, b2MainThreadCtx(m_stackAllocator));

	m_contactManager.ConsumeDeferredAwakes(threadCount);
	m_contactManager.ConsumeDeferredBeginContacts(threadCount);
	m_contactManager.ConsumeDeferredEndContacts(threadCount);
	m_contactManager.ConsumeDeferredPreSolves(threadCount);
	m_contactManager.ConsumeDeferredDestroys(threadCount);
}

void b2World::SynchronizeFixtures(b2TaskExecutor& executor, b2TaskGroup taskGroup, uint32 threadCount)
{
	if (m_nonStaticBodies.GetCount() == 0)
	{
		return;
	}

	b2GenerateDeferredMoveProxiesTask moveTasks[b2_partitionRangeMaxOutput];
	b2PartitionedRange ranges;
	executor.PartitionRange(0, m_nonStaticBodies.GetCount(), ranges);
	for (uint32 i = 0; i < ranges.count; ++i)
	{
		moveTasks[i] = b2GenerateDeferredMoveProxiesTask(ranges[i], &m_contactManager, m_nonStaticBodies.Data());
	}
	b2SubmitTasks(executor, taskGroup, moveTasks, ranges.count);

	executor.Wait(taskGroup, b2MainThreadCtx(m_stackAllocator));

	b2SortMovesTask sortTasks[b2_maxThreads];
	for (uint32 i = 0; i < threadCount; ++i)
	{
		sortTasks[i] = b2SortMovesTask(m_contactManager.m_perThreadData[i]);
	}
	b2SubmitTasks(executor, taskGroup, sortTasks, threadCount);

	executor.Wait(taskGroup, b2MainThreadCtx(m_stackAllocator));

	m_contactManager.ConsumeDeferredMoveProxies(threadCount);
}

void b2World::Solve(b2TaskExecutor& executor, b2TaskGroup taskGroup, const b2TimeStep& step, uint32 threadCount)
{
	// A single static body can be included in multiple islands.
	// In the worst case every non static body is in its own island with every static body.
	int32 maxStaticBodySolveCount = m_nonStaticBodies.GetCount() * m_staticBodies.GetCount();

	// A static body can only be brought into an island by a contact or joint connecting
	// it to a non-static body.
	maxStaticBodySolveCount = b2Min(maxStaticBodySolveCount, m_contactManager.m_contacts.GetCount() + m_jointCount);

	int32 allBodiesCapacity = m_nonStaticBodies.GetCount() + maxStaticBodySolveCount;
	int32 allContactsCapacity = m_contactManager.m_contacts.GetCount();
	int32 allJointsCapacity = m_jointCount;

	b2Body** allBodies = (b2Body**)m_stackAllocator.Allocate(allBodiesCapacity * sizeof(b2Body*));
	b2Contact** allContacts = (b2Contact**)m_stackAllocator.Allocate(allContactsCapacity * sizeof(b2Contact*));
	b2Joint** allJoints = (b2Joint**)m_stackAllocator.Allocate(allJointsCapacity * sizeof(b2Joint*));
	b2Velocity* allVelocities = (b2Velocity*)m_stackAllocator.Allocate(allBodiesCapacity * sizeof(b2Velocity));
	b2Position* allPositions = (b2Position*)m_stackAllocator.Allocate(allBodiesCapacity * sizeof(b2Position));
	int32 allBodiesCount = 0;
	int32 allContactsCount = 0;
	int32 allJointsCount = 0;

	b2Body** bodies = allBodies;
	b2Contact** contacts = allContacts;
	b2Joint** joints = allJoints;
	b2Velocity* velocities = allVelocities;
	b2Position* positions = allPositions;
	int32 bodyCount = 0;
	int32 contactCount = 0;
	int32 jointCount = 0;

	// Clear all the island flags.
	SolveInit(executor, taskGroup);

	b2Timer traversalTimer;

	// Build and simulate all awake islands.
	int32 stackSize = m_bodyCount;
	b2Body** stack = (b2Body**)m_stackAllocator.Allocate(stackSize * sizeof(b2Body*));
	b2SolveTask* solveTaskList = nullptr;
    b2SolveTask* currSolveTask = nullptr;
	for (int32 i = 0; i < m_nonStaticBodies.GetCount(); ++i)
	{
		b2Body* seed = m_nonStaticBodies[i];

		b2Assert(seed->GetType() != b2_staticBody);

		if (seed->m_flags & b2Body::e_islandFlag)
		{
			continue;
		}

		if (seed->IsAwake() == false || seed->IsActive() == false)
		{
			continue;
		}

		// Reset stack.
		int32 stackCount = 0;
		stack[stackCount++] = seed;
		seed->m_flags |= b2Body::e_islandFlag;

		// Perform a depth first search (DFS) on the constraint graph.
		while (stackCount > 0)
		{
			// Grab the next body off the stack and add it to the island.
			b2Body* b = stack[--stackCount];
			b2Assert(b->IsActive() == true);
			bodies[bodyCount++] = b;

			// To keep islands as small as possible, we don't
			// propagate islands across static bodies.
			if (b->GetType() == b2_staticBody)
			{
				continue;
			}

			// Search all contacts connected to this body.
			for (b2ContactEdge* ce = b->m_contactList; ce; ce = ce->next)
			{
				b2Contact* contact = ce->contact;

				// Has this contact already been added to an island?
				if (contact->m_flags & b2Contact::e_islandFlag)
				{
					continue;
				}

				// Is this contact solid and touching?
				if (contact->IsEnabled() == false ||
					contact->IsTouching() == false)
				{
					continue;
				}

				// Skip sensors.
				bool sensorA = contact->m_fixtureA->m_isSensor;
				bool sensorB = contact->m_fixtureB->m_isSensor;
				if (sensorA || sensorB)
				{
					continue;
				}

				contacts[contactCount++] = contact;
				contact->m_flags |= b2Contact::e_islandFlag;

				b2Body* other = ce->other;

				// Was the other body already added to this island?
				if (other->m_flags & b2Body::e_islandFlag)
				{
					continue;
				}

				b2Assert(stackCount < stackSize);
				stack[stackCount++] = other;
				other->m_flags |= b2Body::e_islandFlag;
			}

			// Search all joints connected to this body.
			for (b2JointEdge* je = b->m_jointList; je; je = je->next)
			{
				if (je->joint->m_islandFlag == true)
				{
					continue;
				}

				b2Body* other = je->other;

				// Don't simulate joints connected to inactive bodies.
				if (other->IsActive() == false)
				{
					continue;
				}

				joints[jointCount++] = je->joint;
				je->joint->m_islandFlag = true;

				if (other->m_flags & b2Body::e_islandFlag)
				{
					continue;
				}

				b2Assert(stackCount < stackSize);
				stack[stackCount++] = other;
				other->m_flags |= b2Body::e_islandFlag;
			}
		}

		// Post island traversal cleanup.
		for (int32 j = 0; j < bodyCount; ++j)
		{
			// Allow static bodies to participate in other islands.
			b2Body* b = bodies[j];
			if (b->GetType() == b2_staticBody)
			{
				b->m_flags &= ~b2Body::e_islandFlag;
			}
		}

		if (currSolveTask == nullptr)
		{
			static_assert(sizeof(b2SolveTask) <= b2_maxBlockSize, "Solve task doesn't fit in the allocator");

			currSolveTask = (b2SolveTask*)m_blockAllocator.Allocate(sizeof(b2SolveTask));
			new(currSolveTask) b2SolveTask(m_contactManager.m_perThreadData, m_contactManager.m_contactListener,
				step, m_gravity, m_allowSleep, solveTaskList);

			solveTaskList = currSolveTask;
		}

		currSolveTask->AddIsland(bodyCount, contactCount, jointCount,
			bodies, contacts, joints, velocities, positions);

		bodies += bodyCount;
		contacts += contactCount;
		joints += jointCount;
		velocities += bodyCount;
		positions += bodyCount;

		allBodiesCount += bodyCount;
		allContactsCount += contactCount;
		allJointsCount += jointCount;

		bodyCount = 0;
		contactCount = 0;
		jointCount = 0;

		b2Assert(allBodiesCount <= allBodiesCapacity);
		b2Assert(allContactsCount <= allContactsCapacity);
		b2Assert(allJointsCount <= allJointsCapacity);

		if (currSolveTask->GetCost() >= b2_solveBatchTargetCost ||
			currSolveTask->GetBodyCount() >= b2_solveBatchTargetBodyCount)
		{
			b2SubmitTask(executor, taskGroup, currSolveTask);
			currSolveTask = nullptr;
		}
	}

	// Pick up stragglers.
	if (currSolveTask != nullptr)
	{
		b2SubmitTask(executor, taskGroup, currSolveTask);
		currSolveTask = nullptr;
	}

	m_profile.solveTraversal += traversalTimer.GetMilliseconds();

	// Wait for solve tasks to finish.
	executor.Wait(taskGroup, b2MainThreadCtx(m_stackAllocator));

	// Start sorting post solves.
	b2SortPostSolvesTask sortTasks[b2_maxThreads];
	for (uint32 i = 0; i < threadCount; ++i)
	{
		sortTasks[i] = b2SortPostSolvesTask(m_contactManager.m_perThreadData[i]);
	}
	b2SubmitTasks(executor, taskGroup, sortTasks, threadCount);

	// Deallocate tasks.
	while (solveTaskList)
	{
		b2SolveTask* task = solveTaskList;

		solveTaskList = solveTaskList->GetNext();

		// Free the task
		task->~b2SolveTask();
		m_blockAllocator.Free(task, sizeof(b2SolveTask));
	}

	// Free stack
	m_stackAllocator.Free(stack);

	// Free island memory.
	m_stackAllocator.Free(allPositions);
	m_stackAllocator.Free(allVelocities);
	m_stackAllocator.Free(allJoints);
	m_stackAllocator.Free(allContacts);
	m_stackAllocator.Free(allBodies);

	executor.Wait(taskGroup, b2MainThreadCtx(m_stackAllocator));
	m_contactManager.ConsumeDeferredPostSolves(threadCount);

	{
		b2Timer timer;

		SynchronizeFixtures(executor, taskGroup, threadCount);
		m_profile.broadphaseSyncFixtures += timer.GetMilliseconds();

		{
			b2Timer timer2;

			FindNewContacts(executor, taskGroup, threadCount);

			m_profile.broadphaseFindContacts += timer2.GetMilliseconds();
		}

		float32 broadPhaseTime = timer.GetMilliseconds();
		m_profile.broadphase += broadPhaseTime;
		m_profile.solve -= broadPhaseTime;
	}
}

void b2World::SolveInit(b2TaskExecutor& executor, b2TaskGroup taskGroup)
{
	b2ContactPreSolveTask contactsTasks[b2_partitionRangeMaxOutput];
	if (m_contactManager.GetNonToiCount() > 0)
	{
		b2PartitionedRange ranges;
		executor.PartitionRange(0, m_contactManager.GetNonToiCount(), ranges);
		for (uint32 i = 0; i < ranges.count; ++i)
		{
			contactsTasks[i] = b2ContactPreSolveTask(ranges[i], m_contactManager.GetNonToiBegin(), false);
		}
		b2SubmitTasks(executor, taskGroup, contactsTasks, ranges.count);
	}
	b2ContactPreSolveTask toiContactsTasks[b2_partitionRangeMaxOutput];
	if (m_contactManager.m_toiCount > 0)
	{
		b2PartitionedRange ranges;
		executor.PartitionRange(0, m_contactManager.m_toiCount, ranges);
		for (uint32 i = 0; i < ranges.count; ++i)
		{
			toiContactsTasks[i] = b2ContactPreSolveTask(ranges[i], m_contactManager.GetToiBegin(), true);
		}
		b2SubmitTasks(executor, taskGroup, toiContactsTasks, ranges.count);
	}
	b2BodyPreSolveTask bodyTasks[b2_partitionRangeMaxOutput];
	if (m_nonStaticBodies.GetCount() > 0)
	{
		b2PartitionedRange ranges;
		executor.PartitionRange(0, m_nonStaticBodies.GetCount(), ranges);
		for (uint32 i = 0; i < ranges.count; ++i)
		{
			bodyTasks[i] = b2BodyPreSolveTask(ranges[i], m_nonStaticBodies.Data());
		}
		b2SubmitTasks(executor, taskGroup, bodyTasks, ranges.count);
	}

	for (b2Joint* j = m_jointList; j; j = j->m_next)
	{
		j->m_islandFlag = false;
	}

	executor.Wait(taskGroup, b2MainThreadCtx(m_stackAllocator));
}

void b2World::SolveTOIInit(b2TaskExecutor& executor, b2TaskGroup taskGroup)
{
	b2ContactPreSolveTask contactsTasks[b2_partitionRangeMaxOutput];
	if (m_contactManager.GetNonToiCount() > 0)
	{
		b2PartitionedRange ranges;
		executor.PartitionRange(0, m_contactManager.GetNonToiCount(), ranges);
		for (uint32 i = 0; i < ranges.count; ++i)
		{
			contactsTasks[i] = b2ContactPreSolveTask(ranges[i], m_contactManager.GetNonToiBegin(), false);
		}
		b2SubmitTasks(executor, taskGroup, contactsTasks, ranges.count);
	}
	b2ContactPreSolveTask toiContactsTasks[b2_partitionRangeMaxOutput];
	if (m_contactManager.m_toiCount > 0)
	{
		b2PartitionedRange ranges;
		executor.PartitionRange(0, m_contactManager.m_toiCount, ranges);
		for (uint32 i = 0; i < ranges.count; ++i)
		{
			toiContactsTasks[i] = b2ContactPreSolveTask(ranges[i], m_contactManager.GetToiBegin(), true);
		}
		b2SubmitTasks(executor, taskGroup, toiContactsTasks, ranges.count);
	}
	b2BodyPreSolveTask bodyTasks[b2_partitionRangeMaxOutput];
	if (m_nonStaticBodies.GetCount() > 0)
	{
		b2PartitionedRange ranges;
		executor.PartitionRange(0, m_nonStaticBodies.GetCount(), ranges);
		for (uint32 i = 0; i < ranges.count; ++i)
		{
			bodyTasks[i] = b2BodyPreSolveTask(ranges[i], m_nonStaticBodies.Data());
		}
		b2SubmitTasks(executor, taskGroup, bodyTasks, ranges.count);
	}

	for (int32 i = 0; i < m_staticBodies.GetCount(); ++i)
	{
		b2Body* b = m_staticBodies[i];
		b->m_flags &= ~b2Body::e_islandFlag;
		b->m_sweep.alpha0 = 0.0f;
	}

	executor.Wait(taskGroup, b2MainThreadCtx(m_stackAllocator));
}

void b2World::Step(float32 dt, int32 velocityIterations, int32 positionIterations, b2TaskExecutor& executor)
{
	executor.StepBegin();

	uint32 threadCount = executor.GetThreadCount();

	b2Timer stepTimer;

	memset(&m_profile, 0, sizeof(m_profile));
	for (uint32 i = 0; i < threadCount; ++i)
	{
		memset(&m_contactManager.m_perThreadData[i].m_profile, 0, sizeof(b2Profile));
	}

	b2TaskGroup taskGroup = executor.CreateTaskGroup(m_stackAllocator);

	// If new fixtures were added, we need to find the new contacts.
	if (m_flags & e_newFixture)
	{
		b2Timer timer;

		FindNewContacts(executor, taskGroup, threadCount);

		float32 elapsed = timer.GetMilliseconds();
		m_profile.broadphase += elapsed;
		m_profile.broadphaseFindContacts += elapsed;

		m_flags &= ~e_newFixture;
	}

	m_flags |= e_locked;

	// Update contacts. This is where some contacts are destroyed.
	{
		b2Timer timer;
		Collide(executor, taskGroup, threadCount);
		m_profile.collide = timer.GetMilliseconds();
	}

	b2TimeStep step;
	step.dt = dt;
	step.velocityIterations	= velocityIterations;
	step.positionIterations = positionIterations;
	if (dt > 0.0f)
	{
		step.inv_dt = 1.0f / dt;
	}
	else
	{
		step.inv_dt = 0.0f;
	}

	step.dtRatio = m_inv_dt0 * dt;

	step.warmStarting = m_warmStarting;

	// Integrate velocities, solve velocity constraints, and integrate positions.
	if (m_stepComplete && step.dt > 0.0f)
	{
		b2Timer timer;
		Solve(executor, taskGroup, step, threadCount);
		m_profile.solve += timer.GetMilliseconds();
	}

	// Handle TOI events.
	if (m_continuousPhysics && step.dt > 0.0f)
	{
		b2Timer timer;
		SolveTOI(executor, taskGroup, step);
		m_profile.solveTOI += timer.GetMilliseconds();
	}

	if (step.dt > 0.0f)
	{
		m_inv_dt0 = step.inv_dt;
	}

	if (m_flags & e_clearForces)
	{
		ClearForces();
	}

	m_flags &= ~e_locked;

	executor.DestroyTaskGroup(taskGroup, m_stackAllocator);

	// Add per-thread profile times.
	for (uint32 i = 0; i < threadCount; ++i)
	{
		const b2ContactManagerPerThreadData& td = m_contactManager.m_perThreadData[i];
		m_profile.solveInit += td.m_profile.solveInit;
		m_profile.solveVelocity += td.m_profile.solveVelocity;
		m_profile.solvePosition += td.m_profile.solvePosition;
	}

	m_profile.step = stepTimer.GetMilliseconds();
	stepTimer.Reset();

	executor.StepEnd(m_profile);

	m_profile.step += stepTimer.GetMilliseconds();
}

void b2World::RecalculateToiCandidacy(b2Body* b)
{
	m_contactManager.RecalculateToiCandidacy(b);
}

void b2World::ClearForces()
{
	for (b2Body* body = m_bodyList; body; body = body->GetNext())
	{
		body->m_force.SetZero();
		body->m_torque = 0.0f;
	}
}

struct b2WorldQueryWrapper
{
	bool QueryCallback(int32 proxyId)
	{
		b2FixtureProxy* proxy = (b2FixtureProxy*)broadPhase->GetUserData(proxyId);
		return callback->ReportFixture(proxy->fixture);
	}

	const b2BroadPhase* broadPhase;
	b2QueryCallback* callback;
};

void b2World::QueryAABB(b2QueryCallback* callback, const b2AABB& aabb) const
{
	b2WorldQueryWrapper wrapper;
	wrapper.broadPhase = &m_contactManager.m_broadPhase;
	wrapper.callback = callback;
	m_contactManager.m_broadPhase.Query(&wrapper, aabb);
}

struct b2WorldRayCastWrapper
{
	float32 RayCastCallback(const b2RayCastInput& input, int32 proxyId)
	{
		void* userData = broadPhase->GetUserData(proxyId);
		b2FixtureProxy* proxy = (b2FixtureProxy*)userData;
		b2Fixture* fixture = proxy->fixture;
		int32 index = proxy->childIndex;
		b2RayCastOutput output;
		bool hit = fixture->RayCast(&output, input, index);

		if (hit)
		{
			float32 fraction = output.fraction;
			b2Vec2 point = (1.0f - fraction) * input.p1 + fraction * input.p2;
			return callback->ReportFixture(fixture, point, output.normal, fraction);
		}

		return input.maxFraction;
	}

	const b2BroadPhase* broadPhase;
	b2RayCastCallback* callback;
};

void b2World::RayCast(b2RayCastCallback* callback, const b2Vec2& point1, const b2Vec2& point2) const
{
	b2WorldRayCastWrapper wrapper;
	wrapper.broadPhase = &m_contactManager.m_broadPhase;
	wrapper.callback = callback;
	b2RayCastInput input;
	input.maxFraction = 1.0f;
	input.p1 = point1;
	input.p2 = point2;
	m_contactManager.m_broadPhase.RayCast(&wrapper, input);
}

void b2World::DrawShape(b2Fixture* fixture, const b2Transform& xf, const b2Color& color)
{
	switch (fixture->GetType())
	{
	case b2Shape::e_circle:
		{
			b2CircleShape* circle = (b2CircleShape*)fixture->GetShape();

			b2Vec2 center = b2Mul(xf, circle->m_p);
			float32 radius = circle->m_radius;
			b2Vec2 axis = b2Mul(xf.q, b2Vec2(1.0f, 0.0f));

			m_debugDraw->DrawSolidCircle(center, radius, axis, color);
		}
		break;

	case b2Shape::e_edge:
		{
			b2EdgeShape* edge = (b2EdgeShape*)fixture->GetShape();
			b2Vec2 v1 = b2Mul(xf, edge->m_vertex1);
			b2Vec2 v2 = b2Mul(xf, edge->m_vertex2);
			m_debugDraw->DrawSegment(v1, v2, color);
		}
		break;

	case b2Shape::e_chain:
		{
			b2ChainShape* chain = (b2ChainShape*)fixture->GetShape();
			int32 count = chain->m_count;
			const b2Vec2* vertices = chain->m_vertices;

			b2Color ghostColor(0.75f * color.r, 0.75f * color.g, 0.75f * color.b, color.a);

			b2Vec2 v1 = b2Mul(xf, vertices[0]);
			m_debugDraw->DrawPoint(v1, 4.0f, color);

			if (chain->m_hasPrevVertex)
			{
				b2Vec2 vp = b2Mul(xf, chain->m_prevVertex);
				m_debugDraw->DrawSegment(vp, v1, ghostColor);
				m_debugDraw->DrawCircle(vp, 0.1f, ghostColor);
			}

			for (int32 i = 1; i < count; ++i)
			{
				b2Vec2 v2 = b2Mul(xf, vertices[i]);
				m_debugDraw->DrawSegment(v1, v2, color);
				m_debugDraw->DrawPoint(v2, 4.0f, color);
				v1 = v2;
			}

			if (chain->m_hasNextVertex)
			{
				b2Vec2 vn = b2Mul(xf, chain->m_nextVertex);
				m_debugDraw->DrawSegment(v1, vn, ghostColor);
				m_debugDraw->DrawCircle(vn, 0.1f, ghostColor);
			}
		}
		break;

	case b2Shape::e_polygon:
		{
			b2PolygonShape* poly = (b2PolygonShape*)fixture->GetShape();
			int32 vertexCount = poly->m_count;
			b2Assert(vertexCount <= b2_maxPolygonVertices);
			b2Vec2 vertices[b2_maxPolygonVertices];

			for (int32 i = 0; i < vertexCount; ++i)
			{
				vertices[i] = b2Mul(xf, poly->m_vertices[i]);
			}

			m_debugDraw->DrawSolidPolygon(vertices, vertexCount, color);
		}
		break;

    default:
        break;
	}
}

void b2World::DrawJoint(b2Joint* joint)
{
	b2Body* bodyA = joint->GetBodyA();
	b2Body* bodyB = joint->GetBodyB();
	const b2Transform& xf1 = bodyA->GetTransform();
	const b2Transform& xf2 = bodyB->GetTransform();
	b2Vec2 x1 = xf1.p;
	b2Vec2 x2 = xf2.p;
	b2Vec2 p1 = joint->GetAnchorA();
	b2Vec2 p2 = joint->GetAnchorB();

	b2Color color(0.5f, 0.8f, 0.8f);

	switch (joint->GetType())
	{
	case e_distanceJoint:
		m_debugDraw->DrawSegment(p1, p2, color);
		break;

	case e_pulleyJoint:
	{
		b2PulleyJoint* pulley = (b2PulleyJoint*)joint;
		b2Vec2 s1 = pulley->GetGroundAnchorA();
		b2Vec2 s2 = pulley->GetGroundAnchorB();
		m_debugDraw->DrawSegment(s1, p1, color);
		m_debugDraw->DrawSegment(s2, p2, color);
		m_debugDraw->DrawSegment(s1, s2, color);
	}
	break;

	case e_mouseJoint:
	{
		b2Color c;
		c.Set(0.0f, 1.0f, 0.0f);
		m_debugDraw->DrawPoint(p1, 4.0f, c);
		m_debugDraw->DrawPoint(p2, 4.0f, c);

		c.Set(0.8f, 0.8f, 0.8f);
		m_debugDraw->DrawSegment(p1, p2, c);

	}
	break;

	default:
		m_debugDraw->DrawSegment(x1, p1, color);
		m_debugDraw->DrawSegment(p1, p2, color);
		m_debugDraw->DrawSegment(x2, p2, color);
	}
}

void b2World::DrawDebugData()
{
	if (m_debugDraw == nullptr)
	{
		return;
	}

	uint32 flags = m_debugDraw->GetFlags();

	if (flags & b2Draw::e_shapeBit)
	{
		for (b2Body* b = m_bodyList; b; b = b->GetNext())
		{
			const b2Transform& xf = b->GetTransform();
			for (b2Fixture* f = b->GetFixtureList(); f; f = f->GetNext())
			{
				if (b->IsActive() == false)
				{
					DrawShape(f, xf, b2Color(0.5f, 0.5f, 0.3f));
				}
				else if (b->GetType() == b2_staticBody)
				{
					DrawShape(f, xf, b2Color(0.5f, 0.9f, 0.5f));
				}
				else if (b->GetType() == b2_kinematicBody)
				{
					DrawShape(f, xf, b2Color(0.5f, 0.5f, 0.9f));
				}
				else if (b->IsAwake() == false)
				{
					DrawShape(f, xf, b2Color(0.6f, 0.6f, 0.6f));
				}
				else
				{
					DrawShape(f, xf, b2Color(0.9f, 0.7f, 0.7f));
				}
			}
		}
	}

	if (flags & b2Draw::e_jointBit)
	{
		for (b2Joint* j = m_jointList; j; j = j->GetNext())
		{
			DrawJoint(j);
		}
	}

	if (flags & b2Draw::e_pairBit)
	{
		b2Color color(0.3f, 0.9f, 0.9f);
		for (b2Contact* c = m_contactManager.m_contactList; c; c = c->GetNext())
		{
			//b2Fixture* fixtureA = c->GetFixtureA();
			//b2Fixture* fixtureB = c->GetFixtureB();

			//b2Vec2 cA = fixtureA->GetAABB().GetCenter();
			//b2Vec2 cB = fixtureB->GetAABB().GetCenter();

			//g_debugDraw->DrawSegment(cA, cB, color);
		}
	}

	if (flags & b2Draw::e_aabbBit)
	{
		b2Color color(0.9f, 0.3f, 0.9f);
		b2BroadPhase* bp = &m_contactManager.m_broadPhase;

		for (b2Body* b = m_bodyList; b; b = b->GetNext())
		{
			if (b->IsActive() == false)
			{
				continue;
			}

			for (b2Fixture* f = b->GetFixtureList(); f; f = f->GetNext())
			{
				for (int32 i = 0; i < f->m_proxyCount; ++i)
				{
					b2FixtureProxy* proxy = f->m_proxies + i;
					b2AABB aabb = bp->GetFatAABB(proxy->proxyId);
					b2Vec2 vs[4];
					vs[0].Set(aabb.lowerBound.x, aabb.lowerBound.y);
					vs[1].Set(aabb.upperBound.x, aabb.lowerBound.y);
					vs[2].Set(aabb.upperBound.x, aabb.upperBound.y);
					vs[3].Set(aabb.lowerBound.x, aabb.upperBound.y);

					m_debugDraw->DrawPolygon(vs, 4, color);
				}
			}
		}
	}

	if (flags & b2Draw::e_centerOfMassBit)
	{
		for (b2Body* b = m_bodyList; b; b = b->GetNext())
		{
			b2Transform xf = b->GetTransform();
			xf.p = b->GetWorldCenter();
			m_debugDraw->DrawTransform(xf);
		}
	}
}

int32 b2World::GetProxyCount() const
{
	return m_contactManager.m_broadPhase.GetProxyCount();
}

int32 b2World::GetTreeHeight() const
{
	return m_contactManager.m_broadPhase.GetTreeHeight();
}

int32 b2World::GetTreeBalance() const
{
	return m_contactManager.m_broadPhase.GetTreeBalance();
}

float32 b2World::GetTreeQuality() const
{
	return m_contactManager.m_broadPhase.GetTreeQuality();
}

void b2World::ShiftOrigin(const b2Vec2& newOrigin)
{
	b2Assert((m_flags & e_locked) == 0);
	if ((m_flags & e_locked) == e_locked)
	{
		return;
	}

	for (b2Body* b = m_bodyList; b; b = b->m_next)
	{
		b->m_xf.p -= newOrigin;
		b->m_sweep.c0 -= newOrigin;
		b->m_sweep.c -= newOrigin;
	}

	for (b2Joint* j = m_jointList; j; j = j->m_next)
	{
		j->ShiftOrigin(newOrigin);
	}

	m_contactManager.m_broadPhase.ShiftOrigin(newOrigin);
}

void b2World::Dump()
{
	if ((m_flags & e_locked) == e_locked)
	{
		return;
	}

	b2Log("b2Vec2 g(%.15lef, %.15lef);\n", m_gravity.x, m_gravity.y);
	b2Log("m_world->SetGravity(g);\n");

	b2Log("b2Body** bodies = (b2Body**)b2Alloc(%d * sizeof(b2Body*));\n", m_bodyCount);
	b2Log("b2Joint** joints = (b2Joint**)b2Alloc(%d * sizeof(b2Joint*));\n", m_jointCount);
	int32 i = 0;
	for (b2Body* b = m_bodyList; b; b = b->m_next)
	{
		b->SetIslandIndex(i, 0);
		b->Dump();
		++i;
	}

	i = 0;
	for (b2Joint* j = m_jointList; j; j = j->m_next)
	{
		j->m_index = i;
		++i;
	}

	// First pass on joints, skip gear joints.
	for (b2Joint* j = m_jointList; j; j = j->m_next)
	{
		if (j->m_type == e_gearJoint)
		{
			continue;
		}

		b2Log("{\n");
		j->Dump();
		b2Log("}\n");
	}

	// Second pass on joints, only gear joints.
	for (b2Joint* j = m_jointList; j; j = j->m_next)
	{
		if (j->m_type != e_gearJoint)
		{
			continue;
		}

		b2Log("{\n");
		j->Dump();
		b2Log("}\n");
	}

	b2Log("b2Free(joints);\n");
	b2Log("b2Free(bodies);\n");
	b2Log("joints = nullptr;\n");
	b2Log("bodies = nullptr;\n");
}
