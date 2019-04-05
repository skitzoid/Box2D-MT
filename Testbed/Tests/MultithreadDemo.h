/*
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

#ifndef MULTITHREAD_DEMO_H
#define MULTITHREAD_DEMO_H

class MultithreadDemo : public Test
{
public:
	enum
	{
		e_boxcount = 2800
	};

	// All unused callbacks should override immediate functions to return DO_NOT_CALL_DEFERRED.
	// This avoids unnecessary overhead.
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
		// We can call Test::PreSolve here only because we ensured that it won't cause data races.
		Test::PreSolve(contact, oldManifold);

		return b2ImmediateCallbackResult::DO_NOT_CALL_DEFERRED;
	}
	b2ImmediateCallbackResult PostSolveImmediate(b2Contact* contact, const b2ContactImpulse* impulse) override
	{
		B2_NOT_USED(contact);
		B2_NOT_USED(impulse);
		return b2ImmediateCallbackResult::DO_NOT_CALL_DEFERRED;
	}

	MultithreadDemo()
	{
		m_count = 0;

		// Ground
		{
			// This world has been designed with large overlapping static bodies so tunneling is not an issue.
			// We can disable automatic TOI checks between the static ground body and the other dynamic bodies.
			// This helps because all CCD/TOI is done on a single thread.
			// With this flag the ground body will only use CCD against bodies that have the bullet flag set.
			m_groundBody->SetPreferNoCCD(true);

			b2PolygonShape shape;

			shape.SetAsBox(25.0f, 2.5f, b2Vec2(0.0f, -2.5f), 0.0f);
			m_groundBody->CreateFixture(&shape, 0.0f);

			shape.SetAsBox(2.5f, 47.5f, b2Vec2(-22.5f, 42.5f), 0.0f);
			m_groundBody->CreateFixture(&shape, 0.0f);

			shape.SetAsBox(2.5f, 47.5f, b2Vec2(22.5f, 42.5f), 0.0f);
			m_groundBody->CreateFixture(&shape, 0.0f);

			shape.SetAsBox(2.5f, 2.0f, b2Vec2(-7.5f, 5.0f), 0.0f);
			m_groundBody->CreateFixture(&shape, 0.0f);

			shape.SetAsBox(2.5f, 2.0f, b2Vec2(7.5f, 5.0f), 0.0f);
			m_groundBody->CreateFixture(&shape, 0.0f);

			shape.SetAsBox(5.0f, 2.0f, b2Vec2(0.0f, 12.0f), 0.0f);
			m_groundBody->CreateFixture(&shape, 0.0f);

			shape.SetAsBox(3.5f, 2.0f, b2Vec2(-7.5f, 45.0f), 0.0f);
			m_groundBody->CreateFixture(&shape, 0.0f);

			shape.SetAsBox(3.5f, 2.0f, b2Vec2(7.5f, 45.0f), 0.0f);
			m_groundBody->CreateFixture(&shape, 0.0f);

			shape.SetAsBox(2.5f, 2.0f, b2Vec2(-6.5f, 63.0f), 0.0f);
			m_groundBody->CreateFixture(&shape, 0.0f);

			shape.SetAsBox(2.5f, 2.0f, b2Vec2(6.5f, 63.0f), 0.0f);
			m_groundBody->CreateFixture(&shape, 0.0f);

			shape.SetAsBox(5.0f, 2.0f, b2Vec2(0.0f, 72.0f), 0.0f);
			m_groundBody->CreateFixture(&shape, 0.0f);

			shape.SetAsBox(25.0f, 2.5f, b2Vec2(0.0f, 87.5f), 0.0f);
			m_groundBody->CreateFixture(&shape, 0.0f);

			shape.SetAsBox(4.0f, 2.5f, b2Vec2(-20.0f, 85.0f), b2_pi / 4);
			m_groundBody->CreateFixture(&shape, 0.0f);

			shape.SetAsBox(4.0f, 2.5f, b2Vec2(20.0f, 85.0f), -b2_pi / 4);
			m_groundBody->CreateFixture(&shape, 0.0f);
		}

		CreateUppers();
		CreateSlider();
		CreateWheel(b2Vec2(10.0f, 22.0f), 5.0f);
		CreateWheel(b2Vec2(-10.0f, 22.0f), 5.0f);
		CreateWheel(b2Vec2(-15.0f, 35.0f), 4.0f);
		CreateWheel(b2Vec2(0.0f, 35.0f), 4.0f);
		CreateWheel(b2Vec2(15.0f, 35.0f), 4.0f);
		CreateWheel(b2Vec2(0.0f, 53.0f), 5.0f);
		CreateWheel(b2Vec2(-13.0f, 77.0f), 4.0f);
		CreateWheel(b2Vec2(13.0f, 77.0f), 4.0f);
	}

	void Step(Settings* settings)
	{
		Test::Step(settings);

		float32 x[2] = { -4.0f, 4.0f };
		for (int32 i = 0; i < 2 && m_count < e_boxcount; ++i)
		{
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position = b2Vec2(x[i], 80.0f);
			b2Body* body = m_world->CreateBody(&bd);

			b2PolygonShape shape;
			shape.SetAsBox(0.15f, 0.15f);

			b2FixtureDef fd;
			fd.shape = &shape;
			fd.density = 1.0f;
			fd.friction = 1.0f;
			body->CreateFixture(&shape, 1.0f);

			++m_count;
		}

		if ((m_slider->GetJointSpeed() > 0 && m_slider->GetJointTranslation() >= m_slider->GetUpperLimit() - b2_epsilon) ||
			(m_slider->GetJointSpeed() < 0 && m_slider->GetJointTranslation() <= m_slider->GetLowerLimit() + b2_epsilon))
		{
			m_slider->SetMotorSpeed(-m_slider->GetMotorSpeed());
		}

		for (b2ContactEdge* ce = m_uppers->GetContactList(); ce; ce = ce->next)
		{
			b2Contact* contact = ce->contact;
			if (contact->IsTouching())
			{
				ce->other->ApplyForceToCenter(b2Vec2(0.0f, 1.25f), true);
			}
		}
	}

	static Test* Create()
	{
		return new MultithreadDemo;
	}

	void CreateWheel(b2Vec2 position, float32 armLength)
	{
		b2BodyDef bd;
		bd.type = b2_dynamicBody;
		bd.allowSleep = false;
		bd.position = position;
		bd.angularDamping = 0.5f;
		b2Body* body = m_world->CreateBody(&bd);

		b2PolygonShape shape;
		shape.SetAsBox(0.2f, armLength, b2Vec2_zero, 0);
		body->CreateFixture(&shape, 5.0f);
		shape.SetAsBox(0.2f, armLength, b2Vec2_zero, b2_pi / 2);
		body->CreateFixture(&shape, 5.0f);

		b2RevoluteJointDef jd;
		jd.bodyA = m_groundBody;
		jd.bodyB = body;
		jd.localAnchorA = position;
		jd.localAnchorB.Set(0.0f, 0.0f);
		jd.referenceAngle = 0.0f;

		m_world->CreateJoint(&jd);
	}

	void CreateUppers()
	{
		b2BodyDef bd;
		m_uppers = m_world->CreateBody(&bd);

		b2PolygonShape shape;

		b2FixtureDef def;
		def.isSensor = true;
		def.shape = &shape;

		shape.SetAsBox(2.0f, 42.5f, b2Vec2(18.0f, 42.5f), 0);
		m_uppers->CreateFixture(&def);
		shape.SetAsBox(-2.0f, 42.5f, b2Vec2(-18.0f, 42.5f), 0);
		m_uppers->CreateFixture(&def);
	}

	void CreateSlider()
	{
		b2PolygonShape shape;
		shape.SetAsBox(1.0f, 1.0f);

		b2BodyDef bd;
		bd.type = b2_dynamicBody;
		bd.position.Set(0.0f, 1.0f);
		bd.angle = 0.5f * b2_pi;
		bd.allowSleep = false;
		b2Body* body = m_world->CreateBody(&bd);
		body->CreateFixture(&shape, 5.0f);

		b2PrismaticJointDef pjd;

		// Bouncy limit
		pjd.Initialize(m_groundBody, body, b2Vec2(0.0f, 0.0f), b2Vec2(1.0f, 0.0f));
		pjd.motorSpeed = 8.0f;
		pjd.maxMotorForce = 100000.0f;
		pjd.enableMotor = true;
		pjd.lowerTranslation = -12.0f;
		pjd.upperTranslation = 12.0f;
		pjd.enableLimit = true;

		m_slider = (b2PrismaticJoint*)m_world->CreateJoint(&pjd);
	}

	b2Body* m_uppers;
	b2PrismaticJoint* m_slider;

	int32 m_count;
};

#endif
