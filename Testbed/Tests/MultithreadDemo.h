/*
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

#ifndef MULTITHREAD_DEMO_H
#define MULTITHREAD_DEMO_H

class MultithreadDemo : public Test
{
public:
	enum
	{
		e_boxcount = 2800
	};

	MultithreadDemo()
	{
		m_boxCount = 0;

		// Ground
		{
			b2EdgeShape edgeShape;

			edgeShape.Set(b2Vec2(-15.0f, 60.0f), b2Vec2(-10.0f, 55.0f));
			m_groundBody->CreateFixture(&edgeShape, 0.0f);

			edgeShape.Set(b2Vec2(15.0f, 60.0f), b2Vec2(10.0f, 55.0f));
			m_groundBody->CreateFixture(&edgeShape, 0.0f);

			b2PolygonShape shape;

			b2FixtureDef fd;
			fd.shape = &shape;
			// Fixtures marked as thick walls will only generate TOI events with bullet bodies.
			// This reduces the performance cost of TOI.
			fd.isThickWall = true;

			shape.SetAsBox(25.0f, 2.5f, b2Vec2(0.0f, -2.5f), 0.0f);
			m_groundBody->CreateFixture(&fd);

			shape.SetAsBox(2.5f, 47.5f, b2Vec2(-22.5f, 42.5f), 0.0f);
			m_groundBody->CreateFixture(&fd);

			shape.SetAsBox(2.5f, 47.5f, b2Vec2(22.5f, 42.5f), 0.0f);
			m_groundBody->CreateFixture(&fd);

			shape.SetAsBox(2.5f, 2.0f, b2Vec2(-7.5f, 5.0f), 0.0f);
			m_groundBody->CreateFixture(&fd);

			shape.SetAsBox(2.5f, 2.0f, b2Vec2(7.5f, 5.0f), 0.0f);
			m_groundBody->CreateFixture(&fd);

			shape.SetAsBox(5.0f, 2.0f, b2Vec2(0.0f, 12.0f), 0.0f);
			m_groundBody->CreateFixture(&fd);

			shape.SetAsBox(3.5f, 2.0f, b2Vec2(-7.5f, 45.0f), 0.0f);
			m_groundBody->CreateFixture(&fd);

			shape.SetAsBox(3.5f, 2.0f, b2Vec2(7.5f, 45.0f), 0.0f);
			m_groundBody->CreateFixture(&fd);

			shape.SetAsBox(2.5f, 2.0f, b2Vec2(-6.5f, 63.0f), 0.0f);
			m_groundBody->CreateFixture(&fd);

			shape.SetAsBox(2.5f, 2.0f, b2Vec2(6.5f, 63.0f), 0.0f);
			m_groundBody->CreateFixture(&fd);

			shape.SetAsBox(5.0f, 2.0f, b2Vec2(0.0f, 72.0f), 0.0f);
			m_groundBody->CreateFixture(&fd);

			shape.SetAsBox(25.0f, 2.5f, b2Vec2(0.0f, 87.5f), 0.0f);
			m_groundBody->CreateFixture(&fd);

			shape.SetAsBox(4.0f, 2.5f, b2Vec2(-20.0f, 85.0f), b2_pi / 4);
			m_groundBody->CreateFixture(&fd);

			shape.SetAsBox(4.0f, 2.5f, b2Vec2(20.0f, 85.0f), -b2_pi / 4);
			m_groundBody->CreateFixture(&fd);
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
		bool doStep = settings->pause == false || settings->singleStep;

		Test::Step(settings);

		if (doStep == false)
		{
			return;
		}

		for (b2ContactEdge* ce = m_uppers->GetContactList(); ce; ce = ce->next)
		{
			ce->other->ApplyForceToCenter(b2Vec2(0, 1.25f), true);
		}

		float32 x[9] = { -16.0f, -12.0f, -8.0f, -4.0f, 0.0f, 4.0f, 8.0f, 12.0f, 16.0f };
		for (int32 i = 0; i < 9 && m_boxCount < e_boxcount; ++i)
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

			++m_boxCount;
		}

		if ((m_slider->GetJointSpeed() > 0 && m_slider->GetJointTranslation() >= m_slider->GetUpperLimit() - b2_epsilon) ||
			(m_slider->GetJointSpeed() < 0 && m_slider->GetJointTranslation() <= m_slider->GetLowerLimit() + b2_epsilon))
		{
			m_slider->SetMotorSpeed(-m_slider->GetMotorSpeed());
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

		b2FixtureDef fd;
		fd.shape = &shape;
		fd.density = 5.0f;

		shape.SetAsBox(0.2f, armLength, b2Vec2_zero, 0);
		body->CreateFixture(&fd);

		shape.SetAsBox(0.2f, armLength, b2Vec2_zero, b2_pi / 2);
		body->CreateFixture(&fd);

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

		b2FixtureDef fd;
		fd.isSensor = true;
		fd.shape = &shape;

		shape.SetAsBox(2.0f, 42.5f, b2Vec2(-18.0f, 42.5f), 0);
		b2Fixture* fixture = m_uppers->CreateFixture(&fd);

		shape.SetAsBox(2.0f, 42.5f, b2Vec2(18.0f, 42.5f), 0);
		fixture = m_uppers->CreateFixture(&fd);
	}

	void CreateSlider()
	{
		b2BodyDef bd;
		bd.type = b2_dynamicBody;
		bd.position.Set(0.0f, 1.0f);
		bd.angle = 0.5f * b2_pi;
		bd.allowSleep = false;
		b2Body* body = m_world->CreateBody(&bd);

		b2PolygonShape shape;
		shape.SetAsBox(1.0f, 1.0f);

		b2FixtureDef fd;
		fd.shape = &shape;
		fd.density = 5.0f;

		body->CreateFixture(&fd);

		b2PrismaticJointDef pjd;

		pjd.Initialize(m_groundBody, body, b2Vec2(0.0f, 0.0f), b2Vec2(1.0f, 0.0f));
		pjd.motorSpeed = 8.0f;
		pjd.maxMotorForce = 10000.0f;
		pjd.enableMotor = true;
		pjd.lowerTranslation = -12.0f;
		pjd.upperTranslation = 12.0f;
		pjd.enableLimit = true;

		m_slider = (b2PrismaticJoint*)m_world->CreateJoint(&pjd);
	}

	b2Body* m_uppers;
	b2PrismaticJoint* m_slider;

	int32 m_boxCount;
};

#endif
