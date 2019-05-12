/*
* Copyright (c) 2015 Justin Hoffman https://github.com/jhoffman0x/Box2D-MT
* Copyright (c) 2011 Erin Catto http://www.box2d.org

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

#ifndef SLEEP_COLLIDE_PERF_H
#define SLEEP_COLLIDE_PERF_H

class SleepCollidePerf : public Test
{
public:
	enum
	{
		e_pyramidSize = 20,
        e_pyramidCount = 40,
        e_tumblerSize = 800,
        e_tumblerCount = 4
	};

	SleepCollidePerf()
	{
        m_tumblerSize = 0;

		{
			b2BodyDef bd;
			b2Body* ground = m_world->CreateBody(&bd);

			b2EdgeShape shape;
			shape.Set(b2Vec2(-20.0f * e_pyramidCount, 0.0f), b2Vec2(20.0f * e_pyramidCount, 0.0f));
			ground->CreateFixture(&shape, 0.0f);
		}

        // Pyramids
        {
            const float32 xSpacing = 1.125f * e_pyramidSize;

		    b2Vec2 xInit(-xSpacing * e_pyramidCount * 0.5f - 7.0f, 0.75f);
            for (int32 i = 0; i < e_pyramidCount; ++i)
            {
                float32 a = 0.5f;
                b2PolygonShape shape;
                shape.SetAsBox(a, a);

    		    b2Vec2 x = xInit;
                b2Vec2 y;
                b2Vec2 deltaX(0.5625f, 1.25f);
                b2Vec2 deltaY(1.125f, 0.0f);

                for (int32 i = 0; i < e_pyramidSize; ++i)
                {
                    y = x;

                    for (int32 j = i; j < e_pyramidSize; ++j)
                    {
                        b2BodyDef bd;
                        bd.type = b2_dynamicBody;
                        bd.position = y;
                        b2Body* body = m_world->CreateBody(&bd);
                        body->CreateFixture(&shape, 5.0f);

                        y += deltaY;
                    }

                    x += deltaX;
                }

                xInit.x += xSpacing;
            }
        }


        // Tumbler
        {
            float32 x = -30.0f * e_tumblerCount * 0.5f + 10.0f;
            for (int32 i = 0; i < e_tumblerCount; ++i)
            {
                b2BodyDef bd;
                bd.type = b2_dynamicBody;
                bd.allowSleep = false;
                bd.position.Set(x, 50.0f);
                b2Body* body = m_world->CreateBody(&bd);

                b2PolygonShape shape;
                shape.SetAsBox(0.5f, 10.0f, b2Vec2( 10.0f, 0.0f), 0.0);
                body->CreateFixture(&shape, 5.0f);
                shape.SetAsBox(0.5f, 10.0f, b2Vec2(-10.0f, 0.0f), 0.0);
                body->CreateFixture(&shape, 5.0f);
                shape.SetAsBox(10.0f, 0.5f, b2Vec2(0.0f, 10.0f), 0.0);
                body->CreateFixture(&shape, 5.0f);
                shape.SetAsBox(10.0f, 0.5f, b2Vec2(0.0f, -10.0f), 0.0);
                body->CreateFixture(&shape, 5.0f);

                b2RevoluteJointDef jd;
                jd.bodyA = m_groundBody;
                jd.bodyB = body;
                jd.localAnchorA.Set(x, 50.0f);
                jd.localAnchorB.Set(0.0f, 0.0f);
                jd.referenceAngle = 0.0f;
                jd.motorSpeed = 0.05f * b2_pi;
                jd.maxMotorTorque = 1e8f;
                jd.enableMotor = true;
                m_world->CreateJoint(&jd);

                x += 30.0f;
            }
		}
	}

	void Step(Settings* settings)
	{
		Test::Step(settings);

		if (m_tumblerSize < e_tumblerSize * e_tumblerCount)
		{
            float32 x = -30.0f * e_tumblerCount * 0.5f + 10.0f;
            for (int32 i = 0; i < e_tumblerCount; ++i)
            {
                b2BodyDef bd;
                bd.type = b2_dynamicBody;
                bd.position.Set(x, 50.0f);
                b2Body* body = m_world->CreateBody(&bd);

                b2PolygonShape shape;
                shape.SetAsBox(0.125f, 0.125f);
                body->CreateFixture(&shape, 1.0f);

                ++m_tumblerSize;
                x += 30.0f;
            }
		}
	}

	static Test* Create()
	{
		return new SleepCollidePerf;
	}

	int32 m_tumblerSize;
};

#endif
