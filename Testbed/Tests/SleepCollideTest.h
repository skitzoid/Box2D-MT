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

#ifndef SLEEP_COLLIDE_TEST_H
#define SLEEP_COLLIDE_TEST_H

// Regression test for a bug where collisions between sleeping bodies were ignored.
class SleepCollideTest : public Test
{
public:
	enum
	{
		e_count = 20
	};

	SleepCollideTest()
	{
        m_box = nullptr;
        m_passed = true;

		{
			b2BodyDef bd;
			b2Body* ground = m_world->CreateBody(&bd);

			b2EdgeShape shape;
			shape.Set(b2Vec2(-40.0f, 0.0f), b2Vec2(40.0f, 0.0f));
			ground->CreateFixture(&shape, 0.0f);
		}

		{
			float32 a = 0.5f;
			b2PolygonShape shape;
			shape.SetAsBox(a, a);

			b2Vec2 x(-7.0f, 0.75f);
			b2Vec2 y;
			b2Vec2 deltaX(0.5625f, 1.25f);
			b2Vec2 deltaY(1.125f, 0.0f);

			for (int32 i = 0; i < e_count; ++i)
			{
				y = x;

                b2Body* body;
				for (int32 j = i; j < e_count; ++j)
				{
					b2BodyDef bd;
					bd.type = b2_dynamicBody;
					bd.position = y;
					body = m_world->CreateBody(&bd);
					body->CreateFixture(&shape, 5.0f);

					y += deltaY;
				}

                if (m_box == nullptr)
                {
                    m_box = body;
                }

				x += deltaX;
			}
		}

        {
            b2BodyDef bd;
            bd.type = b2_dynamicBody;
            bd.position.Set(-8.0f, 1.0f);

            m_ball = m_world->CreateBody(&bd);

            b2CircleShape shape;
            shape.m_radius = 0.5f;

            m_ball->CreateFixture(&shape, 100.0f);
        }
	}

	void Step(Settings* settings)
	{
		Test::Step(settings);

        // Push the ball into the pyramid if the box on the opposite end is sleeping.
        if (m_box->IsAwake() == false)
        {
            m_ball->SetLinearVelocity(b2Vec2(0.5f, 0.0f));
        }

        // Check for bodies falling through the ground.
        for (b2Body* b = m_world->GetBodyList(); b; b = b->GetNext())
        {
            if (b->GetPosition().y < 0.0f)
            {
                m_passed = false;
            }
        }

        g_debugDraw.DrawString(5, m_textLine, "This checks for a bug that allowed sleeping bodies to fall through the ground.");
		m_textLine += DRAW_STRING_NEW_LINE;
		g_debugDraw.DrawString(5, m_textLine, "Status: %s", m_passed ? "PASSING" : "FAILED");
		m_textLine += DRAW_STRING_NEW_LINE;
	}

	static Test* Create()
	{
		return new SleepCollideTest;
	}

    TestResult TestPassed() const override { return m_passed ? TestResult::PASS : TestResult::FAIL; }

	bool m_passed;
    b2Body* m_box;
    b2Body* m_ball;
};

#endif
