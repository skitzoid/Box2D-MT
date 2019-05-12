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

#ifndef TUNNELING_TEST_H
#define TUNNELING_TEST_H

// Box2D-MT partitions contacts based on CCD eligibility, which improves
// SolveTOI performance when there's many non-bullet dynamic bodies.
// The contact partitioning must be updated whenever the conditions that
// affect CCD eligibility change, so this test stresses that system by
// repeatedly changing those conditions while checking if the changes were
// properly applied based on whether or not tunneling occurred. It does this
// from outside the step, and from the 4 deferred contact listener callbacks.
// (The functions that modify CCD eligibility are locked during the other 4
// immediate contact listener callbacks.)

struct TunnelingCell
{
	void Create(b2World* world, b2Vec2 pos)
	{
		// This wall will separate a dynamic body (ball) from the static body it's welded to.
		// Without TOI the dynamic body can tunnel through the wall.
		{
			b2BodyDef bd;
			bd.type = b2_staticBody;
			bd.position = pos;
			m_wallBody = world->CreateBody(&bd);
			m_wallBody->SetUserData(this);

			b2EdgeShape shape;
			shape.Set(b2Vec2(-1, 0), b2Vec2(1, 0));

			b2FixtureDef fd;
			fd.shape = &shape;

			m_wallFixture = m_wallBody->CreateFixture(&fd);
			m_wallFixture->SetUserData(this);
		}

		// The ball and its target.
		{
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position = pos + b2Vec2(0, 1);
			m_ballBody = world->CreateBody(&bd);
			m_ballBody->SetUserData(this);

			bd.type = b2_staticBody;
			m_ballTargetBody = world->CreateBody(&bd);

			b2CircleShape shape;
			shape.m_radius = 0.75f;

			b2FixtureDef fd;
			fd.shape = &shape;

			m_ballFixture = m_ballBody->CreateFixture(&fd);
			m_ballFixture->SetUserData(this);

			b2WeldJointDef jd;
			jd.Initialize(m_ballBody, m_ballTargetBody, bd.position);

			world->CreateJoint(&jd);

			// Move the target below the wall.
			m_ballTargetBody->SetTransform(pos + b2Vec2(0 , -1), 0);
		}

		m_toiState = 0;
	}

	// This cycles through every possible combination of sensor/bullet/ccd flags
	// of the bodies and fixtures in the contact.
	void AdvanceConfig()
	{
		constexpr uint32 statesPerNode = 8; // 2^3
		constexpr uint32 statesPerContact = statesPerNode * statesPerNode;

		if (++m_toiState == statesPerContact)
		{
			m_toiState = 0;
		}

		uint32 wallMask = m_toiState % statesPerNode;
		uint32 ballMask = m_toiState / statesPerNode;

		constexpr uint32 sensorFlag = 0x1;
		constexpr uint32 thickWallFlag = 0x2;
		constexpr uint32 bulletFlag = 0x4;

		bool isSensor = (wallMask & sensorFlag) == sensorFlag;
		bool isThickShape = (wallMask & thickWallFlag) == thickWallFlag;
		bool isBullet = (wallMask & bulletFlag) == bulletFlag;

		m_wallFixture->SetSensor(isSensor);
		m_wallFixture->SetThickShape(isThickShape);
		m_wallBody->SetBullet(isBullet);

		isSensor = (ballMask & sensorFlag) == sensorFlag;
		isThickShape = (ballMask & thickWallFlag) == thickWallFlag;
		isBullet = (ballMask & bulletFlag) == bulletFlag;

		m_ballFixture->SetSensor(isSensor);
		m_ballFixture->SetThickShape(isThickShape);
		m_ballBody->SetBullet(isBullet);
	}

	// Should the ball reach its target?
	// True if the contact is eligible for CCD and false if it isn't.
	bool ShouldReachTarget()
	{
		if (m_wallFixture->IsSensor() == false && m_ballFixture->IsSensor() == false)
		{
			// Technically a static body can be a bullet, and a fixture on a dynamic body can be a thick shape,
			// which is strange but it doesn't cause any problems.
			if (m_wallBody->IsBullet() || m_ballBody->IsBullet())
			{
				return false;
			}
			else
			{
				b2Assert(m_wallBody->GetType() != b2_dynamicBody);

				if (m_wallFixture->IsThickShape() == false && m_ballFixture->IsThickShape() == false)
				{
					return false;
				}
			}
		}

		return true;
	}

	// Did the ball reach its target?
	bool ReachedTarget() const
	{
		float32 d = b2DistanceSquared(m_ballBody->GetPosition(), m_ballTargetBody->GetPosition());

		return d < 0.05f;
	}

	// Put the ball back in the starting position.
	void ResetBall()
	{
		b2Vec2 p = m_wallBody->GetPosition() + b2Vec2(0, 1);

		m_ballBody->SetTransform(p, 0);
		m_ballBody->SetAwake(true);
	}

	b2Body* m_wallBody;
	b2Fixture* m_wallFixture;
	b2Body* m_ballBody;
	b2Fixture* m_ballFixture;
	b2Body* m_ballTargetBody;
	uint32 m_toiState;
};

class TunnelingTest : public Test
{
public:
	enum
	{
		e_cellCount = 5
	};

	enum CellState
	{
		e_movingToTarget = 0,
		e_stopped,
		e_ready
	};

	TunnelingTest()
		: m_cellStates{}
	{
		constexpr float32 kCellWidth = 2.5f;

		float32 hx = e_cellCount * kCellWidth;

		b2Vec2 pos(-hx + kCellWidth / 2, 0.0f);

		for (uint32 i = 0; i < e_cellCount; ++i)
		{
			m_cells[i].Create(m_world, pos);
			pos.x += kCellWidth;
		}
		m_testPassed = true;
	}

	bool BeginContactImmediate(b2Contact* c, uint32 threadId) override
	{
		B2_NOT_USED(c);
		B2_NOT_USED(threadId);
		return true;
	}
	void BeginContact(b2Contact* c) override
	{
		B2_NOT_USED(c);
		if (m_cellStates[1] == e_stopped)
		{
			UpdateCellStopped(1);
		}
	}
	bool EndContactImmediate(b2Contact* c, uint32 threadId) override
	{
		B2_NOT_USED(c);
		return true;
	}
	void EndContact(b2Contact* c) override
	{
		B2_NOT_USED(c);
		if (m_cellStates[2] == e_stopped)
		{
			UpdateCellStopped(2);
		}
	}
	bool PreSolveImmediate(b2Contact* c, const b2Manifold* oldManifold, uint32 threadId) override
	{
		Test::PreSolveImmediate(c, oldManifold, threadId);
		return true;
	}
	void PreSolve(b2Contact* c, const b2Manifold* oldManifold) override
	{
		B2_NOT_USED(oldManifold);
		if (m_cellStates[3] == e_stopped)
		{
			UpdateCellStopped(3);
		}
	}
	bool PostSolveImmediate(b2Contact* c, const b2ContactImpulse* impulse, uint32 threadId) override
	{
		B2_NOT_USED(c);
		B2_NOT_USED(impulse);
		B2_NOT_USED(threadId);
		return true;
	}
	void PostSolve(b2Contact* c, const b2ContactImpulse* impulse) override
	{
		B2_NOT_USED(impulse);
		if (m_cellStates[4] == e_stopped)
		{
			UpdateCellStopped(4);
		}
	}
	TestResult TestPassed() const override { return m_testPassed ? TestResult::PASS : TestResult::FAIL; }

	void Step(Settings* settings)
	{
		bool doStep = settings->pause == false || settings->singleStep;

		Test::Step(settings);

		if (doStep == false)
		{
			return;
		}

		for (uint32 i = 0; i < e_cellCount; ++i)
		{
			switch(m_cellStates[i])
			{
				case e_movingToTarget:
					UpdateCellMoving(i);
					break;
				case e_stopped:
					// This is handled in the callbacks for cells > 0.
					if (i == 0)
					{
						UpdateCellStopped(0);
					}
					break;
				case e_ready:
					UpdateReady(i);
					break;
			}
		}

		if (m_visible)
		{
			g_debugDraw.DrawString(5, m_textLine, "This cycles through settings that affect TOI and tests whether tunneling occurred as expected.");
			m_textLine += DRAW_STRING_NEW_LINE;
			g_debugDraw.DrawString(5, m_textLine, "Status: %s", m_testPassed ? "PASSING" : "FAILED");
			m_textLine += DRAW_STRING_NEW_LINE;
		}
	}

	void UpdateCellMoving(uint32 cellIndex)
	{
		TunnelingCell& cell = m_cells[cellIndex];

		b2Vec2 v = cell.m_ballBody->GetLinearVelocity();
		float32 d2 = v.LengthSquared();
		if (d2 > b2_epsilon)
		{
			return;
		}

		bool reachedTarget = cell.ReachedTarget();

		if (reachedTarget != cell.ShouldReachTarget())
		{
			m_testPassed = false;
		}

		m_cellStates[cellIndex] = e_stopped;
	}

	void UpdateCellStopped(uint32 cellIndex)
	{
		m_cells[cellIndex].AdvanceConfig();
		m_cellStates[cellIndex] = e_ready;
	}

	void UpdateReady(uint32 cellIndex)
	{
		TunnelingCell& cell = m_cells[cellIndex];

		cell.ResetBall();

		m_cellStates[cellIndex] = e_movingToTarget;
	}

	static Test* Create()
	{
		return new TunnelingTest;
	}

	TunnelingCell m_cells[e_cellCount];
	CellState m_cellStates[e_cellCount];
	bool m_testPassed;
};

#endif
