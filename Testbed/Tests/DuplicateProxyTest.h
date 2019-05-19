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

#ifndef QUERY_TEST_H
#define QUERY_TEST_H

class QueryTest : public Test
{
public:
	enum
	{
		e_boxcount = 2800
	};

	static Test* Create()
	{
		return new QueryTest;
	}

	QueryTest()
	{
#ifdef b2_dynamicTreeOfTrees
		m_world->SetSubTreeSize(10.0f, 10.0f);
#endif

		m_passed = true;

		b2PolygonShape shape;

		bool fixtureHit[3] = { };

		for (int32 i = 0; i < 3; ++i)
		{
			shape.SetAsBox(i * 10.0f + 10.0f, 10.0f - i);

			b2Fixture* f = m_groundBody->CreateFixture(&shape, 1.0f);
			f->SetUserData(fixtureHit + i);
		}

		struct QueryCallback : public b2QueryCallback
		{
			QueryCallback()
				: m_hitCount(0)
				, m_failed(false)
			{ }

			bool ReportFixture(b2Fixture* f)
			{
				++m_hitCount;

				bool* hit = (bool*)f->GetUserData();
				if (*hit)
				{
					m_failed = true;
				}
				*hit = true;

				return true;
			}

			int32 m_hitCount;
			bool m_failed;
		};

		int32 queryHitCounts[8] = { 3, 3, 3, 3, 3, 2, 1, 0 };

		for (int32 i = 0; i < 8; ++i)
		{
			QueryCallback queryCb;

			b2AABB aabb;
			aabb.lowerBound.x = (i - 4) * 10.2f;
			aabb.lowerBound.y = -10.0f;
			aabb.upperBound.x = 40.0f;
			aabb.upperBound.y = 10.0f;

			m_world->QueryAABB(&queryCb, aabb);
			if (queryCb.m_failed || queryHitCounts[i] != queryCb.m_hitCount)
			{
				m_passed = false;
			}
			memset(fixtureHit, 0, sizeof(fixtureHit));
		}

		struct RayCallback : public b2RayCastCallback
		{
			RayCallback()
				: m_hitCount(0)
				, m_failed(false)
			{ }

			float32 ReportFixture(b2Fixture* fixture, const b2Vec2& point, const b2Vec2& normal, float32 fraction) override
			{
				++m_hitCount;

				bool* hit = (bool*)fixture->GetUserData();
				if (*hit)
				{
					m_failed = true;
				}
				*hit = true;

				return -1.0f;
			}

			int32 m_hitCount;
			bool m_failed;
		};

		int32 rayHitCounts[8] = { 3, 3, 2, 1, 0, 0, 0, 0 };

		for (int32 i = 0; i < 8; ++i)
		{
			RayCallback rayCb;

			b2Vec2 p1((i - 4) * 10.2f, 0.0f);
			b2Vec2 p2(40.0f, 0.0f);

			m_world->RayCast(&rayCb, p1, p2);
			if (rayCb.m_failed || rayHitCounts[i] != rayCb.m_hitCount)
			{
				m_passed = false;
			}
			memset(fixtureHit, 0, sizeof(fixtureHit));
		}
	}

	void Step(Settings* settings)
	{
		Test::Step(settings);

#if 0
		int32 drawIndex = 0;

		b2AABB aabb;
		aabb.lowerBound.x = (drawIndex - 4) * 10.2f;
		aabb.lowerBound.y = -10.0f;
		aabb.upperBound.x = 40.0f;
		aabb.upperBound.y = 10.0f;
		g_debugDraw.DrawAABB(&aabb, b2Color(0.7f, 0.7f, 0.7f));

		b2Vec2 p1((drawIndex - 4) * 10.2f, 0.0f);
		b2Vec2 p2(40.0f, 0.0f);
		g_debugDraw.DrawSegment(p1, p2, b2Color(0.7f, 0.7f, 0.7f));
#endif

		g_debugDraw.DrawString(5, m_textLine, "This checks if broad-phase queries correctly handle proxies split across sub-trees.");
		m_textLine += DRAW_STRING_NEW_LINE;
		g_debugDraw.DrawString(5, m_textLine, "Status: %s", m_passed ? "PASSED" : "FAILED");
		m_textLine += DRAW_STRING_NEW_LINE;
	}

	TestResult TestPassed() const override { return m_passed ? TestResult::PASS : TestResult::FAIL; }

	bool m_passed;
};

#endif
