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

#ifndef MANY_BODIES_H
#define MANY_BODIES_H

struct Floater
{
	b2Body* body;
	float32 speed;
	bool isBullet;
};

class UpdateFloaterTask : public b2RangeTask
{
public:
	UpdateFloaterTask() {}
	UpdateFloaterTask(Floater* floaters, uint32 count, float32 dt)
		: b2RangeTask(b2RangeTaskRange(0, count))
		, m_floaters(floaters)
		, m_dt(dt)
	{}

	virtual void Execute(const b2ThreadContext& ctx, const b2RangeTaskRange& range) override
	{
		const float32 kAccelerationTime = 2.0f;
		const float32 kMaxAccelerationScale = 1 / kAccelerationTime;

		for (uint32 i = range.begin; i < range.end; ++i)
		{
			b2Body* body = m_floaters[i].body;
			float32 targetSpeed = m_floaters[i].speed;
			float32 maxAcceleration = targetSpeed;
			if (m_floaters[i].isBullet == false)
			{
				maxAcceleration *= m_dt * kMaxAccelerationScale;
			}

			// Apply an impulse to accelerate toward our target speed in the current direction.
			b2Vec2 velocity = body->GetLinearVelocity();
			float32 speed = velocity.Normalize();
			float32 targetAcceleration = targetSpeed - speed;
			targetAcceleration = b2Clamp(targetAcceleration, -maxAcceleration, maxAcceleration);
			float32 impulseMagnitude = body->GetMass() * targetAcceleration;
			b2Vec2 impulse = impulseMagnitude * velocity;
			body->ApplyLinearImpulseToCenter(impulse, false);
		}
	}

private:
	Floater* m_floaters;
	float32 m_dt;
};

class ManyBodiesImpl : public Test
{
public:
	enum
	{
		e_maxFloaterCount = 50000
	};

	struct Params
	{
		Params()
		{
			borderHalfLength = 2000.0f;
			speedPerRadius = 8.0f;
			thickFloaterThresholdRadius = 1.0f;
			floaterCount = 20000;
			bulletFloaterCount = 0;
			sleeperCount = 0;
			staticBoxCount = 0;
			staticEdgeCount = 0;
			staticSensorCount = 0;
			minStaticHalfExtent = 5.0f;
			maxStaticHalfExtent = 50.0f;
			subTreeWidth = 500.0f;
			thickWalls = true;
		}
		float32 borderHalfLength;
		float32 speedPerRadius;
		float32 thickFloaterThresholdRadius;
		uint32 floaterCount;
		uint32 bulletFloaterCount;
		uint32 sleeperCount;
		uint32 staticBoxCount;
		uint32 staticEdgeCount;
		uint32 staticSensorCount;
		float32 minStaticHalfExtent;
		float32 maxStaticHalfExtent;
		float32 subTreeWidth;
		bool thickWalls;
	};

	ManyBodiesImpl(const Params& params)
		: m_params(params)
	{
		// Splitting the broad-phase AABB tree into smaller sub-trees improves tree quality
		// when there are many fixtures (better FindNewContacts performance) and improves
		// parallelism of AABB updates (better SynchronizeFixtures performance).
#ifdef b2_dynamicTreeOfTrees
		m_world->SetSubTreeSize(params.subTreeWidth, params.subTreeWidth);
#endif

		const float32 kBorderHalfLength = m_params.borderHalfLength;
		const float32 kBorderHalfWidth = 5.0f;

		// For consistent profiling
		srand(0);

		// Borders
		{
			b2PolygonShape shape;

			b2FixtureDef fd;
			fd.shape = &shape;
			fd.thickShape = m_params.thickWalls;

			shape.SetAsBox(kBorderHalfLength, kBorderHalfWidth, b2Vec2(0, kBorderHalfLength), 0.0f);
			m_groundBody->CreateFixture(&fd);

			shape.SetAsBox(kBorderHalfLength, kBorderHalfWidth, b2Vec2(0, -kBorderHalfLength), 0.0f);
			m_groundBody->CreateFixture(&fd);

			shape.SetAsBox(kBorderHalfWidth, kBorderHalfLength, b2Vec2(kBorderHalfLength, 0), 0.0f);
			m_groundBody->CreateFixture(&fd);

			shape.SetAsBox(kBorderHalfWidth, kBorderHalfLength, b2Vec2(-kBorderHalfLength, 0), 0.0f);
			m_groundBody->CreateFixture(&fd);
		}

		// Static fixtures
		{
			const float32 kMinHalfExtent = m_params.minStaticHalfExtent;
			const float32 kMaxHalfExtent = m_params.maxStaticHalfExtent;
			const float32 kPositionRange = kBorderHalfLength - kBorderHalfWidth - kMaxHalfExtent;

			b2PolygonShape shape;

			b2FixtureDef fd;
			fd.shape = &shape;
			fd.thickShape = m_params.thickWalls;

			for (int32 i = 0; i < m_params.staticBoxCount; ++i)
			{
				float32 hx = RandomFloat(kMinHalfExtent, kMaxHalfExtent);
				float32 hy = RandomFloat(kMinHalfExtent, kMaxHalfExtent);
				float32 x = RandomFloat(-kPositionRange, kPositionRange);
				float32 y = RandomFloat(-kPositionRange, kPositionRange);
				float32 a = RandomFloat(0, 2 * b2_pi);

				shape.SetAsBox(hx, hy, b2Vec2(x, y), a);
				m_groundBody->CreateFixture(&fd);
			}
			fd.thickShape = false;

			fd.isSensor = true;
			for (int32 i = 0; i < m_params.staticSensorCount; ++i)
			{
				float32 hx = kMaxHalfExtent;
				float32 hy = kMaxHalfExtent;
				float32 x = RandomFloat(-kPositionRange, kPositionRange);
				float32 y = RandomFloat(-kPositionRange, kPositionRange);

				shape.SetAsBox(hx, hy, b2Vec2(x, y), 0.0f);
				m_groundBody->CreateFixture(&fd);
			}
			fd.isSensor = false;

			b2EdgeShape edgeShape;

			for (uint32 i = 0; i < m_params.staticEdgeCount; ++i)
			{
				float32 hx = RandomFloat(kMinHalfExtent, kMaxHalfExtent);
				float32 x = RandomFloat(-kPositionRange, kPositionRange);
				float32 y = RandomFloat(-kPositionRange, kPositionRange);
				float32 a = RandomFloat(0, 2 * b2_pi);

				b2Transform xf(b2Vec2(x, y), b2Rot(a));
				b2Vec2 v0 = b2Mul(xf, b2Vec2(-hx, 0.0f));
				b2Vec2 v1 = b2Mul(xf, b2Vec2(hx, 0.0f));

				edgeShape.Set(v0, v1);

				m_groundBody->CreateFixture(&edgeShape, 0.0f);
			}
		}

		// Floaters
		{
			const float32 kMinRadius = 0.5f;
			const float32 kMaxRadius = 5.0f;
			const float32 kSpeedPerRadius = m_params.speedPerRadius;
			const float32 kBulletSpeed = 120.0f;
			const float32 kPositionRange = kBorderHalfLength - kBorderHalfWidth;

			b2BodyDef bd;
			bd.type = b2_dynamicBody;

			b2FixtureDef fd;

			b2PolygonShape polygon;
			b2CircleShape circle;

			b2Shape::Type shapeTypes[] = { b2Shape::e_circle, b2Shape::e_polygon };
			const uint32 kShapeCount = sizeof(shapeTypes) / sizeof(shapeTypes[0]);

			uint32 bodyCount = m_params.floaterCount + m_params.sleeperCount;

			for (int32 i = 0; i < bodyCount; ++i)
			{
				float32 radius = RandomFloat(kMinRadius, kMaxRadius);
				float32 speed = kSpeedPerRadius * radius;
				float32 x = RandomFloat(-kPositionRange, kPositionRange);
				float32 y = RandomFloat(-kPositionRange, kPositionRange);
				float32 a = RandomFloat(0, 2 * b2_pi);

				fd.density = 1.0f;

				bd.bullet = false;
				if (i < m_params.bulletFloaterCount)
				{
					speed = kBulletSpeed;
					radius = kMinRadius;
					bd.bullet = true;
					fd.density = 25.0f;
				}

				bd.position = b2Vec2(x, y);
				bd.angle = a;
				bd.angularDamping = 0.25f;

				if (i < m_params.floaterCount)
				{
					b2Vec2 n = b2Vec2(RandomFloat(), RandomFloat());
					n.Normalize();
					bd.linearVelocity = speed * n;
					bd.linearDamping = 0.0f;
				}
				else
				{
					// We don't control the sleeper bodies.
					bd.linearVelocity = b2Vec2_zero;
					bd.linearDamping = 0.5f;
					fd.density = 5.0f;
				}

				b2Body* body = m_world->CreateBody(&bd);

				uint32 shapeIndex = i % kShapeCount;
				b2Shape::Type shapeType = shapeTypes[shapeIndex];

				switch(shapeType)
				{
				case b2Shape::e_circle:
					{
						circle.m_radius = radius;
						fd.shape = &circle;
					}
					break;
				case b2Shape::e_polygon:
					{
						b2Vec2 vertices[b2_maxPolygonVertices];
						uint32 verticesCount = b2Clamp(i % b2_maxPolygonVertices, 3, b2_maxPolygonVertices);

						float32 arc = 2 * b2_pi / verticesCount;

						for (uint32 v = 0; v < verticesCount; ++v)
						{
							float32 a = (v + 1.0f) * arc;
							b2Transform xf(b2Vec2_zero, b2Rot(a));

							vertices[v] = b2Mul(xf, b2Vec2(radius, 0.0f));
						}

						polygon.Set(vertices, verticesCount);
						fd.shape = &polygon;
					}
					break;
				}

				if (radius > m_params.thickFloaterThresholdRadius)
				{
					// If the shape is thick enough then we don't need TOI, even against static edge shapes.
					fd.thickShape = true;
				}

				body->CreateFixture(&fd);

				if (i < m_params.floaterCount)
				{
					m_floaters[i].body = body;
					m_floaters[i].speed = speed;
					m_floaters[i].isBullet = bd.bullet;
				}
			}
		}

		m_world->SetGravity(b2Vec2_zero);
	}

	void Step(Settings* settings)
	{
		bool doStep = settings->pause == false || settings->singleStep;

		Test::Step(settings);

		if (doStep == false)
		{
			return;
		}

		UpdateFloaterTask updateFloatersTask(m_floaters, m_params.floaterCount, m_timeStep);
		b2ExecuteRangeTask(m_threadPoolExec, updateFloatersTask);
	}

	Params m_params;
	Floater m_floaters[e_maxFloaterCount];
};

// Primarily pressures FindNewContacts.
struct ManyBodies1
{
	static Test* Create()
	{
		ManyBodiesImpl::Params params;
		params.borderHalfLength = 4000.0f;
		params.floaterCount = 10000;
		params.sleeperCount = 40000;
		params.staticBoxCount = 2000;
		params.minStaticHalfExtent = 5.0f;
		params.maxStaticHalfExtent = 50.0f;
		params.subTreeWidth = 500.0f;
		return new ManyBodiesImpl(params);
	}
};

// Primarily pressures FindNewContacts.
struct ManyBodies2
{
	static Test* Create()
	{
		ManyBodiesImpl::Params params;
		params.borderHalfLength = 2000.0f;
		params.floaterCount = 10000;
		params.bulletFloaterCount = 1000;
		params.sleeperCount = 20000;
		params.staticBoxCount = 250;
		params.staticEdgeCount = 250;
		params.minStaticHalfExtent = 5.0f;
		params.maxStaticHalfExtent = 50.0f;
		params.subTreeWidth = 400.0f;
		return new ManyBodiesImpl(params);
	}
};

// Primarily pressures SynchronizeFixtures.
struct ManyBodies3
{
	static Test* Create()
	{
		ManyBodiesImpl::Params params;
		params.borderHalfLength = 4000.0f;
		params.floaterCount = 20000;
		params.speedPerRadius = 20.0f;
		params.subTreeWidth = 500.0f;
		return new ManyBodiesImpl(params);
	}
};

// Primarily pressures island traversal.
struct ManyBodies4
{
	static Test* Create()
	{
		ManyBodiesImpl::Params params;
		params.borderHalfLength = 1000.0f;
		params.floaterCount = 20000;
		params.staticSensorCount = 20;
		params.maxStaticHalfExtent = 200.0f;
		params.subTreeWidth = 125.0f;
		return new ManyBodiesImpl(params);
	}
};

// Primarily pressures SolveTOI.
struct ManyBodies5
{
	static Test* Create()
	{
		ManyBodiesImpl::Params params;
		params.borderHalfLength = 1000.0f;
		params.floaterCount = 10000;
		params.bulletFloaterCount = 1000;
		params.staticEdgeCount = 100;
		params.minStaticHalfExtent = 50.0f;
		params.maxStaticHalfExtent = 200.0f;
		params.subTreeWidth = 250.0f;
		params.thickFloaterThresholdRadius = 5.0f;
		return new ManyBodiesImpl(params);
	}
};

// A reduced size test that can run in acceptable time with drd.
struct ManyBodies6
{
	static Test* Create()
	{
		ManyBodiesImpl::Params params;
		params.borderHalfLength = 500.0f;
		params.floaterCount = 2000;
		params.bulletFloaterCount = 500;
		params.sleeperCount = 0;
		params.staticBoxCount = 25;
		params.staticEdgeCount = 25;
		params.minStaticHalfExtent = 5.0f;
		params.maxStaticHalfExtent = 50.0f;
		params.subTreeWidth = 250.0f;
		params.thickFloaterThresholdRadius = 5.0f;
		return new ManyBodiesImpl(params);
	}
};


#endif
