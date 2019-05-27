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

#ifndef TREE_OF_TREES_MISSING_PROXY_TEST_H
#define TREE_OF_TREES_MISSING_PROXY_TEST_H

struct RayCastCounter : public b2RayCastCallback
{
    RayCastCounter()
    {
        count = 0;
    }

    float32 ReportFixture(b2Fixture* fixture, const b2Vec2& point,
                          const b2Vec2& normal, float32 fraction) override
    {
        ++count;

        return fraction;
    }

    uint32 count;
};

// Regression test for a bug where b2DynamicTreeOfTrees could fail to create
// a sub-proxy in an overlapped sub-tree.
class TreeOfTreesMissingProxyTest : public Test
{
public:
    TreeOfTreesMissingProxyTest()
    {
#ifdef b2_dynamicTreeOfTrees
        m_world->SetSubTreeSize(10.0f, 10.0f);
#endif

        // This creates new sub-trees for the bottom-left and top-right edges.
        {
            b2EdgeShape shape;
            shape.Set(b2Vec2(-2.0f, 4.0f), b2Vec2(4.0f, -2.0f));
            m_groundBody->CreateFixture(&shape, 0.0f);

            shape.Set(b2Vec2(6.0f, 12.0f), b2Vec2(12.0f, 6.0f));
            m_groundBody->CreateFixture(&shape, 0.0f);
        }

        // This should create new sub-trees for the top-left and bottom-right edges.
        // It was failing to happen due to a bug in b2DynamicTreeOfTrees.
        {
            b2EdgeShape shape;
            shape.Set(b2Vec2(-2.0f, 4.0f), b2Vec2(6.0f, 12.0f));
            m_groundBody->CreateFixture(&shape, 0.0f);

            shape.Set(b2Vec2(12.0f, 6.0f), b2Vec2(4.0f, -2.0f));
            m_groundBody->CreateFixture(&shape, 0.0f);
        }

        // Ensure that we can detect the proxies in the top-left and bottom-right sub-trees.
        RayCastCounter rayCallback;

        {
            b2Vec2 p1(0.0f, 10.0f);
            b2Vec2 p2(4.0f, 6.0f);
            m_world->RayCast(&rayCallback, p1, p2);
        }

        {
            b2Vec2 p1(10.0f, 0.0f);
            b2Vec2 p2(6.0f, 4.0f);
            m_world->RayCast(&rayCallback, p1, p2);
        }

        m_passed = rayCallback.count == 2;
    }

    void Step(Settings* settings)
    {
        Test::Step(settings);

        {
            b2Vec2 p1(0.0f, 10.0f);
            b2Vec2 p2(4.0f, 6.0f);
            g_debugDraw.DrawSegment(p1, p2, b2Color(1.0f, 1.0f, 1.0f));
        }

        {
            b2Vec2 p1(10.0f, 0.0f);
            b2Vec2 p2(6.0f, 4.0f);
            g_debugDraw.DrawSegment(p1, p2, b2Color(1.0f, 1.0f, 1.0f));
        }

        g_debugDraw.DrawString(5, m_textLine, "This is a regression test for missing sub-proxies in b2DynamicTreeOfTrees.");
        m_textLine += DRAW_STRING_NEW_LINE;
        g_debugDraw.DrawString(5, m_textLine, "Status: %s", m_passed ? "PASSED" : "FAILED");
        m_textLine += DRAW_STRING_NEW_LINE;
    }

    static Test* Create()
    {
        return new TreeOfTreesMissingProxyTest;
    }

    TestResult TestPassed() const override { return m_passed ? TestResult::PASS : TestResult::FAIL; }

    bool m_passed;
};

#endif
