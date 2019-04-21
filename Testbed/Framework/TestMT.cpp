#include "TestMT.h"
#include <cstdio>

b2Profile ProfileTest(Settings* settings, int32 testIndex)
{
    b2Profile avgProfile{};

    if (settings->mtProfileIterations == 0)
    {
        return avgProfile;
    }

    printf("%s profiling: ", g_testEntries[testIndex].name);
    fflush(stdout);

    int32 stepCount = g_testEntries[testIndex].mtStepCount;
    float32 scale = 1.0f / (settings->mtProfileIterations * stepCount);

    for (int32 testIteration = 0; testIteration < settings->mtProfileIterations; ++testIteration)
    {
        const char* s = testIteration == 0 ? "" : ", ";
        printf("%s%d", s, testIteration + 1);
        fflush(stdout);

        Test* test = g_testEntries[testIndex].createFcn();
        test->SetVisible(false);

        for (int32 i = 0; i < stepCount; ++i)
        {
            test->Step(settings);
        }

        b2Profile totalProfile = test->GetTotalProfile();

        b2AddProfile(avgProfile, totalProfile, scale);

        delete test;
    }

    printf("\n");

    return avgProfile;
}

int32 CheckInconsistent(Settings* settings, int32 testIndex)
{
    int32 inconsistentStep = -1;

    if (settings->mtConsistencyIterations == 0)
    {
        return inconsistentStep;
    }

    printf("%s consistency checks: ", g_testEntries[testIndex].name);
    fflush(stdout);

    for (int32 testIteration = 0; testIteration < settings->mtConsistencyIterations; ++testIteration)
    {
        const char* s = testIteration == 0 ? "" : ", ";
        printf("%s%d", s, testIteration + 1);
        fflush(stdout);

        srand(testIteration);
        Test* testA = g_testEntries[testIndex].createFcn();
        srand(testIteration);
        Test* testB = g_testEntries[testIndex].createFcn();

        b2World* worldA = testA->GetWorld();
        b2World* worldB = testB->GetWorld();

        b2ThreadPoolTaskExecutor* executorA = testA->GetExecutor();
        b2ThreadPoolTaskExecutor* executorB = testB->GetExecutor();

        int32 stepCount = g_testEntries[testIndex].mtStepCount;
        for (int32 i = 0; i < stepCount; ++i)
        {
            int32 seed = (testIteration + 1) * (i + 1);

            srand(seed);
            testA->Step(settings);

            srand(seed);
            testB->Step(settings);

            b2Body* bodyA = worldA->GetBodyList();
            b2Body* bodyB = worldB->GetBodyList();
            while (bodyA)
            {
                if (bodyA->GetPosition() != bodyB->GetPosition() ||
                    bodyA->GetAngle() != bodyB->GetAngle() ||
                    bodyA->IsAwake() != bodyB->IsAwake())
                {
                    inconsistentStep = i;
                    break;
                }

                bodyA = bodyA->GetNext();
                bodyB = bodyB->GetNext();
            }
            if (bodyB != nullptr)
            {
                inconsistentStep = i;
                break;
            }
        }

        if (inconsistentStep != -1)
        {
            printf("  - *** FAILED on step %d ***\n", inconsistentStep);
            break;
        }

        delete testA;
        delete testB;
    }

    printf(" - PASS\n");

    return inconsistentStep;
}

int RunTest(FILE* csv, Settings* settings, int32 testIndex)
{
    b2Profile profile = ProfileTest(settings, testIndex);
    int32 inconsistentStep = CheckInconsistent(settings, testIndex);

    fprintf(csv, "%s, %d, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f\n",
        g_testEntries[testIndex].name,
        inconsistentStep,
        profile.step,
        profile.broadphase,
        profile.broadphaseFindContacts,
        profile.broadphaseSyncFixtures,
        profile.collide,
        profile.solve,
        profile.solveTraversal,
        profile.solveInit,
        profile.solvePosition,
        profile.solveVelocity,
        profile.solveTOI,
        profile.locking);

    if (inconsistentStep == -1)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

void TestMT(Settings* settings, int testIndex)
{
    time_t t = time(0);
    struct tm * now = localtime(&t);

    char filename[80];

    strftime(filename, 80, "mt_test_%Y%m%d%H%M%S.csv", now);
    FILE* csv = fopen(filename, "w");

    fputs("Name, First Inconsistent Step, Step, Broadphase, Broadphase Find Contacts, Broadphase Sync Fixtures, Collide, "
        "Solve, Solve Traversal, Solve Init, Solve Position, Solve Velocity, Solve TOI, Locking\n", csv);

    int inconsistencyCount = 0;

    if (testIndex != -1)
    {
        inconsistencyCount |= RunTest(csv, settings, testIndex);
    }
    else
    {
        for (int32 testIndex = 0; g_testEntries[testIndex].createFcn != nullptr; ++testIndex)
        {
            inconsistencyCount |= RunTest(csv, settings, testIndex);
        }
    }

    fclose(csv);

    printf("Tests finished. See %s for results\n", filename);

    if (settings->mtConsistencyIterations > 0)
    {
        if (inconsistencyCount == 0)
        {
            printf("Success: no inconsistencies found.\n");
        }
        else
        {
            printf("Failure: inconsistencies found in %d tests.\n", inconsistencyCount);
        }
    }
}
