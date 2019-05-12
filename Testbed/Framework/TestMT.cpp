#include "TestMT.h"
#include <cstdio>

static TestResult ProfileTest(Settings* settings, int testIndex, b2Profile* profile)
{
    TestResult testResult = TestResult::NONE;

    if (settings->mtProfileIterations == 0)
    {
        return testResult;
    }

    printf("%s profiling: ", g_testEntries[testIndex].name);
    fflush(stdout);

    int stepCount = g_testEntries[testIndex].mtStepCount;
    float32 scale = 1.0f / (settings->mtProfileIterations * stepCount);

    for (int testIteration = 0; testIteration < settings->mtProfileIterations; ++testIteration)
    {
		// For consistent profiling
        srand(0);

        const char* s = testIteration == 0 ? "" : ", ";
        printf("%s%d", s, testIteration + 1);
        fflush(stdout);

        Test* test = g_testEntries[testIndex].createFcn();
        test->SetVisible(false);

        for (int i = 0; i < stepCount; ++i)
        {
            test->Step(settings);
        }

        testResult &= test->TestPassed();

        b2Profile totalProfile = test->GetTotalProfile();

        b2AddProfile(*profile, totalProfile, scale);

        delete test;
    }

    printf("\n");

    return testResult;
}

static TestResult CheckInconsistent(Settings* settings, int testIndex, int* inconsistentStep)
{
    *inconsistentStep = -1;
    TestResult testResult = TestResult::NONE;

    if (settings->mtConsistencyIterations == 0)
    {
        return testResult;
    }

    printf("%s consistency checks: ", g_testEntries[testIndex].name);
    fflush(stdout);

    for (int testIteration = 0; testIteration < settings->mtConsistencyIterations; ++testIteration)
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

        int stepCount = g_testEntries[testIndex].mtStepCount;
        for (int i = 0; i < stepCount; ++i)
        {
            int seed = (testIteration + 1) * (i + 1);

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
                    *inconsistentStep = i;
                    break;
                }

                bodyA = bodyA->GetNext();
                bodyB = bodyB->GetNext();
            }
            if (bodyB != nullptr)
            {
                *inconsistentStep = i;
                break;
            }
        }

        testResult &= testA->TestPassed();
        testResult &= testB->TestPassed();

        delete testA;
        delete testB;

        if (*inconsistentStep != -1)
        {
            printf("  - *** FAILURE on step %d ***\n", *inconsistentStep);
            break;
        }
    }

    if (*inconsistentStep == -1)
    {
        printf(" - PASS\n");
    }

    return testResult;
}

static void RunTest(FILE* csv, Settings* settings, int testIndex, int* inconsistencyCount, int* failCount)
{
    b2Profile profile{};
    TestResult testResult = ProfileTest(settings, testIndex, &profile);
    int inconsistentStep;
    testResult &= CheckInconsistent(settings, testIndex, &inconsistentStep);

    if (testResult == TestResult::FAIL)
    {
        printf("%s - *** TEST FAILED ***\n", g_testEntries[testIndex].name);
    }

    fprintf(csv, "%s, %s, %d, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f\n",
        g_testEntries[testIndex].name,
        TestResultString(testResult),
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
        profile.solveTOIComputeTOI,
        profile.locking);

    if (inconsistentStep != -1)
    {
        *inconsistencyCount += 1;
    }

    if (testResult == TestResult::FAIL)
    {
        *failCount += 1;
    }
}

void TestMT(Settings* settings, int testIndex)
{
    time_t t = time(0);
    struct tm * now = localtime(&t);

    char filename[80];

    strftime(filename, 80, "mt_test_%Y%m%d%H%M%S.csv", now);
    FILE* csv = fopen(filename, "w");

    fputs("Name, Test Result, Inconsistent Index, Step, Broadphase, Broadphase Find Contacts, Broadphase Sync Fixtures, Collide, "
        "Solve, Solve Traversal, Solve Init, Solve Position, Solve Velocity, Solve TOI, Compute TOIs, Locking\n", csv);

    int inconsistencyCount = 0;
    int failCount = 0;

    if (testIndex != -1)
    {
        RunTest(csv, settings, testIndex, &inconsistencyCount, &failCount);
    }
    else
    {
        for (int testIndex = 0; g_testEntries[testIndex].createFcn != nullptr; ++testIndex)
        {
            RunTest(csv, settings, testIndex, &inconsistencyCount, &failCount);
        }
    }

    fclose(csv);

    printf("----------------------------------------------------------------\n");

    printf("Tests finished. See %s for details\n", filename);

    if (failCount == 0)
    {
        printf("Test result: Success - all tests passed\n");
    }
    else
    {
        printf("Test result: *** FAILURE *** - %d tests failed\n", failCount);
    }

    if (settings->mtConsistencyIterations > 0)
    {
        if (inconsistencyCount == 0)
        {
            printf("Consistency result: Success - no inconsistencies found\n");
        }
        else
        {
            printf("Consistency result: *** FAILURE *** - inconsistencies found in %d tests\n", inconsistencyCount);
        }
    }

    printf("----------------------------------------------------------------\n");
}
