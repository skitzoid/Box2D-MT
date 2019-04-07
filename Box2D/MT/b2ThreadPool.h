/*
* Copyright (c) 2019 Justin Hoffman https://github.com/skitzoid/Box2D-MT
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
#pragma once

#include "Box2D/Common/b2GrowableArray.h"
#include "Box2D/Common/b2Timer.h"
#include "Box2D/MT/b2TaskExecutor.h"
#include "Box2D/MT/b2Threading.h"
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

class b2ThreadPool;

/// A task group is used to wait for completion of a group of tasks.
class b2ThreadPoolTaskGroup
{
public:
	explicit b2ThreadPoolTaskGroup(b2ThreadPool& threadPool);

	~b2ThreadPoolTaskGroup();

private:
	friend class b2ThreadPool;

	b2ThreadPool* m_threadPool;
	std::atomic<uint32> m_remainingTasks;

	// Stats
	b2Timer m_notifyTimer;
	float32 m_maxNotifyAllMilliseconds;
	float32 m_accumulatedNotifyMilliseconds;
	bool m_notifiedAll;
};

/// The thread pool manages worker threads that execute tasks.
class b2ThreadPool
{
public:
	/// Construct a thread pool.
	/// @param threadCount the number of threads to make available for execution. This
	/// includes the user thread, so the thread pool will allocate [threadCount - 1] threads.
	/// -1 is interpreted as the number of logical cores.
	explicit b2ThreadPool(int32 threadCount = -1);

	~b2ThreadPool();

	/// Wake the workers so they can busy-wait for tasks until StopBusyWaiting is called.
	void StartBusyWaiting();

	/// Allow the workers to sleep until tasks are added or StartBusyWaiting is called.
	void StopBusyWaiting();

	/// Submit a single task for execution.
	/// Returns immediately after submission.
	void SubmitTask(b2ThreadPoolTaskGroup& group, b2Task* task);

	/// Submit multiple tasks for execution.
	/// Returns immediately after submission.
	void SubmitTasks(b2ThreadPoolTaskGroup& group, b2Task** tasks, int32 count);

	/// Wait for all tasks in the group to finish.
	/// The allocator is used to execute tasks while waiting.
	void Wait(const b2ThreadPoolTaskGroup& group, b2StackAllocator& allocator);

	/// The number of threads available to execute tasks. This is the number of threads in the pool,
	/// plus one for the additional thread required to submit tasks and wait on them.
	int32 GetThreadCount() const;

	/// Time spent waiting to lock the mutex.
	float32 GetLockMilliseconds() const;

	/// Time until waiting workers acquired a task after notification.
	float32 GetTaskStartMilliseconds() const;

	/// Reset lock timer and task start timer.
	void ResetTimers();

private:
	friend class b2TaskGroup;

	void WorkerMain(int32 threadId);

	std::mutex m_mutex;
	std::condition_variable m_workerCond;
	std::atomic<int32> m_pendingTaskCount;
	std::atomic<bool> m_busyWait;
	b2GrowableArray<b2Task*> m_pendingTasks;

	std::thread* m_threads;
	int32 m_threadCount;

	float32 m_lockMilliseconds;
	float32 m_taskStartMilliseconds;

	bool m_signalShutdown;
};

/// A task executor that uses b2ThreadPool.
class b2ThreadPoolTaskExecutor : public b2TaskExecutor
{
public:
	/// Construct a thread pool task executor.
	/// @param threadCount the number of threads to make available for execution. This
	/// includes the user thread, so the thread pool will allocate [threadCount - 1] threads.
	/// -1 is interpreted as the number of logical cores.
	b2ThreadPoolTaskExecutor(int32 threadCount = -1);

	/// Control whether worker threads busy-wait even after a step ends.
	/// Default is true.
	void SetContinuousBusyWait(bool flag);

	/// Set the target number of tasks that a range task will be partitioned into.
	void SetTargetRangeTaskCount(int32 value);

	/// Get the thread pool.
	b2ThreadPool& GetThreadPool();

	/// Called when the step begins.
	void StepBegin() override;

	/// Called when the step ends.
	void StepEnd(b2Profile& profile) override;

	/// Create a task group.
	/// The provided allocator can provide storage for the task group if desired.
	void* CreateTaskGroup(b2StackAllocator& allocator) override;

	/// Destroy the task group, freeing any allocations made in CreateTaskGroup.
	void DestroyTaskGroup(void* userTaskGroup, b2StackAllocator& allocator) override;

	/// Partition a range into sub-ranges that will each be assigned to a range task.
	b2PartitionedRange PartitionRange(const b2RangeTaskRange& sourceRange) override;

	/// Submit a single task for execution.
	void SubmitTask(void* userTaskGroup, b2Task* task) override;

	/// Submit range tasks for execution.
	void SubmitRangeTasks(void* userTaskGroup, b2Task** rangeTasks, int32 count) override;

	/// Wait for all tasks in the group to finish.
	void Wait(void* userTaskGroup, b2StackAllocator& allocator) override;

private:
	b2ThreadPool m_threadPool;
	int32 m_targetRangeTaskCount;
	bool m_continuousBusyWait;
};

inline b2ThreadPoolTaskGroup::b2ThreadPoolTaskGroup(b2ThreadPool& threadPool)
{
	m_threadPool = &threadPool;
	m_remainingTasks = 0;
	m_maxNotifyAllMilliseconds = 0;
	m_accumulatedNotifyMilliseconds = 0;
	m_notifiedAll = false;
}

inline b2ThreadPoolTaskGroup::~b2ThreadPoolTaskGroup()
{
	// If any tasks were submitted, Wait must be called before the task group is destroyed.
	b2Assert(m_remainingTasks == 0);
}

inline int32 b2ThreadPool::GetThreadCount() const
{
	return m_threadCount + 1;
}

inline float32 b2ThreadPool::GetLockMilliseconds() const
{
	return m_lockMilliseconds;
}

inline float32 b2ThreadPool::GetTaskStartMilliseconds() const
{
	return m_taskStartMilliseconds;
}

inline void b2ThreadPool::ResetTimers()
{
	m_lockMilliseconds = 0;
	m_taskStartMilliseconds = 0;
}

inline b2ThreadPoolTaskExecutor::b2ThreadPoolTaskExecutor(int32 threadCount)
	: m_threadPool(threadCount)
	, m_continuousBusyWait(true)
{
	m_targetRangeTaskCount = 2 * m_threadPool.GetThreadCount();
}

inline void b2ThreadPoolTaskExecutor::SetContinuousBusyWait(bool flag)
{
	m_continuousBusyWait = flag;
}

inline void b2ThreadPoolTaskExecutor::SetTargetRangeTaskCount(int32 value)
{
	m_targetRangeTaskCount = value;
}

inline b2ThreadPool& b2ThreadPoolTaskExecutor::GetThreadPool()
{
	return m_threadPool;
}

