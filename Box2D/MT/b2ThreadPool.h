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
	explicit b2ThreadPoolTaskGroup(b2ThreadPool& threadPool, uint32 priority);

	~b2ThreadPoolTaskGroup();

	uint32 GetPriority() const;

private:
	friend class b2ThreadPool;

	b2ThreadPool* m_threadPool;
	std::atomic<uint32> m_remainingTasks;
	uint32 m_priority;
};

/// The thread pool manages worker threads that execute tasks.
class b2ThreadPool
{
public:
	/// Construct a thread pool.
	/// @param totalThreadCount the number of threads to make available for execution. This
	/// includes the user thread, so the thread pool will allocate [threadCount - 1] threads.
	/// -1 is interpreted as the number of logical cores.
	explicit b2ThreadPool(int32 totalThreadCount = -1);

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
	void SubmitTasks(b2ThreadPoolTaskGroup& group, b2Task** tasks, uint32 count);

	/// Wait for all tasks in the group to finish.
	/// The allocator is used to execute tasks while waiting.
	void Wait(const b2ThreadPoolTaskGroup& group, const b2ThreadContext& ctx);

	/// The number of threads available to execute tasks. This is the number of threads in the pool,
	/// plus one for the additional thread required to submit tasks and wait on them.
	int32 GetThreadCount() const;

	/// Time spent waiting to lock the mutex.
	float32 GetLockMilliseconds() const;

	/// Reset lock timer and task start timer.
	void ResetTimers();

	/// Restart with the specified number of threads
	void Restart(int32 threadCount);

private:
	friend class b2TaskGroup;

	void WorkerMain(uint32 threadId);
	void Shutdown();

	std::mutex m_mutex;
	std::condition_variable m_workerCond;
	std::atomic<int32> m_pendingTaskCount;
	std::atomic<bool> m_busyWait;
	b2GrowableArray<b2Task*> m_pendingTasks;

	std::thread m_threads[b2_maxThreadPoolThreads];
	int32 m_threadCount;

	float32 m_lockMilliseconds;

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
	explicit b2ThreadPoolTaskExecutor(int32 threadCount = -1);

	/// Control whether worker threads busy-wait even after a step ends.
	/// Default is true.
	void SetContinuousBusyWait(bool flag);

	/// Set the target number of tasks that a range task will be partitioned into.
	void SetTargetRangeTaskCount(int32 value);

	/// Get the thread pool.
	b2ThreadPool& GetThreadPool();
	const b2ThreadPool& GetThreadPool() const;

	/// Get the number of threads available for execution.
	uint32 GetThreadCount() const override;

	/// Called when the step begins.
	void StepBegin() override;

	/// Called when the step ends.
	void StepEnd(b2Profile& profile) override;

	/// Create a task group.
	/// The allocator can provide storage for the task group if needed.
	/// Note: the id value will be unique for each active task and less than
	/// b2_maxConcurrentTaskGroups. Tasks with lower ids will be waited on
	/// first, so it can be interpreted as a priority.
	b2TaskGroup CreateTaskGroup(b2StackAllocator& allocator, uint32 id) override;

	/// Destroy the task group, freeing any allocations made in CreateTaskGroup.
	void DestroyTaskGroup(b2TaskGroup taskGroup, b2StackAllocator& allocator) override;

	/// Partition a range into sub-ranges that will each be assigned to a range task.
	void PartitionRange(uint32 begin, uint32 end, b2PartitionedRange& output) override;

	/// Submit a single task for execution.
	void SubmitTask(b2TaskGroup taskGroup, b2Task* task) override;

	/// Submit multiple tasks for execution.
	void SubmitTasks(b2TaskGroup taskGroup, b2Task** tasks, uint32 count) override;

	/// Wait for all tasks in the group to finish.
	void Wait(b2TaskGroup taskGroup, const b2ThreadContext& ctx) override;

private:
	b2ThreadPool m_threadPool;
	bool m_continuousBusyWait;
};

inline b2ThreadPoolTaskGroup::~b2ThreadPoolTaskGroup()
{
	// If any tasks were submitted, Wait must be called before the task group is destroyed.
	b2Assert(m_remainingTasks.load(std::memory_order_relaxed) == 0);
}

inline uint32 b2ThreadPoolTaskGroup::GetPriority() const
{
	return m_priority;
}

inline int32 b2ThreadPool::GetThreadCount() const
{
	return m_threadCount + 1;
}

inline float32 b2ThreadPool::GetLockMilliseconds() const
{
	return m_lockMilliseconds;
}

inline void b2ThreadPool::ResetTimers()
{
	m_lockMilliseconds = 0;
}

inline b2ThreadPoolTaskExecutor::b2ThreadPoolTaskExecutor(int32 threadCount)
	: m_threadPool(threadCount)
	, m_continuousBusyWait(false)
{

}

inline void b2ThreadPoolTaskExecutor::SetContinuousBusyWait(bool flag)
{
	m_continuousBusyWait = flag;
}

inline b2ThreadPool& b2ThreadPoolTaskExecutor::GetThreadPool()
{
	return m_threadPool;
}

inline const b2ThreadPool& b2ThreadPoolTaskExecutor::GetThreadPool() const
{
	return m_threadPool;
}

inline uint32 b2ThreadPoolTaskExecutor::GetThreadCount() const
{
	return m_threadPool.GetThreadCount();
}

