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

#include "Box2D/MT/b2ThreadPool.h"
#include "Box2D/Common/b2Math.h"
#include "Box2D/Common/b2StackAllocator.h"
#include "Box2D/Dynamics/b2TimeStep.h"
#include <algorithm>

// Compare the cost of two tasks.
inline bool b2TaskCostLessThan(const b2Task* l, const b2Task* r)
{
	return l->GetCost() < r->GetCost();
}

b2ThreadPool::b2ThreadPool(int32 threadCount)
{
	b2Assert(threadCount <= b2_maxThreadPoolThreads);
	b2Assert(threadCount >= -1);

	if (threadCount == -1)
	{
		// Match the number of cores, minus one for the user thread.
		threadCount = (int32)std::thread::hardware_concurrency() - 1;
	}

	// Account for invalid input, single core processors, or hardware_concurrency not being well defined.
	threadCount = b2Clamp(threadCount, 0, b2_maxThreadPoolThreads);

	m_signalShutdown = false;
	m_threadCount = threadCount;

	if (threadCount > 0)
	{
		m_threads = (std::thread*)b2Alloc(threadCount * sizeof(std::thread));
		for (int32 i = 0; i < threadCount; ++i)
		{
			new(&m_threads[i]) std::thread(&b2ThreadPool::WorkerMain, this, 1 + i);
		}
	}
	else
	{
		m_threads = nullptr;
	}

	m_lockMilliseconds = 0;
	m_taskStartMilliseconds = 0;
	m_pendingTaskCount.store(0, std::memory_order_relaxed);
	m_busyWait.store(false, std::memory_order_relaxed);
}

b2ThreadPool::~b2ThreadPool()
{
	{
		std::lock_guard<std::mutex> lk(m_mutex);
		m_signalShutdown = true;
		m_busyWait.store(false, std::memory_order_relaxed);
	}
	m_workerCond.notify_all();

	for (int32 i = 0; i < m_threadCount; ++i)
	{
		if (m_threads[i].joinable())
		{
			m_threads[i].join();
		}
	}

	for (int32 i = 0; i < m_threadCount; ++i)
	{
		m_threads[i].~thread();
	}
	b2Free(m_threads);
}

void b2ThreadPool::StartBusyWaiting()
{
	{
		std::lock_guard<std::mutex> lk(m_mutex);
		m_busyWait.store(true, std::memory_order_relaxed);
	}
	m_workerCond.notify_all();
}

void b2ThreadPool::StopBusyWaiting()
{
	std::lock_guard<std::mutex> lk(m_mutex);
	m_busyWait.store(false, std::memory_order_relaxed);
}

void b2ThreadPool::SubmitTasks(b2ThreadPoolTaskGroup& group, b2Task** tasks, int32 count)
{
	{
		b2Timer lockTimer;
		std::lock_guard<std::mutex> lk(m_mutex);
		m_lockMilliseconds += lockTimer.GetMilliseconds();

		for (int32 i = 0; i < count; ++i)
		{
			m_pendingTasks.Push(tasks[i]);
			std::push_heap(m_pendingTasks.Data(), m_pendingTasks.Data() + m_pendingTasks.GetCount(), b2TaskCostLessThan);
		}
		m_pendingTaskCount.store(m_pendingTasks.GetCount(), std::memory_order_relaxed);
		group.m_remainingTasks.store(group.m_remainingTasks.load(std::memory_order_relaxed) + count, std::memory_order_relaxed);

		group.m_accumulatedNotifyMilliseconds += group.m_maxNotifyAllMilliseconds;
		group.m_maxNotifyAllMilliseconds = 0;
		group.m_notifiedAll = true;
		group.m_notifyTimer.Reset();
	}
	m_workerCond.notify_all();
}

void b2ThreadPool::SubmitTask(b2ThreadPoolTaskGroup& group, b2Task* task)
{
	{
		b2Timer lockTimer;
		std::lock_guard<std::mutex> lk(m_mutex);
		m_lockMilliseconds += lockTimer.GetMilliseconds();

		m_pendingTasks.Push(task);
		std::push_heap(m_pendingTasks.Data(), m_pendingTasks.Data() + m_pendingTasks.GetCount(), b2TaskCostLessThan);
		m_pendingTaskCount.store(m_pendingTasks.GetCount(), std::memory_order_relaxed);
		group.m_remainingTasks.store(group.m_remainingTasks.load(std::memory_order_relaxed) + 1, std::memory_order_relaxed);

		group.m_maxNotifyAllMilliseconds = 0;
		group.m_notifiedAll = false;
		group.m_notifyTimer.Reset();
	}
	m_workerCond.notify_one();
}

void b2ThreadPool::Wait(const b2ThreadPoolTaskGroup& group, b2StackAllocator& allocator)
{
    b2TaskExecutionContext context;
    context.allocator = &allocator;
    context.threadId = 0;

	b2Timer lockTimer;
	std::unique_lock<std::mutex> lk(m_mutex);
	m_lockMilliseconds += lockTimer.GetMilliseconds();

	while (true)
	{
		if (group.m_remainingTasks.load(std::memory_order_relaxed) == 0)
		{
			m_taskStartMilliseconds += group.m_maxNotifyAllMilliseconds + group.m_accumulatedNotifyMilliseconds;
			return;
		}

		if (m_pendingTasks.GetCount() == 0)
		{
			// Wait for workers to finish executing the group.
			lk.unlock();
			while (group.m_remainingTasks.load(std::memory_order_relaxed) > 0)
			{
				std::this_thread::yield();
			}
			lockTimer.Reset();
			lk.lock();
			m_lockMilliseconds += lockTimer.GetMilliseconds();

			m_taskStartMilliseconds += group.m_maxNotifyAllMilliseconds + group.m_accumulatedNotifyMilliseconds;
			return;
		}

		// Execute a task while waiting.
		std::pop_heap(m_pendingTasks.Data(), m_pendingTasks.Data() + m_pendingTasks.GetCount(), b2TaskCostLessThan);
		b2Task* task = m_pendingTasks.Pop();
		m_pendingTaskCount.store(m_pendingTasks.GetCount(), std::memory_order_relaxed);

		lk.unlock();

		task->Execute(context);

		lockTimer.Reset();
		lk.lock();
		m_lockMilliseconds += lockTimer.GetMilliseconds();

		// This is not necessarily the group we're waiting on.
		b2ThreadPoolTaskGroup* executeGroup = (b2ThreadPoolTaskGroup*)task->GetTaskGroup();

		// We only modify this while the mutex is locked, so it's okay to do this non-atomically.
		executeGroup->m_remainingTasks.store(executeGroup->m_remainingTasks.load(std::memory_order_relaxed) - 1, std::memory_order_relaxed);
	}
}

void b2ThreadPool::WorkerMain(int32 threadId)
{
	b2SetThreadId(threadId);

	b2StackAllocator allocator;

    b2TaskExecutionContext context;
    context.allocator = &allocator;
    context.threadId = threadId;

	b2Timer lockTimer;
	std::unique_lock<std::mutex> lk(m_mutex);

	while (true)
	{
		bool waitedForTasks = false;

		while (m_pendingTasks.GetCount() == 0)
		{
			// Wait for tasks
			waitedForTasks = true;

			if (m_busyWait.load(std::memory_order_relaxed))
			{
				lk.unlock();
				while (m_pendingTaskCount.load(std::memory_order_relaxed) == 0 &&
					m_busyWait.load(std::memory_order_relaxed))
				{
					std::this_thread::yield();
				}
				lockTimer.Reset();
				lk.lock();
				m_lockMilliseconds += lockTimer.GetMilliseconds();
				// Note: the pending task count will be checked again now that we have the lock,
				// and we'll go back to waiting if there's no longer any pending tasks.
			}
			else
			{
				m_workerCond.wait(lk, [this]()
				{
					if (m_busyWait.load(std::memory_order_relaxed))
					{
						return true;
					}
					if (m_pendingTasks.GetCount() > 0)
					{
						return true;
					}
					if (m_signalShutdown)
					{
						return true;
					}
					return false;
				});
			}

			if (m_signalShutdown)
			{
				// Shutting down in the middle of processing tasks is not supported.
				b2Assert(m_pendingTasks.GetCount() == 0);

				return;
			}
		}

		std::pop_heap(m_pendingTasks.Data(), m_pendingTasks.Data() + m_pendingTasks.GetCount(), b2TaskCostLessThan);
		b2Task* task = m_pendingTasks.Pop();
		m_pendingTaskCount.store(m_pendingTasks.GetCount(), std::memory_order_relaxed);

		b2ThreadPoolTaskGroup* group = (b2ThreadPoolTaskGroup*)task->GetTaskGroup();

		// Measure how long it took for threads to get to work after notification.
		// Busy waiting reduces this time.
		if (waitedForTasks)
		{
			if (group->m_notifiedAll)
			{
				group->m_maxNotifyAllMilliseconds = group->m_notifyTimer.GetMilliseconds();
			}
			else
			{
				group->m_accumulatedNotifyMilliseconds += group->m_notifyTimer.GetMilliseconds();
			}
		}

		lk.unlock();

		task->Execute(context);

		lockTimer.Reset();
		lk.lock();
		m_lockMilliseconds += lockTimer.GetMilliseconds();

		// We only modify this while the mutex is locked, so it's okay to do this non-atomically.
		group->m_remainingTasks.store(group->m_remainingTasks.load(std::memory_order_relaxed) - 1, std::memory_order_relaxed);
	}
}

void b2ThreadPoolTaskExecutor::StepBegin()
{
	m_threadPool.StartBusyWaiting();
	m_threadPool.ResetTimers();
}

void b2ThreadPoolTaskExecutor::StepEnd(b2Profile& profile)
{
	profile.locking = m_threadPool.GetLockMilliseconds();
	profile.taskStarting = m_threadPool.GetTaskStartMilliseconds();

	if (m_continuousBusyWait == false)
	{
		m_threadPool.StopBusyWaiting();
	}
}

void* b2ThreadPoolTaskExecutor::CreateTaskGroup(b2StackAllocator& allocator)
{
	b2ThreadPoolTaskGroup* taskGroup = (b2ThreadPoolTaskGroup*)allocator.Allocate(sizeof(b2ThreadPoolTaskGroup));
	new(taskGroup) b2ThreadPoolTaskGroup(m_threadPool);
	return taskGroup;
}

void b2ThreadPoolTaskExecutor::DestroyTaskGroup(void* userTaskGroup, b2StackAllocator& allocator)
{
	b2ThreadPoolTaskGroup* taskGroup = (b2ThreadPoolTaskGroup*)userTaskGroup;
	taskGroup->~b2ThreadPoolTaskGroup();
	allocator.Free(taskGroup);
}

b2PartitionedRange b2ThreadPoolTaskExecutor::PartitionRange(const b2RangeTaskRange& sourceRange)
{
	b2PartitionedRange output = b2PartitionRange(sourceRange, m_targetRangeTaskCount, b2_rangeTaskMinPartitionSize);

	return output;
}

void b2ThreadPoolTaskExecutor::SubmitTask(void* userTaskGroup, b2Task* task)
{
	b2ThreadPoolTaskGroup* taskGroup = (b2ThreadPoolTaskGroup*)userTaskGroup;

	m_threadPool.SubmitTask(*taskGroup, task);
}

void b2ThreadPoolTaskExecutor::SubmitRangeTasks(void* userTaskGroup, b2Task** rangeTasks, int32 count)
{
	b2ThreadPoolTaskGroup* taskGroup = (b2ThreadPoolTaskGroup*)userTaskGroup;

	m_threadPool.SubmitTasks(*taskGroup, rangeTasks, count);
}

void b2ThreadPoolTaskExecutor::Wait(void* userTaskGroup, b2StackAllocator& allocator)
{
	b2ThreadPoolTaskGroup* taskGroup = (b2ThreadPoolTaskGroup*)userTaskGroup;

	m_threadPool.Wait(*taskGroup, allocator);
}