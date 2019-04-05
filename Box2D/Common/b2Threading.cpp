/*
* Copyright (c) 2015 Justin Hoffman https://github.com/skitzoid/Box2D-MT
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

#include "Box2D/Common/b2Threading.h"
#include "Box2D/Common/b2Math.h"
#include "Box2D/Common/b2StackAllocator.h"
#include <algorithm>

// Compare the cost of two tasks.
bool b2TaskLessThan(const b2Task* l, const b2Task* r)
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
}

b2ThreadPool::~b2ThreadPool()
{
	{
		std::lock_guard<std::mutex> lk(m_mutex);
		m_signalShutdown = true;
	}
	m_taskAddedCond.notify_all();

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

int32 b2ThreadPool::GetThreadCount() const
{
	return m_threadCount;
}

void b2ThreadPool::AddTasks(b2TaskGroup* group, b2Task** tasks, int32 count)
{
	{
		std::lock_guard<std::mutex> lk(m_mutex);
		group->m_remainingTasks += count;
		for (int32 i = 0; i < count; ++i)
		{
			m_pendingTasks.Push(tasks[i]);
			std::push_heap(m_pendingTasks.Data(), m_pendingTasks.Data() + m_pendingTasks.GetCount(), b2TaskLessThan);
		}
	}
	m_taskAddedCond.notify_all();
}

void b2ThreadPool::AddTask(b2TaskGroup* group, b2Task* task)
{
	{
		std::lock_guard<std::mutex> lk(m_mutex);
		group->m_remainingTasks += 1;
		m_pendingTasks.Push(task);
		std::push_heap(m_pendingTasks.Data(), m_pendingTasks.Data() + m_pendingTasks.GetCount(), b2TaskLessThan);
	}
	m_taskAddedCond.notify_one();
}

void b2ThreadPool::Wait(const b2TaskGroup& taskGroup, b2StackAllocator& allocator)
{
	std::unique_lock<std::mutex> lk(m_mutex);

	for (;;)
	{
		if (taskGroup.m_remainingTasks == 0)
		{
			return;
		}

		if (m_pendingTasks.GetCount() == 0)
		{
			// Wait for workers to finish executing the group.
			m_taskGroupFinishedCond.wait(lk, [&taskGroup]() -> bool
			{
				return taskGroup.m_remainingTasks == 0;
			});

			return;
		}

		// Execute a task while waiting.
		std::pop_heap(m_pendingTasks.Data(), m_pendingTasks.Data() + m_pendingTasks.GetCount(), b2TaskLessThan);
		b2Task* task = m_pendingTasks.Pop();

		lk.unlock();

		task->Execute(allocator);

		lk.lock();

		b2TaskGroup* group = task->GetTaskGroup();
		group->m_remainingTasks -= 1;
	}
}

void b2ThreadPool::WorkerMain(int32 threadId)
{
	b2SetThreadId(threadId);

	b2StackAllocator allocator;

	std::unique_lock<std::mutex> lk(m_mutex);

	for (;;)
	{
		if (m_pendingTasks.GetCount() == 0)
		{
			// Wait for tasks to be added, or for the pool to shutdown.
			m_taskAddedCond.wait(lk, [this]()
			{
				if (this->m_signalShutdown)
				{
					return true;
				}
				if (m_pendingTasks.GetCount() > 0)
				{
					return true;
				}
				return false;
			});

			if (m_signalShutdown)
			{
				// Shutting down in the middle of processing tasks is not supported.
				b2Assert(m_pendingTasks.GetCount() == 0);

				return;
			}
		}

		std::pop_heap(m_pendingTasks.Data(), m_pendingTasks.Data() + m_pendingTasks.GetCount(), b2TaskLessThan);
		b2Task* task = m_pendingTasks.Pop();

		lk.unlock();

		task->Execute(allocator);

		lk.lock();

		b2TaskGroup* group = task->GetTaskGroup();
		if (--group->m_remainingTasks == 0)
		{
			m_taskGroupFinishedCond.notify_all();
		}
	}
}
