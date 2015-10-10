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

#include <Box2D/Common/b2Threading.h>
#include <Box2D/Common/b2Math.h>
#include <Box2D/Common/b2StackAllocator.h>
#include <algorithm>

using std::thread;
using std::unique_lock;
using std::mutex;

static_assert(b2_maxThreads > 1, "b2_maxThreads must be at least 2 for b2ThreadPool to work.");

const int32 b2_initialPendingTaskCapacity = 256;

// Compare the cost of two tasks.
bool b2TaskLessThan(const b2Task* l, const b2Task* r)
{
	return l->GetCost() < r->GetCost();
}

b2ThreadPool::b2ThreadPool(int32 threadCount)
: m_pendingTasks(b2_initialPendingTaskCapacity)
{
	m_threadCount = 0;

	Restart(threadCount);
}

b2ThreadPool::~b2ThreadPool()
{
	Destroy();
}

void b2ThreadPool::Restart(int32 threadCount)
{
	b2Assert(threadCount <= b2_maxThreadPoolThreads);
	b2Assert(threadCount >= -1);

	if (threadCount == -1)
	{
		// Match the number of cores, minus one for the user thread.
		threadCount = (int32)thread::hardware_concurrency() - 1;
	}

	// Don't exceed the max.
	threadCount = b2Min(threadCount, b2_maxThreadPoolThreads);

	// Account for invalid input or hardware_concurrency not being well defined.
	threadCount = b2Max(threadCount, 0);

	// Terminate currently running threads.
	if (m_threadCount > 0)
	{
		Destroy();
	}

	// Mark the pool as running.
	m_signalShutdown = false;

	// Set the thread count.
	m_threadCount = threadCount;

	// Construct worker threads.
	if (threadCount > 0)
	{
		m_threads = (thread*)b2Alloc(threadCount * sizeof(thread));
		for (int32 i = 0; i < threadCount; ++i)
		{
			new(&m_threads[i]) thread(&b2ThreadPool::WorkerMain, this, 1 + i);
		}
	}
}

int32 b2ThreadPool::GetThreadCount() const
{
	return m_threadCount;
}

void b2ThreadPool::AddTasks(b2Task** tasks, int32 count)
{
	{
		unique_lock<mutex> lk(m_taskMut);
		for (int32 i = 0; i < count; ++i)
		{
			m_pendingTasks.Push(tasks[i]);
			std::push_heap(m_pendingTasks.Data(), m_pendingTasks.Data() + m_pendingTasks.GetCount(), b2TaskLessThan);
		}
	}

	m_taskAddedCond.notify_all();
}

void b2ThreadPool::AddTask(b2Task* task)
{
	{
		unique_lock<mutex> lk(m_taskMut);
		m_pendingTasks.Push(task);
		std::push_heap(m_pendingTasks.Data(), m_pendingTasks.Data() + m_pendingTasks.GetCount(), b2TaskLessThan);
	}

	m_taskAddedCond.notify_one();
}

void b2ThreadPool::Wait(const b2TaskGroup& taskGroup, b2StackAllocator& allocator)
{
	while (taskGroup.m_remainingTasks.load(std::memory_order_acquire) != 0)
	{
		b2Task* task = NULL;

		// Try to execute a task
		{
			std::lock(m_taskMut, m_taskGroupMut);
			std::lock_guard<std::mutex> lk1(m_taskMut, std::adopt_lock);
			std::lock_guard<std::mutex> lk2(m_taskGroupMut, std::adopt_lock);

			// Make sure our group didn't finish between the last check and acquiring the locks.
			if (taskGroup.m_remainingTasks.load(std::memory_order_acquire) == 0)
			{
				return;
			}

			// Consume a task.
			if (m_pendingTasks.GetCount() > 0)
			{
				std::pop_heap(m_pendingTasks.Data(), m_pendingTasks.Data() + m_pendingTasks.GetCount(), b2TaskLessThan);
				task = m_pendingTasks.Pop();
			}
		}

		if (task == NULL)
		{
			// No more tasks to execute.
			unique_lock<mutex> lk(m_taskGroupMut);
			m_taskGroupFinishedCond.wait(lk, [&taskGroup]() -> bool
			{
				return taskGroup.m_remainingTasks.load(std::memory_order_acquire) == 0;
			});

			return;
		}

		// Execute the task.
		task->Execute(allocator);

		// Reduce the count of tasks remaining in the group.
		int32 groupRemainingTasks = task->GetTaskGroup()->m_remainingTasks.fetch_sub(1, std::memory_order_release);

		// If this was the last task in the group.
		if (groupRemainingTasks == 1)
		{
			// Sync with the waiting thread to ensure it reads '0' from the group's remaining tasks count.
			{
				unique_lock<mutex> lk(m_taskGroupMut);
			}

			// Let waiting threads know that a group is finished.
			m_taskGroupFinishedCond.notify_all();
		}
	}
}

void b2ThreadPool::WorkerMain(int32 threadId)
{
	b2SetThreadId(threadId);

	b2StackAllocator allocator;

	for (;;)
	{
		b2Task* task = NULL;

		{
			unique_lock<mutex> lk(m_taskMut);

			// Wait for tasks to be added, or for the pool to shutdown.
			m_taskAddedCond.wait(lk, [this, threadId]() -> bool
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

			// Is the pool shutting down?
			if (m_signalShutdown)
			{
				// Shutting down in the middle of processing tasks is not supported.
				b2Assert(m_pendingTasks.GetCount() == 0);

				return;
			}

			// Consume a task.
			std::pop_heap(m_pendingTasks.Data(), m_pendingTasks.Data() + m_pendingTasks.GetCount(), b2TaskLessThan);
			task = m_pendingTasks.Pop();
		}

		// Execute the task.
		task->Execute(allocator);

		// Reduce the count of tasks remaining in the group.
		int32 groupRemainingTasks = task->GetTaskGroup()->m_remainingTasks.fetch_sub(1, std::memory_order_release);

		// If this was the last task in the group.
		if (groupRemainingTasks == 1)
		{
			// Sync with the waiting thread to ensure it reads '0' from the group's remaining tasks count.
			{
				unique_lock<mutex> lk(m_taskGroupMut);
			}

			// Let waiting threads know that a group is finished.
			m_taskGroupFinishedCond.notify_all();
		}
	}
}

void b2ThreadPool::Destroy()
{
	// Wake up the threads.
	{
		unique_lock<mutex> lk(m_taskMut);
		m_signalShutdown = true;
	}
	m_taskAddedCond.notify_all();

	// Wait for them to finish.
	for (int32 i = 0; i < m_threadCount; ++i)
	{
		if (m_threads[i].joinable())
		{
			m_threads[i].join();
		}
	}

	// Destroy threads.
	for (int32 i = 0; i < m_threadCount; ++i)
	{
		m_threads[i].~thread();
	}
	b2Free(m_threads);
}
