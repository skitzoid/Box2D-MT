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

#ifndef B2_THREADING_H
#define B2_THREADING_H


#include <Box2D/Common/b2Settings.h>
#include <Box2D/Common/b2GrowableArray.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

class b2TaskGroup;
class b2StackAllocator;
class b2ThreadPool;

/// The base class for all tasks that are run by the thread pool.
class b2Task
{
public:
	/// Construct a task.
	b2Task();

	// Set the estimated cost of executing the task.
	// Higher cost tasks can start execution before lower cost tasks.
	void SetCost(int32 costEstimate);

	// Get the estimated cost of executing the task.
	int32 GetCost() const;

	/// Get the task group that submitted the task to the thread pool.
	/// NULL if the task hasn't been submitted.
	b2TaskGroup* GetTaskGroup() const;

	/// Execute the task. Called by a worker thread after the
	/// task has been submitted to the thread pool.
	virtual void Execute(b2StackAllocator& threadStack) = 0;

private:
	friend class b2TaskGroup;

	// How long will this task take to execute?
	int32 m_costEstimate;

	/// The task group that submitted the task to the thread pool.
	b2TaskGroup* m_taskGroup;
};

/// The base class for tasks that operate on a range of items.
class b2RangedTask : public b2Task
{
public:
	// Construct a ranged task.
	b2RangedTask();

protected:
	friend class b2TaskGroup;

	int32 m_beginIndex;
	int32 m_endIndex;
};

/// A task group is used to submit tasks to the thread pool.
class b2TaskGroup
{
public:
	/// Construct a task group.
	b2TaskGroup(b2ThreadPool& threadPool);

	~b2TaskGroup();

	/// Submit tasks for execution.
	void SubmitTasks(b2Task** tasks, int32 count);

	/// Submit a single task for execution.
	void SubmitTask(b2Task* task);

	/// Initialize the indices of ranged tasks and submit them for execution.
	template <typename RangedTaskType>
	void SubmitRangedTasks(RangedTaskType* tasks, int32 taskCount, int32 elementCount, b2StackAllocator& allocator);

	/// Wait for all tasks in the group to finish. The
	/// allocator is used to execute tasks while waiting.
	void Wait(b2StackAllocator& allocator);

private:
	friend class b2ThreadPool;

	std::atomic<uint32> m_remainingTasks;
	b2ThreadPool* m_threadPool;
};

/// The thread pool executes tasks submitted by task groups.
class b2ThreadPool
{
public:
	/// Construct a thread pool.
	/// @param threadCount the number of threads to use. If -1, defaults to the number of logical cores - 1.
	b2ThreadPool(int32 threadCount = -1);

	~b2ThreadPool();

	/// Get the number of threads in the pool.
	int32 GetThreadCount() const;

private:
	friend class b2TaskGroup;

	// Add multiple tasks to be executed.
	void AddTasks(b2Task** tasks, int32 count);

	// Add a single task to be executed.
	void AddTask(b2Task* task);

	// Wait for all tasks in the group to finish. The
	// allocator is used to execute tasks while waiting.
	void Wait(const b2TaskGroup& taskGroup, b2StackAllocator& allocator);

	void WorkerMain(int32 threadId);

	void Destroy();

	std::mutex m_taskMut;
	std::condition_variable m_taskAddedCond;
	std::mutex m_taskGroupMut;
	std::condition_variable m_taskGroupFinishedCond;
	b2GrowableArray<b2Task*> m_pendingTasks;

	std::thread* m_threads;
	int32 m_threadCount;

	bool m_signalShutdown;
};

inline b2Task::b2Task()
{
	m_taskGroup = NULL;
}

inline void b2Task::SetCost(int32 costEstimate)
{
	m_costEstimate = costEstimate;
}

inline int32 b2Task::GetCost() const
{
	return m_costEstimate;
}

inline b2TaskGroup* b2Task::GetTaskGroup() const
{
	return m_taskGroup;
}

inline b2RangedTask::b2RangedTask()
{
	m_beginIndex = 0;
	m_endIndex = 0;
}

template <typename RangedTaskType>
void b2TaskGroup::SubmitRangedTasks(RangedTaskType* tasks, int32 taskCount, int32 elementCount, b2StackAllocator& allocator)
{
	b2Assert(taskCount > 0);

	if (elementCount == 0)
	{
		return;
	}

	b2Task** taskPtrs = (b2Task**)allocator.Allocate(taskCount * sizeof(b2Task*));
	int32 taskPtrCount = 0;

	int32 elementsPerTask = elementCount / taskCount;
	int32 elementsRemainder = elementCount % taskCount;

	int32 beginIndex = 0;
	int32 endIndex = 0;
	for (int32 i = 0; i < taskCount; ++i)
	{
		int32 count = elementsPerTask;
		if (i < elementsRemainder)
		{
			++count;
		}
		endIndex = beginIndex + count;
		if (endIndex > elementCount)
		{
			endIndex = elementCount;
		}
		tasks[taskPtrCount].m_beginIndex = beginIndex;
		tasks[taskPtrCount].m_endIndex = endIndex;
		taskPtrs[taskPtrCount] = tasks + taskPtrCount;
		++taskPtrCount;
		if (endIndex == elementCount)
		{
			break;
		}
		beginIndex = endIndex;
	}

	SubmitTasks(taskPtrs, taskPtrCount);

	allocator.Free(taskPtrs);
}

inline b2TaskGroup::b2TaskGroup(b2ThreadPool& threadPool)
{
	m_remainingTasks.store(0, std::memory_order_relaxed);
	m_threadPool = &threadPool;
}

inline b2TaskGroup::~b2TaskGroup()
{
	// If any tasks were submitted, Wait must be called before the task group is destroyed.
	b2Assert(m_remainingTasks == 0);
}

inline void b2TaskGroup::SubmitTasks(b2Task** tasks, int32 count)
{
	m_remainingTasks.fetch_add(count, std::memory_order_relaxed);

	for (int32 i = 0; i < count; ++i)
	{
		tasks[i]->m_taskGroup = this;
	}

	m_threadPool->AddTasks(tasks, count);
}

inline void b2TaskGroup::SubmitTask(b2Task* task)
{
	m_remainingTasks.fetch_add(1, std::memory_order_relaxed);

	task->m_taskGroup = this;

	m_threadPool->AddTask(task);
}

inline void b2TaskGroup::Wait(b2StackAllocator& allocator)
{
	m_threadPool->Wait(*this, allocator);
}

#endif
