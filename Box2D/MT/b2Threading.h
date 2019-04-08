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
#pragma once

#include "Box2D/Common/b2Settings.h"

class b2StackAllocator;

/// Thread specific data given to a task for exeuction.
struct b2TaskExecutionContext
{
	b2StackAllocator* allocator;
	int32 threadId;
};

/// The base class for all tasks that are run by the thread pool.
class b2Task
{
public:
	/// Construct a task.
	b2Task();

	/// Execute the task.
	virtual void Execute(b2TaskExecutionContext& context) = 0;

	virtual ~b2Task() {}

	/// Set the estimated cost of executing the task so the executor can prioritize higher cost tasks.
	void SetCost(uint32 costEstimate);

	/// Get the estimated cost of executing the task.
	int32 GetCost() const;

	/// Associate this task with a task group.
	void SetTaskGroup(void* taskGroup);

	/// Get the group that this task is associated with.
	void* GetTaskGroup() const;

private:
	uint32 m_costEstimate;
	void* m_taskGroup;
};

/// A range over which a range task executes.
struct b2RangeTaskRange
{
	b2RangeTaskRange() {}

	b2RangeTaskRange(uint32 beginIn, uint32 endIn) : begin(beginIn), end(endIn) {}

	uint32 begin;
	uint32 end;
};

/// A set of sequential ranges.
struct b2PartitionedRange
{
	b2PartitionedRange() : count(0) {}

	b2RangeTaskRange ranges[b2_partitionedRangeTasksCapacity];
	uint32 count;
};

/// The base class for tasks that operate on a range of items.
class b2RangeTask : public b2Task
{
public:
	/// Construct a range task.
	b2RangeTask();

	/// Execute the task over the specified range.
	virtual void Execute(b2TaskExecutionContext& context, const b2RangeTaskRange& range) = 0;

	const b2RangeTaskRange& GetRange() const;

protected:
	b2RangeTaskRange m_range;
};

/// Set the calling thread's ID.
/// Must be in the range [0, b2_maxThreads) and unique for each of an executors concurrently executing threads.
void b2SetThreadId(int32 threadId);

/// Get the calling thread's ID.
int32 b2GetThreadId();

/// Evenly divide the source range into the target number of ranges.
/// The size of the output ranges will be at least minRangeSize.
/// The difference between the size of any two output ranges will be 0 or 1.
b2PartitionedRange b2PartitionRange(const b2RangeTaskRange& sourceRange, int32 targetOutputCount, int32 minRangeSize);

inline b2Task::b2Task()
{
	m_costEstimate = 0;
	m_taskGroup = nullptr;
}

inline void b2Task::SetCost(uint32 costEstimate)
{
	m_costEstimate = costEstimate;
}

inline int32 b2Task::GetCost() const
{
	return m_costEstimate;
}

inline void b2Task::SetTaskGroup(void* data)
{
	m_taskGroup = data;
}

inline void* b2Task::GetTaskGroup() const
{
	return m_taskGroup;
}

inline b2RangeTask::b2RangeTask()
	: m_range{}
{

}

inline const b2RangeTaskRange& b2RangeTask::GetRange() const
{
	return m_range;
}
