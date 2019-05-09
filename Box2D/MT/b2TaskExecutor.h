/*
* Copyright (c) 2019 Justin Hoffman https://github.com/jhoffman0x/Box2D-MT
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

#ifndef B2_TASK_EXECUTOR_H
#define B2_TASK_EXECUTOR_H

#include "Box2D/MT/b2Task.h"

class b2World;

/// The base class for task executors.
class b2TaskExecutor
{
public:

	/// The total number of threads that can be executed.
	/// Must be between 1 and b2_maxThreads.
	virtual uint32 GetThreadCount() const = 0;

	/// Called when a world's step begins.
	virtual void StepBegin(b2World& world)
	{
		B2_NOT_USED(world);
	}

	/// Called when a world's step ends.
	virtual void StepEnd(b2World& world)
	{
		B2_NOT_USED(world);
	}

	/// Create a task group.
	/// The allocator can provide storage for the task group if needed.
	virtual b2TaskGroup CreateTaskGroup(b2StackAllocator& allocator)
	{
		B2_NOT_USED(allocator);
		b2TaskGroup taskGroup(nullptr);
		return taskGroup;
	}

	/// Destroy the task group, freeing any allocations made by CreateTaskGroup.
	virtual void DestroyTaskGroup(b2TaskGroup taskGroup, b2StackAllocator& allocator)
	{
		B2_NOT_USED(taskGroup);
		B2_NOT_USED(allocator);
	}

	/// Partition a range into sub-ranges that will each be assigned to a range task.
	/// Implementing this is optional (you could instead divide ranges during task submission).
	virtual void PartitionRange(b2TaskType type, uint32 begin, uint32 end, b2PartitionedRange& output)
	{
		B2_NOT_USED(type);

		output[0] = b2RangeTaskRange(begin, end);
		output.count = 1;
	}

	/// Submit a single task for execution.
	virtual void SubmitTask(b2TaskGroup taskGroup, b2Task* task)
	{
		B2_NOT_USED(taskGroup);
		B2_NOT_USED(task);
	}

	/// Submit multiple tasks for execution.
	virtual void SubmitTasks(b2TaskGroup taskGroup, b2Task** tasks, uint32 count)
	{
		for (uint32 i = 0; i < count; ++i)
		{
			SubmitTask(taskGroup, tasks[i]);
		}
	}

	/// Wait for all tasks in the group to finish.
	virtual void Wait(b2TaskGroup taskGroup, const b2ThreadContext& ctx)
	{
		B2_NOT_USED(taskGroup);
		B2_NOT_USED(ctx);
	}
};

#endif
