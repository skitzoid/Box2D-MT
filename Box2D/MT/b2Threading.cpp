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

#include "Box2D/Dynamics/b2TimeStep.h"
#include "Box2D/MT/b2Threading.h"

thread_local int32 b2_threadId = 0;

void b2SetThreadId(int32 threadId)
{
	b2_threadId = threadId;
}

int32 b2GetThreadId()
{
	return b2_threadId;
}

b2PartitionedRange b2PartitionRange(const b2RangeTaskRange& sourceRange, int32 targetOutputCount, int32 minRangeSize)
{
	b2Assert(targetOutputCount > 0);
	b2Assert(targetOutputCount <= b2_partitionedRangeTasksCapacity);

	b2PartitionedRange output;

	int32 elementCount = sourceRange.end - sourceRange.begin;

	int32 elementsPerTask = elementCount / targetOutputCount;
	if (elementsPerTask < minRangeSize)
	{
		targetOutputCount = b2Max(1, elementCount / minRangeSize);
		elementsPerTask = elementCount / targetOutputCount;
	}
	int32 elementsRemainder = elementCount % targetOutputCount;

	int32 beginIndex = sourceRange.begin;
	int32 endIndex = 0;
	for (int32 i = 0; i < targetOutputCount; ++i)
	{
		int32 rangeSize = elementsPerTask;
		if (i < elementsRemainder)
		{
			rangeSize += 1;
		}
		endIndex = beginIndex + rangeSize;
		if (endIndex > elementCount)
		{
			endIndex = elementCount;
		}
		output.ranges[output.count].begin = beginIndex;
		output.ranges[output.count].end = endIndex;
		output.count += 1;
		if (endIndex == elementCount)
		{
			break;
		}
		beginIndex = endIndex;
	}

	return output;
}

