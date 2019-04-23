/*
* Copyright (c) 2015 Justin Hoffman https://github.com/jhoffman0x/Box2D-MT
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

void b2PartitionRange(uint32 begin, uint32 end, uint32 targetOutputCount, b2PartitionedRange& output)
{
	b2Assert(targetOutputCount <= b2_partitionRangeMaxOutput);
	b2Assert(begin < end);

	uint32 elementCount = end - begin;

	uint32 elementsPerTask = elementCount / targetOutputCount;
	uint32 elementsRemainder = elementCount % targetOutputCount;

	uint32 beginIndex = begin;
	uint32 endIndex = 0;
	for (uint32 i = 0; i < targetOutputCount; ++i)
	{
		uint32 rangeSize = elementsPerTask;
		if (i < elementsRemainder)
		{
			rangeSize += 1;
		}
		endIndex = beginIndex + rangeSize;
		if (endIndex > elementCount)
		{
			endIndex = elementCount;
		}
		output[output.count].begin = beginIndex;
		output[output.count].end = endIndex;
		output.count += 1;
		if (endIndex == elementCount)
		{
			break;
		}
		beginIndex = endIndex;
	}
}
