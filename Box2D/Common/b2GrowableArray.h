/*
* Copyright (c) 2015, Justin Hoffman https://github.com/skitzoid
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

#ifndef B2_GROWABLE_ARRAY_H
#define B2_GROWABLE_ARRAY_H
#include "Box2D/Common/b2Settings.h"
#include <cstring>

/// This is a growable array, meant for internal use only.
template <typename T>
class b2GrowableArray
{
public:
	b2GrowableArray(int32 startCapacityHint = 1)
	{
		m_capacity = startCapacityHint > 0 ? startCapacityHint : 1;
		m_count = 0;

		m_array = (T*)b2Alloc(m_capacity * sizeof(T));
	}

	~b2GrowableArray()
	{
		b2Free(m_array);
		m_array = nullptr;
	}

	void Push(const T& element)
	{
		if (m_count == m_capacity)
		{
			m_capacity *= 2;
			T* old = m_array;
			m_array = (T*)b2Alloc(m_capacity * sizeof(T));
			memcpy(m_array, old, m_count * sizeof(T));
			b2Free(old);
		}

		m_array[m_count] = element;
		++m_count;
	}

	T Pop()
	{
		b2Assert(m_count > 0);
		--m_count;
		return m_array[m_count];
	}

	T& Peek() const
	{
		b2Assert(m_count > 0);
		return m_array[m_count - 1];
	}

	void Clear()
	{
		m_count = 0;
	}

	int32 GetCount() const
	{
		return m_count;
	}

	void RemoveAndSwap(int32 index)
	{
		m_array[index] = m_array[--m_count];
	}

	T& At(size_t i)
	{
		return m_array[i];
	}

	const T& At(size_t i) const
	{
		return m_array[i];
	}

	T* Data()
	{
		return m_array;
	}

	const T* Data() const
	{
		return m_array;
	}

private:
	T* m_array;
	int32 m_count;
	int32 m_capacity;
};

#endif
