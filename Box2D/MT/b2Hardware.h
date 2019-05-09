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

#ifndef B2_HARDWARE_H
#define B2_HARDWARE_H

#include "Box2D/Common/b2Settings.h"

#ifdef b2_hwloc
#include "hwloc.h"
#include <mutex>

/// Use hwloc to interact with the hardware.
class b2Hardware
{
public:
    /// Set the affinity of the current thread.
    /// @param threadIndex the index of the current thread.
    /// @param relaxed allow the thread to move between logical cores
    /// on the assigned physical core.
    static void SetThreadAffinity(uint32 threadIndex, bool relaxed);

private:
    static b2Hardware& _get();

    b2Hardware();
    ~b2Hardware();

    void _initThreadAffinities();
    void _setThreadAffinity(uint32 threadIndex, bool useCoreAffinity);

    std::mutex m_mutex;
    hwloc_cpuset_t m_coreAffinities[b2_maxThreads];
    hwloc_cpuset_t m_puAffinities[b2_maxThreads];
    hwloc_topology_t m_topology;
};
#else
// Dummy hardware class.
class b2Hardware
{
public:
    static void SetThreadAffinity(uint32 threadIndex, bool relaxed)
    {
        B2_NOT_USED(threadIndex);
        B2_NOT_USED(relaxed);
    }
};
#endif // #ifdef b2_hwloc
#endif // #ifndef B2_HARDWARE_H
