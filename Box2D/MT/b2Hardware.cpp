#include "Box2D/MT/b2Hardware.h"

#ifdef b2_hwloc

void b2Hardware::SetThreadAffinity(uint32 threadIndex, bool relaxed)
{
    _get()._setThreadAffinity(threadIndex, relaxed);
}

b2Hardware& b2Hardware::_get()
{
    static b2Hardware s_instance;
    return s_instance;
}

b2Hardware::b2Hardware()
{
    hwloc_topology_init(&m_topology);
    hwloc_topology_set_all_types_filter(m_topology, HWLOC_TYPE_FILTER_KEEP_NONE);
    hwloc_topology_set_type_filter(m_topology, HWLOC_OBJ_CORE, HWLOC_TYPE_FILTER_KEEP_ALL);
    hwloc_topology_set_type_filter(m_topology, HWLOC_OBJ_PU, HWLOC_TYPE_FILTER_KEEP_ALL);
    hwloc_topology_load(m_topology);

    _initThreadAffinities();
}

b2Hardware::~b2Hardware()
{
    hwloc_topology_destroy(m_topology);
}

// Assign a logical core (hwloc processing unit) to each thread index.
void b2Hardware::_initThreadAffinities()
{
    uint32 numCores = hwloc_get_nbobjs_by_type(m_topology, HWLOC_OBJ_CORE);
    if (numCores > b2_maxThreads)
    {
        numCores = b2_maxThreads;
    }

    uint32 puThreadIndex = 0;
    uint32 coreThreadIndex = 0;

    for (; coreThreadIndex < numCores; ++coreThreadIndex)
    {
        hwloc_obj_t core = hwloc_get_obj_by_type(m_topology, HWLOC_OBJ_CORE, coreThreadIndex);
        if (core == nullptr)
        {
            continue;
        }

        m_coreAffinities[coreThreadIndex] = core->cpuset;

        uint32 puCount = hwloc_get_nbobjs_inside_cpuset_by_type(m_topology, core->cpuset, HWLOC_OBJ_PU);
        for (uint32 puIndex = 0; puIndex < puCount; ++puIndex)
        {
            hwloc_obj_t pu = hwloc_get_obj_inside_cpuset_by_type(m_topology, core->cpuset, HWLOC_OBJ_PU, puIndex);
            if (pu == nullptr)
            {
                continue;
            }

            // Use logical cores on the same physical core only after all physical cores are used.
            uint32 threadIndex = coreThreadIndex + puIndex * numCores;
            if (threadIndex >= b2_maxThreads)
            {
                break;
            }

            m_puAffinities[threadIndex] = pu->cpuset;
            ++puThreadIndex;
        }
    }

    // Fill in values for any additional threads in case of over subscription.
    uint32 numPUs = puThreadIndex;
    for (uint32 copyIndex = 0; puThreadIndex < b2_maxThreads; ++copyIndex)
    {
        uint32 threadIndex = copyIndex % numPUs;
        m_puAffinities[puThreadIndex++] = m_puAffinities[threadIndex];
    }
    for (uint32 copyIndex = 0; coreThreadIndex < b2_maxThreads; ++copyIndex)
    {
        uint32 threadIndex = copyIndex % numCores;
        m_coreAffinities[coreThreadIndex++] = m_coreAffinities[threadIndex];
    }
}

void b2Hardware::_setThreadAffinity(uint32 threadIndex, bool useCoreAffinity)
{
    // Locking here might be unecessary but I haven't seen a guarantee that this is thread safe on all platforms.
    std::lock_guard<std::mutex> lk(m_mutex);

    if (useCoreAffinity)
    {
        hwloc_set_cpubind(m_topology, m_coreAffinities[threadIndex], HWLOC_CPUBIND_THREAD);
    }
    else
    {
        hwloc_set_cpubind(m_topology, m_puAffinities[threadIndex], HWLOC_CPUBIND_THREAD);
    }
}

#endif // #ifdef b2_hwloc