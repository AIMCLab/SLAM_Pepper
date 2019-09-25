#include <alproxies/almemoryproxy.h>

#include "Device/QIMemory.h"

using namespace AL;

QIMemory::QIMemory(boost::shared_ptr<ALBroker> broker, const char* name)
    : QIDevice(broker, name),
      m_memory_proxy(broker ? new ALMemoryProxy(broker) : nullptr)
{
}

QIMemory::~QIMemory()
{
    unsubscribeAll();
}

void QIMemory::pullRemoteData()
{
}

void QIMemory::writeSharedMemory()
{
}

ALValue QIMemory::getData(const std::string& key)
{
    if (m_memory_proxy)
        return m_memory_proxy->getData(key);
    else
        return ALValue();
}

void QIMemory::subscribeEvent(const std::string& name, CallbackFn fn)
{
    if (m_memory_proxy)
    {
        qi::AnyObject subscriber = m_memory_proxy->subscriber(name);
        uint64_t id = subscriber.connect("signal", fn);
        m_subscriber_list.push_back(std::make_pair(subscriber, id));
    }
}

void QIMemory::unsubscribeAll()
{
    for (auto& p : m_subscriber_list)
        p.first.disconnect(p.second);
    m_subscriber_list.clear();
}
