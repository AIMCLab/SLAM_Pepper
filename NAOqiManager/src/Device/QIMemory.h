#ifndef QI_MEMORY_H
#define QI_MEMORY_H

#include "Device/QIDevice.h"

namespace AL
{
class ALMemoryProxy;
}

class QIMemory : public QIDevice
{
public:
    typedef boost::function<void(AL::ALValue)> CallbackFn;

    QIMemory(boost::shared_ptr<AL::ALBroker> broker, const char* name);
    virtual ~QIMemory();

    void pullRemoteData() override;
    void writeSharedMemory() override;

    AL::ALValue getData(const std::string& key);

    void subscribeEvent(const std::string& name, CallbackFn fn);
    void unsubscribeAll();

private:
    boost::shared_ptr<AL::ALMemoryProxy> m_memory_proxy;
    std::vector<std::pair<qi::AnyObject, uint64_t>> m_subscriber_list;
};

#endif // QI_MEMORY_H
