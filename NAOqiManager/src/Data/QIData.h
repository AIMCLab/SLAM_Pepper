#ifndef QI_DATA
#define QI_DATA

#include <alvalue/alvalue.h>

class QIData
{
public:
    QIData();
    virtual ~QIData();

    virtual AL::ALValue toALValue() const = 0;
    virtual size_t getBufferSize() const = 0;
    virtual void writeBuffer(void* dst) = 0;

    void dumpFile(const char* fname);
};

#endif // QI_DATA
