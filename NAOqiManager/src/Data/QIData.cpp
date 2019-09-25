#include "Data/QIData.h"

QIData::QIData()
{
}

QIData::~QIData()
{
}

void QIData::dumpFile(const char* fname)
{
    size_t size = getBufferSize();
    char* buf = new char[size];
    writeBuffer(buf);

    FILE* f = fopen(fname, "wb");
    fwrite(buf, 1, size, f);
    fclose(f);
    delete[] buf;
}
