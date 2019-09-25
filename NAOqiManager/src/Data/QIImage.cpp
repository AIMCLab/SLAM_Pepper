#include "Data/QIImage.h"

#include <cstring>
#include <assert.h>
#include <boost/make_shared.hpp>

QIImage::QIImage(const AL::ALValue& value)
    : m_width(value[0]), m_height(value[1]),
      m_layers(value[2]), m_color_space(value[3]),
      m_time_stamp(1000000ll * (int) value[4] + (int) value[5]),
      m_camera_id(value[7]), m_data(value[6].GetBinary()),
      m_need_free(false)
{
}

QIImage::QIImage(int width, int height, int layers, int colorSpace,
                 const void* data, bool copyData)
    : m_width(width), m_height(height),
      m_layers(layers), m_color_space(colorSpace),
      m_time_stamp(0), m_camera_id(0), m_data(data),
      m_need_free(false)
{
    if (copyData && data != nullptr)
    {
        size_t size = getImageSize();
        char* buffer = new char[size];
        memcpy(buffer, data, size);
        m_need_free = true;
        m_data = buffer;
    }
}

QIImage::QIImage(const void* buffer, bool copyData)
    : m_need_free(false)
{
    const char* ptr = (const char*) buffer;
    m_width = *(int*) ptr, ptr += 4;
    m_height = *(int*) ptr, ptr += 4;
    m_layers = *(int*) ptr, ptr += 4;
    m_color_space = *(int*) ptr, ptr += 4;
    m_time_stamp = *(uint64_t*) ptr, ptr += 8;
    m_camera_id = *(int*) ptr, ptr += 4;
    if (copyData)
    {
        size_t size = getImageSize();
        char* buffer = new char[size];
        memcpy(buffer, ptr, size);
        m_need_free = true;
        m_data = buffer;
    }
    else
        m_data = ptr;
}

QIImage::~QIImage()
{
    if (m_need_free)
        delete[](const char*) m_data;
}

AL::ALValue QIImage::toALValue() const
{
    assert(m_data != nullptr);
    AL::ALValue value;
    value.arraySetSize(12);
    value[0] = m_width;
    value[1] = m_height;
    value[2] = m_layers;
    value[3] = m_color_space;
    value[4] = (int) (m_time_stamp / 1000000);
    value[5] = (int) (m_time_stamp % 1000000);
    value[6].SetBinary(m_data, getImageSize());
    value[7] = m_camera_id;
    return value;
}

void QIImage::writeBuffer(void* dst)
{
    assert(m_data != nullptr);
    char* ptr = (char*) dst;
    *(int*) ptr = m_width, ptr += 4;
    *(int*) ptr = m_height, ptr += 4;
    *(int*) ptr = m_layers, ptr += 4;
    *(int*) ptr = m_color_space, ptr += 4;
    *(uint64_t*) ptr = m_time_stamp, ptr += 8;
    *(int*) ptr = m_camera_id, ptr += 4;
    memcpy(ptr, m_data, getImageSize());
}

boost::shared_ptr<QIImage> QIImage::fromFile(const char* fname)
{
    FILE* f = fopen(fname, "rb");
    fseek(f, 0, SEEK_END);
    size_t size = ftell(f);
    fseek(f, 0, SEEK_SET);

    char* buffer = new char[size];
    fread(buffer, 1, size, f);
    fclose(f);

    auto ptr = boost::make_shared<QIImage>(buffer, true);
    delete[] buffer;
    return ptr;
}
