#ifndef QI_IMAGE
#define QI_IMAGE

#include "Data/QIData.h"

#include <boost/shared_ptr.hpp>

class QIImage : public QIData
{
public:
    QIImage(const AL::ALValue& value);
    QIImage(int width, int height, int layers, int colorSpace = 0,
            const void* data = nullptr, bool copyData = false);
    QIImage(const void* buffer, bool copyData = false);
    virtual ~QIImage();

    virtual AL::ALValue toALValue() const override;
    virtual size_t getBufferSize() const override { return getImageSize() + DATA_OFFSET; }
    virtual void writeBuffer(void* dst) override;

    inline size_t getImageSize() const { return m_width * m_height * m_layers; }

    inline int getWidth(void) const { return m_width; }
    inline int getHeight(void) const { return m_height; }
    inline int getNbLayers(void) const { return m_layers; }
    inline int getColorSpace(void) const { return m_color_space; }
    inline uint64_t getTimeStamp(void) const { return m_time_stamp; }
    inline int getCameraId() const { return m_camera_id; }
    inline const void* getImageData() const { return m_data; }

    static size_t calcBufferSize(int width, int height, int layers)
    {
        return width * height * layers + DATA_OFFSET;
    }

    static boost::shared_ptr<QIImage> fromFile(const char* fname);

private:
    static constexpr int DATA_OFFSET = 7 * sizeof(int);

    /// width of the image
    int m_width;

    /// height of the image
    int m_height;

    /// number of layers of the image
    int m_layers;

    /// color space of the image
    int m_color_space;

    /// Time in microsecond when the image was captured
    uint64_t m_time_stamp;

    /// ID of the camera that shot the picture
    int m_camera_id;

    /// pointer to the image data
    const void* m_data;

    /// whether need to free `m_data`
    bool m_need_free;
};

#endif // QI_IMAGE
