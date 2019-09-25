#ifndef QI_CAMERA_CLIENT
#define QI_CAMERA_CLIENT

#include "Client/QIClient.h"
#include "Data/QIImage.h"

class QICameraClient : public QIClient
{
public:
    QICameraClient(const char* name);
    ~QICameraClient();

    boost::shared_ptr<QIImage> getImage() const;

private:
};

#endif // QI_CAMERA_CLIENT
