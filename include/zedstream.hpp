#ifndef ZEDSTREAM_HPP
#define ZEDSTREAM_HPP
#include <Driver/OniDriverAPI.h>

#include <sl/Camera.hpp>

class ZedStream : public oni::driver::StreamBase {
    friend class ZedDevice;

public:
    ZedStream();
    virtual ~ZedStream();

    virtual OniStatus setProperty(int propertyId, const void* data, int dataSize);



private:
    sl::Camera mZed;

    class ZedDevice* mDevice;

};

#endif
