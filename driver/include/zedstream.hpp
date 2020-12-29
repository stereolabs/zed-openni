#ifndef ZEDSTREAM_HPP
#define ZEDSTREAM_HPP

#include <Driver/OniDriverAPI.h>

#include "zedtools.hpp"

namespace oni { namespace driver {


class ZedStream : public StreamBase {
    friend class ZedDevice;

public:
    ZedStream(OniSensorType sensorType);
    virtual ~ZedStream();

protected:
    ZedStream(const ZedStream&);
    void operator=(const ZedStream&);

    void shutdown();

private:

    class ZedDevice* mDevice;

    bool mEnabled = false;

    int mZedType;
    OniSensorType mOniType;
};

} } // namespace driver // namespace oni

#endif
