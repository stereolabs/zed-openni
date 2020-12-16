#ifndef ZEDDEVICE_HPP
#define ZEDDEVICE_HPP

#include <Driver/OniDriverAPI.h>
#include <sl/Camera.hpp>

#include "zedstream.hpp"

namespace oni { namespace driver {

class ZedDevice : public DeviceBase {
    friend class ZedDriver;

public:
    ZedDevice(class ZedDriver* driver, sl::DeviceProperties& prop){}
    virtual ~ZedDevice(){}

    OniStatus initialize(){ return ONI_STATUS_OK;}

    virtual OniStatus getSensorInfoList(OniSensorInfo** pSensorInfos, int* numSensors){}

    virtual StreamBase* createStream(OniSensorType){}
    virtual void destroyStream(StreamBase* pStream){}

};

} // namespace driver
} // namespace oni

#endif
