#ifndef ZEDDEVICE_HPP
#define ZEDDEVICE_HPP

#include <Driver/OniDriverAPI.h>
#include <sl/Camera.hpp>

#include "zedtools.hpp"
#include "zedstream.hpp"

namespace oni { namespace driver {

class ZedDevice : public DeviceBase {
    friend class ZedDriver;

public:
    ZedDevice(class ZedDriver* driver, sl::DeviceProperties& prop);
    virtual ~ZedDevice();

    OniStatus initialize(){ zedLogFunc(""); return ONI_STATUS_OK;}

    virtual OniStatus getSensorInfoList(OniSensorInfo** pSensorInfos, int* numSensors) override;

    virtual StreamBase* createStream(OniSensorType){zedLogFunc("");}
    virtual void destroyStream(StreamBase* pStream){zedLogFunc("");}

protected:
    std::mutex mStateMutex;
    std::mutex mDevicesMutex;

    OniDeviceInfo mInfo;
    std::vector<OniSensorInfo> mSensorInfo;
};

} } // namespace driver // namespace oni


#endif
