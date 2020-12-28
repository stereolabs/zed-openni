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

    virtual OniStatus getSensorInfoList(OniSensorInfo** pSensorInfos, int* numSensors) override;

    virtual StreamBase* createStream(OniSensorType){zedLogFunc("");}
    virtual void destroyStream(StreamBase* pStream){zedLogFunc("");}

protected:
    OniStatus initialize();
    OniStatus initializeStreams();

    void grabThreadFunc();

protected:
    std::mutex mStateMutex;
    std::mutex mStreamsMutex;

    OniDeviceInfo mInfo;
    std::vector<OniSensorInfo> mSensorInfo;

    std::unique_ptr<std::thread> mGrabThread;

    bool mThreadRunning=false;
    bool mStopThread=false;

    sl::Camera mZed;
    sl::DeviceProperties mZedProp;

    ZedDriver* mDriver = nullptr;
};

} } // namespace driver // namespace oni


#endif
