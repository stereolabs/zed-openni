#ifndef ZEDDEVICE_HPP
#define ZEDDEVICE_HPP

#include <list>

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

    virtual StreamBase* createStream(OniSensorType) override;
    virtual void destroyStream(StreamBase* pStream) override;

protected:
    OniStatus initialize();
    void shutdown();
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

    std::list<std::shared_ptr<ZedStream>> mAvailableStreams;
    std::list<std::shared_ptr<ZedStream>> mCreatedStreams;
};

} } // namespace driver // namespace oni


#endif
