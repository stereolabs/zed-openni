#ifndef ZEDDEVICE_HPP
#define ZEDDEVICE_HPP

#include <list>

#include <Driver/OniDriverAPI.h>
#include <sl/Camera.hpp>

#include "zedtools.hpp"
#include "zedstream.hpp"

namespace oni { namespace driver {

class ZedDevice : public DeviceBase, public std::enable_shared_from_this<ZedDevice> {
    friend class ZedDriver;
    friend class ZedStream;

public:
    ZedDevice(class ZedDriver* driver, sl::DeviceProperties& prop);
    virtual ~ZedDevice();    

    virtual OniStatus getSensorInfoList(OniSensorInfo** pSensorInfos, int* numSensors) override;

    virtual StreamBase* createStream(OniSensorType sensorType) override;
    virtual void destroyStream(StreamBase* pStream) override;
protected:
    OniStatus initialize();
    void shutdown();
    OniStatus initializeStreams();
    OniStatus addStream(OniSensorType sensorType, int profileId, std::vector<ZedStreamProfileInfo> *profiles);
    void findStreamProfiles(std::vector<ZedStreamProfileInfo>* dst, OniSensorType sensorType);
    int getCurrentProfileId(std::vector<ZedStreamProfileInfo>* profiles);
    int getProfileId(const std::vector<ZedStreamProfileInfo>* profiles, int width, int height, int fps);

    void changeVideoMode(const ZedStreamProfileInfo* spi);

    void publishFrame(std::shared_ptr<ZedStream> stream, int frameId);

    void grabThreadFunc();

    OniStatus startCamera(const ZedStreamProfileInfo* spi);
    void stopCamera();
    OniStatus restartCamera(const ZedStreamProfileInfo* spi);
    bool hasEnabledStreams();

protected:
    std::mutex mStateMutex;
    std::mutex mStreamsMutex;
    std::mutex mCamMutex;

    OniDeviceInfo mInfo;
    std::vector<OniSensorInfo> mSensorInfo;

    std::unique_ptr<std::thread> mGrabThread;

    volatile bool mThreadRunning=false;
    volatile bool mStopThread=false;
    volatile bool mVideoModeChanged=false;

    sl::Camera mZed;
    sl::DeviceProperties mZedProp;

    std::vector<ZedStreamProfileInfo> mProfiles;

    ZedDriver* mDriver = nullptr;

    std::list<std::shared_ptr<ZedStream>> mAvailableStreams;
    std::list<std::shared_ptr<ZedStream>> mCreatedStreams;

    std::list<std::shared_ptr<ZedStream>> mStreams;
};

} } // namespace driver // namespace oni


#endif
