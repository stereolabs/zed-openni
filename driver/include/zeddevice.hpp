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

    virtual OniStatus setProperty(int propertyId, const void* data, int dataSize) override;
    virtual OniStatus getProperty(int propertyId, void* data, int* dataSize) override;
    virtual OniBool isPropertySupported(int propertyId) override;
protected:
    OniStatus initialize();
    void initCameraParams();
    void shutdown();
    OniStatus initializeStreams();
    OniStatus addStream(OniSensorType sensorType, int profileId, std::vector<ZedStreamProfileInfo> *profiles);
    void findStreamProfiles(std::vector<ZedStreamProfileInfo>* dst, OniSensorType sensorType);
    int getCurrentProfileId(std::vector<ZedStreamProfileInfo>* profiles);
    int getProfileId(const std::vector<ZedStreamProfileInfo>* profiles, int width, int height, int fps);

    void publishFrame(std::shared_ptr<ZedStream> stream, int frameId);
    void grabThreadFunc();

    bool hasEnabledStreams();

protected:
    std::mutex mStateMutex;
    std::mutex mStreamsMutex;
    std::mutex mCloseMutex;

    OniDeviceInfo mInfo;
    std::vector<OniSensorInfo> mSensorInfo;

    OniImageRegistrationMode mRegistrationMode;

    std::unique_ptr<std::thread> mGrabThread;

    volatile bool mThreadRunning=false;
    volatile bool mStopThread=false;
    volatile bool mVideoModeChanged=false;

    sl::Camera mZed;
    sl::DeviceProperties mZedProp;

    bool mRightMeasure=false;
    bool mVerbose=false;

    std::vector<ZedStreamProfileInfo> mProfiles;

    ZedDriver* mDriver = nullptr;

    std::list<std::shared_ptr<ZedStream>> mAvailableStreams;
    std::list<std::shared_ptr<ZedStream>> mCreatedStreams;

    std::list<std::shared_ptr<ZedStream>> mStreams;

    sl::InitParameters mZedInitParams;
    sl::RuntimeParameters mZedRtParams;
};

} } // namespace driver // namespace oni


#endif
