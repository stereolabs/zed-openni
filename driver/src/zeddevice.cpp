#include "zeddevice.hpp"

#define SENSOR_COUNT 3

namespace oni { namespace driver {

ZedDevice::ZedDevice(class ZedDriver* driver, sl::DeviceProperties& prop)
    : mDriver(driver)
    , mZedProp(prop)

{
    zedLogDebug("+ZedDevice");
}

ZedDevice::~ZedDevice()
{
    zedLogDebug("~Rs2Device");

    shutdown();
}

void ZedDevice::shutdown()
{
    zedLogFunc("");

    std::lock_guard<std::mutex> lock(mStateMutex);

    if(mThreadRunning)
    {
        mStopThread = true;
        if( mGrabThread->joinable() )
        {
            mGrabThread->join();
        }
    }
}

OniStatus ZedDevice::getSensorInfoList(OniSensorInfo** pSensorInfos, int* numSensors)
{
    zedLogFunc("");

    std::lock_guard<std::mutex> lock(mStateMutex);

    *numSensors = (int)mSensorInfo.size();
    *pSensorInfos = ((*numSensors > 0) ? &mSensorInfo[0] : nullptr);

    return ONI_STATUS_OK;
}

StreamBase* ZedDevice::createStream(OniSensorType sensorType)
{
    zedLogFunc("sensorType=%d", (int)sensorType);

    std::lock_guard<std::mutex> lock(mStreamsMutex);

    for (auto iter = mStreams.begin(); iter != mStreams.end(); ++iter)
    {
        std::shared_ptr<ZedStream> streamObj = *iter;
        if (streamObj->getOniType() == sensorType)
        {
            mCreatedStreams.push_back(streamObj);
            mStreams.remove(streamObj);
            return streamObj.get();
        }
    }

    return nullptr;
}

void ZedDevice::destroyStream(StreamBase* streamBase)
{
    zedLogFunc("ptr=%p", streamBase);

    if(streamBase)
    {
        std::lock_guard<std::mutex> lock(mStreamsMutex);

        ZedStream* streamObj = (ZedStream*)streamBase;
        std::shared_ptr<ZedStream> streamPtr(streamObj);

        streamPtr->stop();

        mStreams.push_back(streamPtr);
        mCreatedStreams.remove(streamPtr);
    }
}

void ZedDevice::changeVideoMode(const ZedStreamProfileInfo *spi)
{
    zedLogFunc("");

    std::lock_guard<std::mutex> lock(mCamMutex);

    restartCamera(spi);
}

void ZedDevice::grabThreadFunc()
{
    zedLogFunc("");
    zedLogDebug("Grab thread started");

    mThreadRunning=true;
    mStopThread=false;

    sl::ERROR_CODE ret;
    sl::RuntimeParameters rtParams; // TODO LOAD RT PARAMETERS

    int frameCount=0;
    int noStreamCount=0;

    while(true)
    {
        if(mStopThread)
        {
            zedLogDebug("Grab thread stopped");
            break;
        }

        std::lock_guard<std::mutex> lock(mCamMutex);

        ret = mZed.grab(rtParams);
        if(ret!=sl::ERROR_CODE::SUCCESS)
        {
            zedLogError("ZED Grab error: %s", sl::toString(ret).c_str());
            break; // TODO Improve grab error handling
        }

        if(!hasEnabledStreams())
        {
            zedLogDebug("No Enabled Streams #%d", ++noStreamCount);
            continue;
        }
#if 0
        zedLogDebug("Grabbed #%d", frameCount);
#endif
        for (auto iter = mCreatedStreams.begin(); iter != mCreatedStreams.end(); ++iter)
        {
            publishFrame(*iter, frameCount );
        }
        ++frameCount;
    };

    zedLogDebug("Grab thread finished");
    mThreadRunning = false;
}

void ZedDevice::publishFrame(std::shared_ptr<ZedStream> stream, int frameId)
{
    OniSensorType sensType = stream->getOniType();

    sl::Mat zedFrame;
    sl::ERROR_CODE ret;

    const void* frameData = nullptr;

    switch(sensType)
    {
    case ONI_SENSOR_DEPTH:
        ret = mZed.retrieveMeasure( zedFrame, sl::MEASURE::DEPTH_U16_MM );
        frameData = zedFrame.getPtr<sl::ushort1>();
        break;
    case ONI_SENSOR_COLOR:
        ret = mZed.retrieveImage( zedFrame, sl::VIEW::LEFT );
        frameData = zedFrame.getPtr<sl::uchar4>();
        break;
    case ONI_SENSOR_IR:
        ret = mZed.retrieveImage( zedFrame, sl::VIEW::LEFT_GRAY );
        frameData = zedFrame.getPtr<sl::uchar1>();
        break;
    }

    if(ret!=sl::ERROR_CODE::SUCCESS)
    {
        zedLogError("Error retrieving ZED frame: %s", sl::toString(ret).c_str());
        return;
    }

    ZedStreamProfileInfo spi = stream->getProfile();

    OniFrame* oniFrame;
    {
        oniFrame = stream->getServices().acquireFrame();
        if (!oniFrame)
        {
            zedLogError("acquireFrame failed");
            return;
        }
    }

    oniFrame->sensorType = sensType;
    oniFrame->timestamp = zedFrame.timestamp.getMicroseconds();
    oniFrame->frameIndex = frameId;

    OniVideoMode mode = stream->getVideoMode();

    oniFrame->width = spi.width;
    oniFrame->height = spi.height;
    oniFrame->stride = spi.stride;

    oniFrame->videoMode = mode;
    oniFrame->croppingEnabled = false;
    oniFrame->cropOriginX = 0;
    oniFrame->cropOriginY = 0;

    const size_t frameSize = oniFrame->stride * oniFrame->height;

    if (frameSize != oniFrame->dataSize)
    {
        zedLogError("invalid frame: rsSize=%u oniSize=%u", (unsigned int)frameSize, (unsigned int)oniFrame->dataSize);
        stream->getServices().releaseFrame(oniFrame);
        return;
    }

    if(sensType==ONI_SENSOR_IR || sensType==ONI_SENSOR_DEPTH)
    {
        memcpy(oniFrame->data, frameData, oniFrame->dataSize);

#if 0
        if(sensType==ONI_SENSOR_DEPTH)
        {
            zedLogDebug("[%lu] Depth ready", oniFrame->timestamp);
        }
        else
        {
            zedLogDebug("[%lu] Gray ready", oniFrame->timestamp);
        }
#endif
        stream->raiseNewFrame(oniFrame);
    }
    else if(sensType==ONI_SENSOR_COLOR)
    {
        sl::uchar4* bgra = (sl::uchar4*)frameData;
        uint8_t* ch = (uint8_t*)oniFrame->data;

        int chIdx = -1;
        for (int i = 0; i < oniFrame->width * oniFrame->height; ++i)
        {
            ch[++chIdx] = (*bgra).b;
            ch[++chIdx] = (*bgra).g;
            ch[++chIdx] = (*bgra).r;
            ++bgra;
        }
#if 0
        zedLogDebug("[%lu] Color ready", oniFrame->timestamp);
#endif
        stream->raiseNewFrame(oniFrame);
    }

    stream->getServices().releaseFrame(oniFrame);
}

OniStatus ZedDevice::startCamera(const ZedStreamProfileInfo *spi)
{
    zedLogFunc("");

    // ----> Retrieve configuration
    sl::InitParameters initParams;
    {
        initParams.input.setFromSerialNumber(mZedProp.serial_number);
        initParams.camera_fps = spi->framerate;
        initParams.camera_resolution = spi->zedRes;
        initParams.coordinate_units = sl::UNIT::MILLIMETER;
        initParams.depth_maximum_distance = 9999.f;
    }
    // <---- Retrieve configuration

    // ----> Initialize ZED
    sl::ERROR_CODE ret = mZed.open( initParams );
    if(ret!=sl::ERROR_CODE::SUCCESS)
    {
        zedLogError("Error opening ZED camera: %s", sl::toString(ret).c_str());
        return ONI_STATUS_ERROR;
    }
    zedLogDebug("ZED Camera connected");
    // <---- Initialize ZED

    return ONI_STATUS_OK;
}

void ZedDevice::stopCamera()
{
    zedLogFunc("");

    if(mZed.isOpened())
    {
        mZed.close();
    }
}

OniStatus ZedDevice::restartCamera(const ZedStreamProfileInfo *spi)
{
    zedLogFunc("");

    stopCamera();

    startCamera(spi);

    bool hasStreams = false;
    for( auto iter = mCreatedStreams.begin(); iter!=mCreatedStreams.end(); iter++ )
    {
        hasStreams=true;

        std::shared_ptr<ZedStream> streamPtr = *iter;

        int sensorId = streamPtr->getSensorId();
        const std::vector<ZedStreamProfileInfo>* profiles = streamPtr->getProfiles();
        int profileId = getProfileId( profiles, spi->width, spi->height, spi->framerate);

        if(streamPtr->initialize(shared_from_this(), sensorId, profileId, profiles ) != ONI_STATUS_OK)
        {
            zedLogError("ZedStream::initialize failed");
            return ONI_STATUS_ERROR;
        }
    }
}

bool ZedDevice::hasEnabledStreams()
{
    for (auto iter = mCreatedStreams.begin(); iter != mCreatedStreams.end(); ++iter)
    {
        std::shared_ptr<ZedStream> stream = *iter;
        if (stream->isEnabled())
        {
            return true;
        }
    }

    return false;
}

OniStatus ZedDevice::initialize()
{
    zedLogFunc("");

    std::lock_guard<std::mutex> lock(mStateMutex);

    // ----> Initialize ZED
    sl::InitParameters initParams;
    initParams.input.setFromSerialNumber(mZedProp.serial_number);
    initParams.camera_fps = 30;
    initParams.camera_resolution = sl::RESOLUTION::VGA;
    initParams.coordinate_units = sl::UNIT::MILLIMETER;
    initParams.depth_maximum_distance = 9999.f;

    sl::ERROR_CODE ret = mZed.open( initParams );
    if(ret!=sl::ERROR_CODE::SUCCESS)
    {
        zedLogError("Error opening ZED camera: %s", sl::toString(ret).c_str());
        return ONI_STATUS_ERROR;
    }
    zedLogDebug("ZED Camera connected");
    // <---- Initialize ZED

    // ----> Initialize streams
    {
        std::lock_guard<std::mutex> lock(mStreamsMutex);

        if (initializeStreams() != ONI_STATUS_OK)
        {
            zedLogError("initializeStreams failed");
            return ONI_STATUS_ERROR;
        }
    }
    // <---- Initialize streams

    try {
        mGrabThread = std::make_unique<std::thread>(&ZedDevice::grabThreadFunc, this);
    }
    catch (std::exception& ex) {
        zedLogError("std::thread failed: %s", ex.what());
        return ONI_STATUS_ERROR;
    }

    return ONI_STATUS_OK;
}

OniStatus ZedDevice::initializeStreams()
{
    zedLogFunc("");

    mProfiles.clear();

    std::map<int, OniSensorType> sensorStreams;

    int profileId = 0;

    for(size_t res=static_cast<int>(sl::RESOLUTION::HD2K); res<static_cast<int>(sl::RESOLUTION::LAST); res++)
    {
        sl::RESOLUTION zed_res = static_cast<sl::RESOLUTION>(res);
        sl::Resolution resol = sl::getResolution(zed_res);

        // ----> 15 FPS
        for(int i=0; i<SENSOR_COUNT; i++)
        {
            ZedStreamProfileInfo spi;
            spi.sensorId = i;
            spi.profileId = profileId;
            spi.width = resol.width;
            spi.height = resol.height;
            spi.framerate = 15;
            spi.zedRes = zed_res;
            if(i==0)
            {
                spi.streamType = ONI_SENSOR_COLOR;
                spi.format = ONI_PIXEL_FORMAT_RGB888;
                spi.stride = spi.width*3;
            }
            else if(i==1)
            {
                spi.streamType = ONI_SENSOR_DEPTH;
                spi.format = ONI_PIXEL_FORMAT_DEPTH_1_MM;
                spi.stride = spi.width*2;
            }
            else
            {
                spi.streamType = ONI_SENSOR_IR;
                spi.format = ONI_PIXEL_FORMAT_GRAY8;
                spi.stride = spi.width;
            }
            mProfiles.push_back(spi);

            zedLogDebug("\ttype=%d sensorId=%d profileId=%d format=%d width=%d height=%d framerate=%d",
                        (int)spi.streamType, (int)spi.sensorId, (int)spi.profileId, (int)spi.format,
                        (int)spi.width, (int)spi.height, (int)spi.framerate);

            sensorStreams[spi.profileId] = spi.streamType;
        }
        profileId++;
        // <---- 15 FPS

        // ----> 30 FPS
        if(zed_res != sl::RESOLUTION::HD2K)
        {
            for(int i=0; i<SENSOR_COUNT; i++)
            {
                ZedStreamProfileInfo spi;
                spi.sensorId = i;
                spi.profileId = profileId;
                spi.width = resol.width;
                spi.height = resol.height;
                spi.framerate = 30;
                spi.zedRes = zed_res;
                if(i==0)
                {
                    spi.streamType = ONI_SENSOR_COLOR;
                    spi.format = ONI_PIXEL_FORMAT_RGB888;
                    spi.stride = spi.width*3;
                }
                else if(i==1)
                {
                    spi.streamType = ONI_SENSOR_DEPTH;
                    spi.format = ONI_PIXEL_FORMAT_DEPTH_1_MM;
                    spi.stride = spi.width*2;
                }
                else
                {
                    spi.streamType = ONI_SENSOR_IR;
                    spi.format = ONI_PIXEL_FORMAT_GRAY8;
                    spi.stride = spi.width;
                }
                mProfiles.push_back(spi);

                zedLogDebug("\ttype=%d sensorId=%d profileId=%d format=%d width=%d height=%d framerate=%d",
                            (int)spi.streamType, (int)spi.sensorId, (int)spi.profileId, (int)spi.format,
                            (int)spi.width, (int)spi.height, (int)spi.framerate);

                sensorStreams[spi.profileId] = spi.streamType;
            }
            profileId++;
        }
        // <---- 30 FPS

        // ----> 60 FPS
        if(zed_res != sl::RESOLUTION::HD2K &&
                zed_res != sl::RESOLUTION::HD1080)
        {
            for(int i=0; i<SENSOR_COUNT; i++)
            {
                ZedStreamProfileInfo spi;
                spi.sensorId = i;
                spi.profileId = profileId;
                spi.width = resol.width;
                spi.height = resol.height;
                spi.framerate = 60;
                spi.zedRes = zed_res;
                if(i==0)
                {
                    spi.streamType = ONI_SENSOR_COLOR;
                    spi.format = ONI_PIXEL_FORMAT_RGB888;
                    spi.stride = spi.width*3;
                }
                else if(i==1)
                {
                    spi.streamType = ONI_SENSOR_DEPTH;
                    spi.format = ONI_PIXEL_FORMAT_DEPTH_1_MM;
                    spi.stride = spi.width*2;
                }
                else
                {
                    spi.streamType = ONI_SENSOR_IR;
                    spi.format = ONI_PIXEL_FORMAT_GRAY8;
                    spi.stride = spi.width;
                }
                mProfiles.push_back(spi);

                zedLogDebug("\ttype=%d sensorId=%d profileId=%d format=%d width=%d height=%d framerate=%d",
                            (int)spi.streamType, (int)spi.sensorId, (int)spi.profileId, (int)spi.format,
                            (int)spi.width, (int)spi.height, (int)spi.framerate);

                sensorStreams[spi.profileId] = spi.streamType;
            }
            profileId++;
        }
        // <---- 60 FPS

        // ----> 100 FPS
        if(zed_res == sl::RESOLUTION::VGA)
        {
            for(int i=0; i<SENSOR_COUNT; i++)
            {
                ZedStreamProfileInfo spi;
                spi.sensorId = i;
                spi.profileId = profileId;
                spi.width = resol.width;
                spi.height = resol.height;
                spi.framerate = 100;
                spi.zedRes = zed_res;
                if(i==0) // Color
                {
                    spi.streamType = ONI_SENSOR_COLOR;
                    spi.format = ONI_PIXEL_FORMAT_RGB888;
                    spi.stride = spi.width*3;
                }
                else if(i==1) // Depth
                {
                    spi.streamType = ONI_SENSOR_DEPTH;
                    spi.format = ONI_PIXEL_FORMAT_DEPTH_1_MM;
                    spi.stride = spi.width*2;
                }
                else // Gray/IR
                {
                    spi.streamType = ONI_SENSOR_IR;
                    spi.format = ONI_PIXEL_FORMAT_GRAY8;
                    spi.stride = spi.width;
                }

                mProfiles.push_back(spi);

                zedLogDebug("\ttype=%d sensorId=%d profileId=%d format=%d width=%d height=%d framerate=%d",
                            (int)spi.streamType, (int)spi.sensorId, (int)spi.profileId, (int)spi.format,
                            (int)spi.width, (int)spi.height, (int)spi.framerate);

                sensorStreams[spi.profileId] = spi.streamType;
            }
            profileId++;
        }
        // <---- 100 FPS
    }

    for(int sensorId=0; sensorId<SENSOR_COUNT; sensorId++)
    {
        OniSensorType sensType = convertStreamType(sensorId);
        std::vector<ZedStreamProfileInfo> profiles;
        findStreamProfiles(&profiles, sensType);

        int profileId = getCurrentProfileId(&profiles);

        if (addStream( sensType, profileId, &profiles) != ONI_STATUS_OK)
        {
            zedLogDebug("Error adding stream sensorId=%d sensorType=%d profileId=%d", sensorId, sensType, profileId);
        }
    }

    zedLogDebug("FILL OniSensorInfo");
    for (auto iter = mStreams.begin(); iter != mStreams.end(); ++iter)
    {
        ZedStream* stream = (*iter).get();
        zedLogDebug("STREAM type=%d sensorId=%d", (int)stream->getOniType(), stream->getSensorId());

        std::vector<ZedStreamProfileInfo> profiles;
        findStreamProfiles(&profiles, stream->getOniType());

        OniSensorInfo info;
        info.sensorType = stream->getOniType();
        info.numSupportedVideoModes = (int)profiles.size();
        info.pSupportedVideoModes = nullptr;

        if (info.numSupportedVideoModes > 0)
        {
            info.pSupportedVideoModes = new OniVideoMode[info.numSupportedVideoModes];
            int modeId = 0;

            for (auto p = profiles.begin(); p != profiles.end(); ++p)
            {
                OniVideoMode& mode = info.pSupportedVideoModes[modeId];
                mode.pixelFormat = p->format;
                mode.resolutionX = p->width;
                mode.resolutionY = p->height;
                mode.fps = p->framerate;
                modeId++;

#if 1
                zedLogDebug("\tstreamType=%d sensorId=%d profileId=%d format=%d width=%d height=%d framerate=%d",
                            (int)p->streamType, (int)p->sensorId, (int)p->profileId, (int)p->format, (int)p->width, (int)p->height, (int)p->framerate);
#endif
            }

            mSensorInfo.push_back(info);
        }
    }

    return ONI_STATUS_OK;
}

int ZedDevice::getProfileId(const std::vector<ZedStreamProfileInfo>* profiles, int width, int height, int fps)
{
    for (auto iter = (*profiles).begin(); iter != (*profiles).end(); ++iter)
    {
        ZedStreamProfileInfo spi = *iter;
        if(spi.framerate == fps &&
                spi.height == height &&
                spi.width == width)
        {
            return spi.profileId;
        }
    }
    return -1;
}

int ZedDevice::getCurrentProfileId(std::vector<ZedStreamProfileInfo>* profiles)
{
    sl::CameraInformation zedInfo = mZed.getCameraInformation();
    int w = zedInfo.camera_resolution.width;
    int h = zedInfo.camera_resolution.height;
    int fps = zedInfo.camera_fps;

    zedLogFunc("w=%d h=%d fps=%d",w,h,fps);

    for (auto iter = (*profiles).begin(); iter != (*profiles).end(); ++iter)
    {
        ZedStreamProfileInfo spi = *iter;
        if(spi.framerate == fps &&
                spi.height == h &&
                spi.width == w)
        {
            return spi.profileId;
        }
    }
    return -1;
}

void ZedDevice::findStreamProfiles(std::vector<ZedStreamProfileInfo>* dst, OniSensorType sensorType)
{
    zedLogFunc("type=%d", (int)sensorType);

    for (auto iter = mProfiles.begin(); iter != mProfiles.end(); ++iter)
    {
        ZedStreamProfileInfo& p = *iter;
        if(p.streamType == sensorType)
        {
            dst->push_back(p);
        }
    }
    zedLogDebug("%lu stream profiles available", dst->size());
}

OniStatus ZedDevice::addStream(OniSensorType sensorType, int profileId, std::vector<ZedStreamProfileInfo>* profiles)
{
    zedLogFunc("type=%d profileId=%d", (int)sensorType, profileId);

    int sensorId = convertStreamType(sensorType);

    std::shared_ptr<ZedStream> streamObj;
    switch (sensorType)
    {
    case ONI_SENSOR_IR: streamObj = std::make_shared<ZedGrayStream>(); break;
    case ONI_SENSOR_COLOR: streamObj = std::make_shared<ZedColorStream>(); break;
    case ONI_SENSOR_DEPTH: streamObj = std::make_shared<ZedDepthStream>(); break;

    default:
    {
        zedLogError("Invalid type=%d", (int)sensorType);
        return ONI_STATUS_ERROR;
    }
    }

    if (streamObj->initialize(shared_from_this(), sensorId, profileId, profiles) != ONI_STATUS_OK)
    {
        zedLogError("ZedStream::initialize failed");
        return ONI_STATUS_ERROR;
    }

    mStreams.push_back(streamObj);
    return ONI_STATUS_OK;
}

} } // namespace driver // namespace oni
