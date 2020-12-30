#include "zeddevice.hpp"


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
    // TODO

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

StreamBase* ZedDevice::createStream(OniSensorType)
{
    zedLogFunc("");

    return nullptr;
}

void ZedDevice::destroyStream(StreamBase* pStream)
{
    zedLogFunc("");
}

void ZedDevice::updateConfiguration()
{
    zedLogFunc("");

    mConfigId++;
}

void ZedDevice::grabThreadFunc()
{
    zedLogFunc("");
    zedLogDebug("Grab thread started");

    mThreadRunning=true;
    mStopThread=false;

    sl::ERROR_CODE ret;
    sl::RuntimeParameters rtParams; // TODO LOAD RT PARAMETERS

    while(true)
    {
        if(mStopThread)
        {
            zedLogDebug("Grab thread stopped");
            break;
        }

        ret = mZed.grab(rtParams);
        if(ret!=sl::ERROR_CODE::SUCCESS)
        {
            zedLogError("ZED Grab error: %s", sl::toString(ret).c_str());
            break; // TODO Improve grab error handling
        }
    };

    zedLogDebug("Grab thread finished");
    mThreadRunning = false;
}

OniStatus ZedDevice::initialize()
{
    zedLogFunc("");

    std::lock_guard<std::mutex> lock(mStateMutex);

    // ----> Initialize ZED
    sl::InitParameters initParams;
    initParams.input.setFromSerialNumber(mZedProp.serial_number);
    initParams.camera_fps = 100;
    initParams.camera_resolution = sl::RESOLUTION::VGA;

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

    int streamId=0;

    for(size_t res=static_cast<int>(sl::RESOLUTION::HD2K); res<static_cast<int>(sl::RESOLUTION::LAST); res++)
    {
        sl::RESOLUTION zed_res = static_cast<sl::RESOLUTION>(res);

        // ----> 15 FPS
        for(int i=0; i<3; i++)
        {
            ZedStreamProfileInfo spi;
            spi.sensorId = i;
            spi.streamId = streamId++;

            switch(zed_res)
            {
            case sl::RESOLUTION::HD2K:
                spi.height = 1242;
                spi.width = 2208;
                break;
            case sl::RESOLUTION::HD1080:
                spi.height = 1080;
                spi.width = 1920;
                break;
            case sl::RESOLUTION::HD720:
                spi.height = 720;
                spi.width = 1280;
                break;
            case sl::RESOLUTION::VGA:
                spi.height = 376;
                spi.width = 672;
                break;
            }

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

            spi.framerate = 15;

            zedLogDebug("\ttype=%d sensorId=%d streamId=%d format=%d width=%d height=%d framerate=%d",
                        (int)spi.streamType, (int)spi.sensorId, (int)spi.streamId, (int)spi.format,
                        (int)spi.width, (int)spi.height, (int)spi.framerate);

            mProfiles.push_back(spi);
            sensorStreams[spi.streamId] = spi.streamType;
        }
        // <---- 15 FPS

        // ----> 30 FPS
        if(zed_res != sl::RESOLUTION::HD2K)
        {
            for(int i=0; i<3; i++)
            {
                ZedStreamProfileInfo spi;
                spi.sensorId = i;
                spi.streamId = streamId++;

                switch(zed_res)
                {
                case sl::RESOLUTION::HD1080:
                    spi.height = 1080;
                    spi.width = 1920;
                    break;
                case sl::RESOLUTION::HD720:
                    spi.height = 720;
                    spi.width = 1280;
                    break;
                case sl::RESOLUTION::VGA:
                    spi.height = 376;
                    spi.width = 672;
                    break;
                }

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

                spi.framerate = 30;

                zedLogDebug("\ttype=%d sensorId=%d streamId=%d format=%d width=%d height=%d framerate=%d",
                            (int)spi.streamType, (int)spi.sensorId, (int)spi.streamId, (int)spi.format,
                            (int)spi.width, (int)spi.height, (int)spi.framerate);

                mProfiles.push_back(spi);
                sensorStreams[spi.streamId] = spi.streamType;
            }
        }
        // <---- 30 FPS

        // ----> 60 FPS
        if(zed_res != sl::RESOLUTION::HD2K &&
                zed_res != sl::RESOLUTION::HD1080)
        {
            for(int i=0; i<3; i++)
            {
                ZedStreamProfileInfo spi;
                spi.sensorId = i;
                spi.streamId = streamId++;

                switch(zed_res)
                {
                case sl::RESOLUTION::HD720:
                    spi.height = 720;
                    spi.width = 1280;
                    break;
                case sl::RESOLUTION::VGA:
                    spi.height = 376;
                    spi.width = 672;
                    break;
                }

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

                spi.framerate = 60;

                zedLogDebug("\ttype=%d sensorId=%d streamId=%d format=%d width=%d height=%d framerate=%d",
                            (int)spi.streamType, (int)spi.sensorId, (int)spi.streamId, (int)spi.format,
                            (int)spi.width, (int)spi.height, (int)spi.framerate);

                mProfiles.push_back(spi);
                sensorStreams[spi.streamId] = spi.streamType;
            }
        }
        // <---- 60 FPS

        // ----> 100 FPS
        if(zed_res == sl::RESOLUTION::VGA)
        {
            for(int i=0; i<3; i++)
            {
                ZedStreamProfileInfo spi;
                spi.sensorId = i;
                spi.streamId = streamId++;

                spi.height = 376;
                spi.width = 672;


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

                spi.framerate = 100;

                zedLogDebug("\ttype=%d sensorId=%d streamId=%d format=%d width=%d height=%d framerate=%d",
                            (int)spi.streamType, (int)spi.sensorId, (int)spi.streamId, (int)spi.format,
                            (int)spi.width, (int)spi.height, (int)spi.framerate);

                mProfiles.push_back(spi);
                sensorStreams[spi.streamId] = spi.streamType;
            }
        }
        // <---- 100 FPS
    }

    for(int sensorId=0; sensorId<3; sensorId++)
    {
        for (auto iter = sensorStreams.begin(); iter != sensorStreams.end(); ++iter)
        {
            const OniSensorType oniType = iter->second;

            std::vector<ZedStreamProfileInfo> profiles;
            findStreamProfiles(&profiles, oniType, iter->first);

            if (addStream(oniType, sensorId, iter->first, &profiles) != ONI_STATUS_OK)
            {
                zedLogDebug("Error adding stream streamId=%d sensorType=%d ",iter->first, (int)iter->second);
            }
        }
    }

    zedLogDebug("FILL OniSensorInfo");
    for (auto iter = mStreams.begin(); iter != mStreams.end(); ++iter)
    {
        ZedStream* stream = (*iter).get();
        zedLogDebug("STREAM type=%d sensorId=%d streamId=%d", (int)stream->getOniType(), stream->getSensorId(), stream->getStreamId());

        std::vector<ZedStreamProfileInfo> profiles;
        findStreamProfiles(&profiles, stream->getOniType(), stream->getStreamId());

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
                zedLogDebug("\ttype=%d sensorId=%d streamId=%d format=%d width=%d height=%d framerate=%d",
                    (int)p->streamType, (int)p->sensorId, (int)p->streamId, (int)p->format, (int)p->width, (int)p->height, (int)p->framerate);
                #endif
            }

            mSensorInfo.push_back(info);
        }
    }

    return ONI_STATUS_OK;
}

void ZedDevice::findStreamProfiles(std::vector<ZedStreamProfileInfo>* dst, OniSensorType sensorType, int streamId)
{
    for (auto iter = mProfiles.begin(); iter != mProfiles.end(); ++iter)
    {
        ZedStreamProfileInfo& p = *iter;
        if (p.streamType == sensorType && p.streamId == streamId)
        {
            dst->push_back(p);
        }
    }
}

OniStatus ZedDevice::addStream(OniSensorType sensorType, int sensorId, int streamId, std::vector<ZedStreamProfileInfo>* profiles)
{
    zedLogFunc("type=%d sensorId=%d streamId=%d", (int)sensorType, sensorId, streamId);

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

    if (streamObj->initialize(shared_from_this(), sensorId, streamId, profiles) != ONI_STATUS_OK)
    {
        zedLogError("ZedStream::initialize failed");
        return ONI_STATUS_ERROR;
    }

    mStreams.push_back(streamObj);
    return ONI_STATUS_OK;
}

} } // namespace driver // namespace oni
