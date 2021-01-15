///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2021, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

#include "zeddevice.hpp"
#include "PS1080.h"
#include <chrono>

#define SENSOR_COUNT 3

namespace oni { namespace driver {

ZedDevice::ZedDevice(class ZedDriver* driver, sl::DeviceProperties& prop)
    : mDriver(driver)
    , mZedProp(prop)

{
    if(mVerbose)
        zedLogDebug("+ZedDevice");
}

ZedDevice::~ZedDevice()
{
    if(mVerbose)
        zedLogDebug("~Rs2Device");

    shutdown();
}

void ZedDevice::shutdown()
{
    if(mVerbose)
        zedLogFunc("");

    std::lock_guard<std::mutex> lock1(mStateMutex);

    if(mThreadRunning)
    {
        mStopThread = true;

        std::lock_guard<std::mutex> lock2(mCloseMutex);

        if( mGrabThread->joinable() )
        {
            mGrabThread->join();
        }
        mZed.close();
    }
}

OniStatus ZedDevice::getSensorInfoList(OniSensorInfo** pSensorInfos, int* numSensors)
{
    if(mVerbose)
        zedLogFunc("");

    std::lock_guard<std::mutex> lock(mStateMutex);

    *numSensors = (int)mSensorInfo.size();
    *pSensorInfos = ((*numSensors > 0) ? &mSensorInfo[0] : nullptr);

    return ONI_STATUS_OK;
}

StreamBase* ZedDevice::createStream(OniSensorType sensorType)
{
    if(mVerbose)
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
    if(mVerbose)
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

void ZedDevice::grabThreadFunc()
{
    if(mVerbose)
        zedLogFunc("");
    if(mVerbose)
        zedLogDebug("Grab thread started");

    mThreadRunning=true;
    mStopThread=false;

    sl::ERROR_CODE ret;

    int frameCount=0;
    int noStreamCount=0;

    while(true)
    {
        if(mStopThread)
        {
            if(mVerbose)
                zedLogDebug("Grab thread stopped");
            mThreadRunning=false;
            break;
        }

        std::lock_guard<std::mutex> lock(mCloseMutex);

        ret = mZed.grab(mZedRtParams);
        if(ret!=sl::ERROR_CODE::SUCCESS)
        {
            zedLogError("ZED Grab error: %s", sl::toString(ret).c_str());
        }

        if(!hasEnabledStreams())
        {
            if(mVerbose)
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

    if(mVerbose)
        zedLogDebug("Grab thread finished");
    mThreadRunning = false;
}

//#define MEASURE_PUB_TIME
void ZedDevice::publishFrame(std::shared_ptr<ZedStream> stream, int frameIdx)
{
#ifdef MEASURE_PUB_TIME
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
#endif

    OniSensorType sensType = stream->getOniType();

    sl::Mat zedFrame;
    sl::ERROR_CODE ret = sl::ERROR_CODE::SUCCESS;

    const void* frameData = nullptr;

    switch(sensType)
    {
    case ONI_SENSOR_DEPTH:
#ifdef MEASURE_PUB_TIME
        zedLogDebug("Publishing Depth: ");
#endif
        if(mRightMeasure)
            ret = mZed.retrieveMeasure( zedFrame, sl::MEASURE::DEPTH_U16_MM_RIGHT, sl::MEM::CPU, stream->getRuntimeRes() );
        else
            ret = mZed.retrieveMeasure( zedFrame, sl::MEASURE::DEPTH_U16_MM, sl::MEM::CPU, stream->getRuntimeRes() );
        frameData = zedFrame.getPtr<sl::ushort1>();
        break;
    case ONI_SENSOR_COLOR:
#ifdef MEASURE_PUB_TIME
        zedLogDebug("Publishing Color: ");
#endif
        if(mRightMeasure)
            ret = mZed.retrieveImage( zedFrame, sl::VIEW::RIGHT, sl::MEM::CPU, stream->getRuntimeRes() );
        else
            ret = mZed.retrieveImage( zedFrame, sl::VIEW::LEFT, sl::MEM::CPU, stream->getRuntimeRes() );
        frameData = zedFrame.getPtr<sl::uchar4>();
        break;
    case ONI_SENSOR_IR:
        if(mRightMeasure)
            ret = mZed.retrieveImage( zedFrame, sl::VIEW::RIGHT_GRAY, sl::MEM::CPU, stream->getRuntimeRes() );
        else
            ret = mZed.retrieveImage( zedFrame, sl::VIEW::LEFT_GRAY, sl::MEM::CPU, stream->getRuntimeRes() );
        frameData = zedFrame.getPtr<sl::uchar1>();
        break;
    }

    if(ret!=sl::ERROR_CODE::SUCCESS)
    {
        zedLogError("Error retrieving ZED frame: %s", sl::toString(ret).c_str());
        return;
    }

    ZedStreamProfileInfo spi = stream->getProfile();

    OniFrame* oniFrame = stream->getServices().acquireFrame();
    if (!oniFrame)
    {
        zedLogError("acquireFrame failed");
        return;
    }

    oniFrame->sensorType = sensType;
    oniFrame->timestamp = zedFrame.timestamp.getMicroseconds();
    oniFrame->frameIndex = frameIdx;

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
#ifdef MEASURE_PUB_TIME
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    double elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
    zedLogDebug("\tElapsed %g sec", elapsed/1e6);
#endif
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

void ZedDevice::initCameraParams()
{
    if(mVerbose)
        zedLogFunc("");

    // ----> Initialization parameters
    if(!mZedInitParams.load(DEFAULT_INIT_PARAMS_FILE))
    {
        zedLogDebug("A valid ZED initialization file has not been found. Using default configuration.");
        mZedInitParams.camera_fps = 60;
        mZedInitParams.camera_resolution = sl::RESOLUTION::HD720;
        mZedInitParams.coordinate_units = sl::UNIT::MILLIMETER;
        mZedInitParams.depth_maximum_distance = 9999.f;

        if(!mZedInitParams.save(DEFAULT_INIT_PARAMS_FILE))
        {
            zedLogError("Cannot save the file containing the default parameters value");
        }
        else
        {
            zedLogDebug("A new configuration file containing the default ZED initialization parameters has been saved.");
        }
    }
    else
    {
        zedLogDebug("ZED initialization parameters loaded from file");
    }

    // ----> Overwrite important parameters
    mZedInitParams.input.setFromSerialNumber(mZedProp.serial_number);
    mZedInitParams.coordinate_units = sl::UNIT::MILLIMETER;
    if(mZedInitParams.depth_maximum_distance>9999.f)
    {
        mZedInitParams.depth_maximum_distance = 9999.f;
    }
    // <---- Overwrite important parameters

    zedLogDebug("***** ZED INIT PARAMETERS *****");
    zedLogDebug(" * Resolution: %s", sl::toString(mZedInitParams.camera_resolution).c_str() );
    zedLogDebug(" * Framerate: %d", mZedInitParams.camera_fps );
    zedLogDebug(" * Disable self calib: %d", mZedInitParams.camera_disable_self_calib );
    zedLogDebug(" * Image flip: %d", mZedInitParams.camera_image_flip );
    zedLogDebug(" * Coordinate System: %s", sl::toString(mZedInitParams.coordinate_system).c_str() );
    zedLogDebug(" * Coordinate Units: %s", sl::toString(mZedInitParams.coordinate_units).c_str() );
    zedLogDebug(" * Minimum depth: %g", mZedInitParams.depth_minimum_distance );
    zedLogDebug(" * Maximum depth: %g", mZedInitParams.depth_maximum_distance );
    zedLogDebug(" * Depth mode: %s", sl::toString(mZedInitParams.depth_mode).c_str() );
    zedLogDebug(" * Depth stabilization: %d", mZedInitParams.depth_stabilization );
    zedLogDebug(" * Image enhancement: %d", mZedInitParams.enable_image_enhancement );
    mRightMeasure = mZedInitParams.enable_right_side_measure;
    zedLogDebug(" * Right Side Measure: %d", mRightMeasure );
    zedLogDebug(" * Optional OpenCV calibration file: %s", mZedInitParams.optional_opencv_calibration_file.c_str() );
    zedLogDebug(" * Optional camera setting path: %s", mZedInitParams.optional_settings_path.c_str() );
    zedLogDebug(" * SDK GPU ID: %d", mZedInitParams.sdk_gpu_id );
    mVerbose = mZedInitParams.sdk_verbose;
    zedLogDebug(" * SDK verbose: %d", mVerbose );
    zedLogDebug(" * SDK verbose LOG file: %s", mZedInitParams.sdk_verbose_log_file.c_str() );
    zedLogDebug(" * Sensors required: %d", mZedInitParams.sensors_required );
    zedLogDebug(" * SVO Realtime mode: %d", mZedInitParams.svo_real_time_mode );
    // <---- Initialization parameters

    // ----> Runtime parameters
    if(!mZedRtParams.load(DEFAULT_RT_PARAMS_FILE))
    {
        zedLogDebug("A valid ZED runtime configuration file has not been found. Using default configuration.");
        mZedRtParams.confidence_threshold = 80;
        mZedRtParams.texture_confidence_threshold = 100;

        if(!mZedRtParams.save(DEFAULT_RT_PARAMS_FILE))
        {
            zedLogError("Cannot save the file containing the default parameters value");
        }
        else
        {
            zedLogDebug("A new configuration file containing the default ZED initialization parameters has been saved.");
        }
    }
    else
    {
        zedLogDebug("ZED runtime parameters loaded from file");
    }

    // ----> Overwrite important parameters
    mZedRtParams.enable_depth = true;
    mZedRtParams.measure3D_reference_frame = sl::REFERENCE_FRAME::CAMERA;
    // <---- Overwrite important parameters

    zedLogDebug("***** ZED RUNTIME PARAMETERS *****");
    zedLogDebug(" * Depth Confidence threshold: %d", mZedRtParams.confidence_threshold );
    zedLogDebug(" * Texture Confidence threshold: %d", mZedRtParams.texture_confidence_threshold );
    zedLogDebug(" * Sensing Mode: %s", sl::toString(mZedRtParams.sensing_mode).c_str() );
    // <---- Runtime parameters
}

OniStatus ZedDevice::initialize()
{
    if(mVerbose)
        zedLogFunc("");

    std::lock_guard<std::mutex> lock(mStateMutex);

    // Load camera parameters
    initCameraParams();

    // ----> Initialize ZED
    sl::ERROR_CODE ret = mZed.open( mZedInitParams );
    if(ret!=sl::ERROR_CODE::SUCCESS)
    {
        zedLogError("Error opening ZED camera: %s", sl::toString(ret).c_str());
        return ONI_STATUS_ERROR;
    }
    if(mVerbose)
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
    if(mVerbose)
        zedLogFunc("");

    mProfiles.clear();

    std::map<int, OniSensorType> sensorStreams;

    int profileId = 0;

    bool isResVGA=true;

    sl::RESOLUTION zed_res = mZedInitParams.camera_resolution;
    sl::Resolution native_res = sl::getResolution(zed_res);

    // ----> Native resolution
    for(int i=0; i<SENSOR_COUNT; i++)
    {
        ZedStreamProfileInfo spi;
        spi.sensorId = i;
        spi.profileId = profileId;
        spi.width = native_res.width;
        spi.height = native_res.height;
        spi.framerate = mZedInitParams.camera_fps;
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

        if(mVerbose)
            zedLogDebug("\ttype=%d sensorId=%d profileId=%d format=%d width=%d height=%d framerate=%d",
                        (int)spi.streamType, (int)spi.sensorId, (int)spi.profileId, (int)spi.format,
                        (int)spi.width, (int)spi.height, (int)spi.framerate);

        sensorStreams[spi.profileId] = spi.streamType;
    }
    profileId++;
    // <----  Native resolution

    // ----> 640x480
    if(zed_res!=sl::RESOLUTION::VGA)
    {
        isResVGA = false;
        for(int i=0; i<SENSOR_COUNT; i++)
        {
            ZedStreamProfileInfo spi;
            spi.sensorId = i;
            spi.profileId = profileId;
            spi.width = 640;
            spi.height = 480;
            spi.framerate = mZedInitParams.camera_fps;
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

            if(mVerbose)
                zedLogDebug("\ttype=%d sensorId=%d profileId=%d format=%d width=%d height=%d framerate=%d",
                            (int)spi.streamType, (int)spi.sensorId, (int)spi.profileId, (int)spi.format,
                            (int)spi.width, (int)spi.height, (int)spi.framerate);

            sensorStreams[spi.profileId] = spi.streamType;
        }
        profileId++;
    }
    // <---- 640x480

    // ----> 320x240
    for(int i=0; i<SENSOR_COUNT; i++)
    {
        ZedStreamProfileInfo spi;
        spi.sensorId = i;
        spi.profileId = profileId;
        spi.width = 320;
        spi.height = 240;
        spi.framerate = mZedInitParams.camera_fps;
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

        if(mVerbose)
            zedLogDebug("\ttype=%d sensorId=%d profileId=%d format=%d width=%d height=%d framerate=%d",
                        (int)spi.streamType, (int)spi.sensorId, (int)spi.profileId, (int)spi.format,
                        (int)spi.width, (int)spi.height, (int)spi.framerate);

        sensorStreams[spi.profileId] = spi.streamType;
    }
    profileId++;
    // <---- 320x240

    // ----> 160x120
    for(int i=0; i<SENSOR_COUNT; i++)
    {
        ZedStreamProfileInfo spi;
        spi.sensorId = i;
        spi.profileId = profileId;
        spi.width = 160;
        spi.height = 120;
        spi.framerate = mZedInitParams.camera_fps;
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

        if(mVerbose)
            zedLogDebug("\ttype=%d sensorId=%d profileId=%d format=%d width=%d height=%d framerate=%d",
                        (int)spi.streamType, (int)spi.sensorId, (int)spi.profileId, (int)spi.format,
                        (int)spi.width, (int)spi.height, (int)spi.framerate);

        sensorStreams[spi.profileId] = spi.streamType;
    }
    profileId++;
    // <---- 160x120


    size_t start_w=640;
    size_t start_h=480;
    if(isResVGA)
    {
        start_w=320;
        start_h=240;
    }

    for(int sensorId=0; sensorId<SENSOR_COUNT; sensorId++)
    {
        OniSensorType sensType = convertStreamType(sensorId);
        std::vector<ZedStreamProfileInfo> profiles;
        findStreamProfiles(&profiles, sensType);

        int profId = getCurrentProfileId(&profiles, start_w, start_h);

        if (addStream( sensType, profId, &profiles) != ONI_STATUS_OK)
        {
            zedLogError("Error adding stream sensorId=%d sensorType=%d profileId=%d", sensorId, sensType, profId);
        }
    }

    if(mVerbose)
        zedLogDebug("FILL OniSensorInfo");
    for (auto iter = mStreams.begin(); iter != mStreams.end(); ++iter)
    {
        ZedStream* stream = (*iter).get();
        if(mVerbose)
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
                if(mVerbose)
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

int ZedDevice::getCurrentProfileId(std::vector<ZedStreamProfileInfo>* profiles, size_t w, size_t h)
{
    sl::CameraInformation zedInfo = mZed.getCameraInformation();
    int fps = (int)zedInfo.camera_configuration.fps;

    if(mVerbose)
        zedLogFunc("w=%zd h=%zd fps=%d",w,h,fps);

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
    if(mVerbose)
        zedLogFunc("type=%d", (int)sensorType);

    for (auto iter = mProfiles.begin(); iter != mProfiles.end(); ++iter)
    {
        ZedStreamProfileInfo& p = *iter;
        if(p.streamType == sensorType)
        {
            dst->push_back(p);
        }
    }
    if(mVerbose)
        zedLogDebug("%zu stream profiles available", dst->size());
}

OniStatus ZedDevice::addStream(OniSensorType sensorType, int profileId, std::vector<ZedStreamProfileInfo>* profiles)
{
    if(mVerbose)
        zedLogFunc("type=%d profileId=%d", (int)sensorType, profileId);

    int sensorId = convertStreamType(sensorType);

    std::shared_ptr<ZedStream> streamObj;
    switch (sensorType)
    {
    case ONI_SENSOR_IR: streamObj = std::make_shared<ZedGrayStream>(mVerbose); break;
    case ONI_SENSOR_COLOR: streamObj = std::make_shared<ZedColorStream>(mVerbose); break;
    case ONI_SENSOR_DEPTH: streamObj = std::make_shared<ZedDepthStream>(mVerbose); break;

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

OniStatus ZedDevice::setProperty(int propertyId, const void* data, int dataSize)
{
    if(mVerbose)
        zedLogFunc("propertyId=%d dataSize=%d", propertyId, dataSize);

    switch (propertyId)
    {
    case ONI_DEVICE_PROPERTY_IMAGE_REGISTRATION:
    {
        if (data && (dataSize == sizeof(OniImageRegistrationMode)))
        {
            mRegistrationMode = *((OniImageRegistrationMode*)data);
            if(mVerbose)
                zedLogDebug("registrationMode=%d", (int)mRegistrationMode);
            return ONI_STATUS_OK;
        }
        break;
    }

    default:
    {
        zedLogError("Not supported: propertyId=%d", propertyId);
        return ONI_STATUS_NOT_SUPPORTED;
    }
    }

    zedLogError("propertyId=%d dataSize=%d", propertyId, dataSize);
    return ONI_STATUS_ERROR;
}

OniStatus ZedDevice::getProperty(int propertyId, void* data, int* dataSize)
{
    switch (propertyId)
    {
    case ONI_DEVICE_PROPERTY_SERIAL_NUMBER:
    {
        if (data && dataSize && *dataSize > 0)
        {
            int n = snprintf((char*)data, *dataSize - 1, "%u", mZedProp.serial_number);
            *dataSize = n + 1;
            return ONI_STATUS_OK;
        }
        break;
    }

    case ONI_DEVICE_PROPERTY_IMAGE_REGISTRATION:
    {
        if (data && dataSize && *dataSize == sizeof(OniImageRegistrationMode))
        {
            *((OniImageRegistrationMode*)data) = mRegistrationMode;
            return ONI_STATUS_OK;
        }
        break;
    }

#ifdef EMULATE_PRIMESENSE_HARDWARE
    case XN_MODULE_PROPERTY_AHB:
    {
        if (data && dataSize && *dataSize == 12)
        {
            unsigned char hack[] = {0x40, 0x0, 0x0, 0x28, 0x6A, 0x26, 0x54, 0x4F, 0xFF, 0xFF, 0xFF, 0xFF};
            memcpy(data, hack, sizeof(hack));
            return ONI_STATUS_OK;
        }
        break;
    }
#endif

    default:
    {
        zedLogError("Not supported: propertyId=%d", propertyId);
        return ONI_STATUS_NOT_SUPPORTED;
    }
    }

    zedLogError("propertyId=%d dataSize=%d", propertyId, *dataSize);
    return ONI_STATUS_ERROR;
}

OniBool ZedDevice::isPropertySupported(int propertyId)
{
    switch (propertyId)
    {
    case ONI_DEVICE_PROPERTY_SERIAL_NUMBER:
    case ONI_DEVICE_PROPERTY_IMAGE_REGISTRATION:
        return true;

    default:
        return false;
    }
}

} } // namespace driver // namespace oni
