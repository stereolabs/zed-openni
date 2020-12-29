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
    // TODO

    zedLogFunc("");

    return ONI_STATUS_OK;
}

} } // namespace driver // namespace oni
