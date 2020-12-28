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
}

OniStatus ZedDevice::getSensorInfoList(OniSensorInfo** pSensorInfos, int* numSensors)
{
    zedLogFunc("");

    std::lock_guard<std::mutex> lock(mStateMutex);

    *numSensors = (int)mSensorInfo.size();
    *pSensorInfos = ((*numSensors > 0) ? &mSensorInfo[0] : nullptr);

    return ONI_STATUS_OK;
}

void ZedDevice::grabThreadFunc()
{
    zedLogFunc("");
    zedLogDebug("Grab thread started");

    mThreadRunning=true;
    mStopThread=false;

    while(true)
    {
        if(mStopThread)
        {
            zedLogDebug("Grab thread stopped");
            break;
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

    zedLogFunc("TODO");

    return ONI_STATUS_OK;
}

} } // namespace driver // namespace oni
