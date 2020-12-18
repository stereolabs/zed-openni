#include "zeddevice.hpp"


namespace oni { namespace driver {

ZedDevice::ZedDevice(class ZedDriver* driver, sl::DeviceProperties& prop)
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

} } // namespace driver // namespace oni
