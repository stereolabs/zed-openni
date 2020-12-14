#include "zeddriver.hpp"

#include <iostream>

namespace oni
{
namespace driver
{

ZedDriver::ZedDriver(OniDriverServices *driverServices)
    : DriverBase(driverServices)
{

}

ZedDriver::~ZedDriver()
{
    shutdown();
}

OniStatus ZedDriver::initialize(
    DeviceConnectedCallback connectedCallback,
    DeviceDisconnectedCallback disconnectedCallback,
    DeviceStateChangedCallback deviceStateChangedCallback,
    void* cookie)
{
    for(;;)
    {
        std::lock_guard(mStateMutex);

        if (DriverBase::initialize(connectedCallback, disconnectedCallback, deviceStateChangedCallback, cookie) != ONI_STATUS_OK)
        {
            std::cerr << "DriverBase::initialize failed" << std::endl;
            break;
        }

        enumerateDevices();

        rs2_set_devices_changed_callback(m_context, devicesChangedCallback, this, &e);
        if (!e.success())
        {
            rsTraceError("rs2_set_devices_changed_callback failed: %s", e.get_message());
            break;
        }

        rsLogDebug("Rs2Driver INITIALIZED");
        return ONI_STATUS_OK;
    }

    shutdown();
    return ONI_STATUS_ERROR;
}


ZedDriver::enumerateDevices()
{

}

} // namespace driver
} // namespace oni
