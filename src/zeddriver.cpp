#include "zeddriver.hpp"

#include "zedtools.hpp"

namespace oni { namespace driver {

ZedDriver::ZedDriver(OniDriverServices *driverServices)
    : DriverBase(driverServices)
{
    zedLogDebug("+ZedDriver");
}

ZedDriver::~ZedDriver()
{
    zedLogDebug("~ZedDriver");

    if(mDevices.size()>0)
    {
        shutdown();
    }
}

void ZedDriver::shutdown()
{
    zedLogFunc("");

    std::lock_guard<std::mutex> lock(mStateMutex);

    mDevices.clear();
}

OniStatus ZedDriver::initialize(
        DeviceConnectedCallback connectedCallback,
        DeviceDisconnectedCallback disconnectedCallback,
        DeviceStateChangedCallback deviceStateChangedCallback,
        void* cookie)
{
    zedLogFunc("");

    std::lock_guard<std::mutex> lock(mStateMutex);

    if (DriverBase::initialize(connectedCallback, disconnectedCallback, deviceStateChangedCallback, cookie) != ONI_STATUS_OK)
    {
        zedLogError("DriverBase::initialize failed");
        return ONI_STATUS_ERROR;
    }

    if(enumerateDevices()>0)
    {
        zedLogDebug("initialized");
        return ONI_STATUS_OK;
    }

    zedLogError( "No ZED device detected.");

    shutdown();
    return ONI_STATUS_ERROR;
}


size_t ZedDriver::enumerateDevices()
{
    zedLogFunc("");

    std::lock_guard<std::mutex> lock(mDevicesMutex);

    mZedDevList = sl::Camera::getDeviceList();

    zedLogDebug("Found %zu ZED devices", mZedDevList.size());

    return mZedDevList.size();
}

DeviceBase* ZedDriver::deviceOpen(const char* uri, const char* mode)
{
    zedLogFunc("");

    unsigned int serial_number = 0;

    if(uri!=nullptr)
    {
        serial_number = std::stoi( std::string(uri));
    }

    zedLogDebug("Serial Number: %u", serial_number);

    std::vector<sl::DeviceProperties>::iterator it;
    bool found = false;

    for( it=mZedDevList.begin(); it!=mZedDevList.end(); it++)
    {
        std::lock_guard<std::mutex> lock(mDevicesMutex);

        sl::DeviceProperties prop = *it;

        if(serial_number!=0 && prop.serial_number!=serial_number)
            continue;

        if(prop.camera_state == sl::CAMERA_STATE::NOT_AVAILABLE)
        {
            zedLogDebug("Camera with sn %u is busy", prop.serial_number);
            continue;
        }

        std::shared_ptr<ZedDevice> zedDevice = std::make_shared<ZedDevice>(this, prop);

        if (zedDevice->initialize() != ONI_STATUS_OK)
        {
            zedLogError("ZedDevice::initialize failed");
        }
        else
        {
            mDevices[prop.serial_number] = zedDevice;
            found = true;

            zedLogDebug("Camera model:\t%s", sl::toString(prop.camera_model).c_str());
            zedLogDebug("Camera serial number:\t%u", prop.serial_number);
            zedLogDebug("Camera ID:\t%d", prop.id);

            return zedDevice.get();
        }

        if(serial_number==0)
        {
            continue; // Try to open the next ZED Camera
        }

        break;
    }

    return nullptr;
}

void ZedDriver::deviceClose(DeviceBase* deviceBase)
{
    zedLogFunc("");
}

OniStatus ZedDriver::tryDevice(const char* uri)
{
    zedLogFunc("");
}

void* ZedDriver::enableFrameSync(StreamBase** streams, int streamCount)
{
    zedLogFunc("");
}

void ZedDriver::disableFrameSync(void* frameSyncGroup)
{
    zedLogFunc("");
}



} // namespace driver
              } // namespace oni
