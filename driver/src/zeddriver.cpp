#include "zeddriver.hpp"
#include "zedtools.hpp"


namespace oni { namespace driver {

ZedDriver::ZedDriver(OniDriverServices *driverServices)
    : DriverBase(driverServices)
{
#ifndef NDEBUG
    zedLogDebug("+ZedDriver");
#endif
}

ZedDriver::~ZedDriver()
{
#ifndef NDEBUG
    zedLogDebug("~ZedDriver");
#endif

    if(mDevices.size()>0)
    {
        shutdown();
    }
}

void ZedDriver::shutdown()
{
#ifndef NDEBUG
    zedLogFunc("");
#endif

    std::lock_guard<std::mutex> lock(mStateMutex);

    mDevices.clear();
}

OniStatus ZedDriver::initialize(
        DeviceConnectedCallback connectedCallback,
        DeviceDisconnectedCallback disconnectedCallback,
        DeviceStateChangedCallback deviceStateChangedCallback,
        void* cookie)
{
#ifndef NDEBUG
    zedLogFunc("");
#endif

    std::lock_guard<std::mutex> lock(mStateMutex);

    if (DriverBase::initialize(connectedCallback, disconnectedCallback, deviceStateChangedCallback, cookie) != ONI_STATUS_OK)
    {
        zedLogError("DriverBase::initialize failed");
        return ONI_STATUS_ERROR;
    }
#ifndef NDEBUG
    zedLogDebug("initialized");
#endif

    enumerateDevices();

    for(size_t i=0; i<mZedDevList.size(); i++)
    {
        if(mZedDevList[i].camera_state==sl::CAMERA_STATE::NOT_AVAILABLE)
        {
            continue;
        }

        OniDeviceInfo info;

        std::string uri = std::to_string(mZedDevList[i].serial_number);
        uri += '\0';
        strncpy(info.uri, uri.c_str(), uri.size());

#ifdef EMULATE_PRIMESENSE_HARDWARE
        zedLogDebug("Hack! Emulating PrimeSense device to be able to use NiTE");
        strncpy(info.name, "PS1080", sizeof(info.name) - 1);
        strncpy(info.vendor, "PrimeSense", sizeof(info.vendor) - 1);
        info.usbVendorId = 7463;
        info.usbProductId = 1537;
#else
        sl::String camModel = sl::toString(mZedDevList[i].camera_model);
        strncpy(info.name, camModel.c_str(), camModel.size());
        std::string vendor = "Stereolabs";
        vendor += '\0';
        strncpy(info.vendor, vendor.c_str(), vendor.size());

        info.usbVendorId = SL_USB_VENDOR;

        switch(mZedDevList[i].camera_model)
        {
        case sl::MODEL::ZED:
            info.usbProductId = SL_USB_PROD_ZED;
            break;
        case sl::MODEL::ZED_M:
            info.usbProductId = SL_USB_PROD_ZED_M;
            break;
        case sl::MODEL::ZED2:
            info.usbProductId = SL_USB_PROD_ZED_2;
            break;
        default:
            zedLogError("Camera model not valid");
            return ONI_STATUS_ERROR;
        }
#endif

        zedLogDebug("Device found: %s [%u] - Id: %d",
                    sl::toString(mZedDevList[i].camera_model).c_str(),
                    mZedDevList[i].serial_number, mZedDevList[i].id );

        // Notify OpenNI that a ZED is available
        deviceConnected(&info);
    }

    return ONI_STATUS_OK;
}


size_t ZedDriver::enumerateDevices()
{
#ifndef NDEBUG
    zedLogFunc("");
#endif

    std::lock_guard<std::mutex> lock(mDevicesMutex);

    mZedDevList = sl::Camera::getDeviceList();

    zedLogDebug("Found %zu ZED devices", mZedDevList.size());

    return mZedDevList.size();
}

DeviceBase* ZedDriver::deviceOpen(const char* uri, const char* mode)
{
#ifndef NDEBUG
    zedLogFunc("");
#endif

    unsigned int serial_number = 0;

    if(uri!=nullptr)
    {
        serial_number = std::stoi( std::string(uri));
    }

    zedLogDebug("Opening camera with serial Number: %u", serial_number);

    std::vector<sl::DeviceProperties>::iterator it;

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

            zedLogDebug("Device opened: %s [%u] - Id: %d",
                        sl::toString(prop.camera_model).c_str(), prop.serial_number, prop.id );

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
#ifndef NDEBUG
    zedLogFunc("");
#endif
}

OniStatus ZedDriver::tryDevice(const char* uri)
{
#ifndef NDEBUG
    zedLogFunc("");
#endif
}

void* ZedDriver::enableFrameSync(StreamBase** streams, int streamCount)
{
#ifndef NDEBUG
    zedLogFunc("");
#endif
}

void ZedDriver::disableFrameSync(void* frameSyncGroup)
{
#ifndef NDEBUG
    zedLogFunc("");
#endif
}



} } // namespace driver // namespace oni
