#ifndef ZEDDRIVER_HPP
#define ZEDDRIVER_HPP

#include <Driver/OniDriverAPI.h>

#include <memory>
#include <mutex>
#include <vector>

#include <sl/Camera.hpp>

#include "zeddevice.hpp"

namespace oni {
namespace driver {

class ZedDriver : public DriverBase
{
public:
    ZedDriver(OniDriverServices* driverServices);
    virtual ~ZedDriver();

    virtual OniStatus initialize(
        DeviceConnectedCallback connectedCallback,
        DeviceDisconnectedCallback disconnectedCallback,
        DeviceStateChangedCallback deviceStateChangedCallback,
        void* cookie);

    virtual void shutdown();

    virtual DeviceBase* deviceOpen(const char* uri, const char* mode);
    virtual void deviceClose(DeviceBase* deviceBase);
    virtual OniStatus tryDevice(const char* uri);

    virtual void* enableFrameSync(StreamBase** streams, int streamCount);
    virtual void disableFrameSync(void* frameSyncGroup);

protected:
    size_t enumerateDevices();

    ZedDriver(const ZedDriver&);
    void operator=(const ZedDriver&);

private:
    std::mutex mStateMutex;
    std::mutex mDevicesMutex;

    sl::Camera mZed;

    std::vector<sl::DeviceProperties> mZedDevList;

    std::map<unsigned int, std::shared_ptr<ZedDevice>> mDevices;
};

#if !defined(XN_NEW)
#define XN_NEW(type, arg) new type(arg)
#endif

#if !defined(XN_DELETE)
#define XN_DELETE(arg) delete arg
#endif

ONI_EXPORT_DRIVER(ZedDriver)

} // namespace driver
} // namespace oni

#endif
