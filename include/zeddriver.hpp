#ifndef ZEDDRIVER_HPP
#define ZEDDRIVER_HPP

#include <Driver/OniDriverAPI.h>
#include <memory>
#include <mutex>
#include <vector>

#include <sl/Camera.hpp>

namespace oni {
namespace driver {

class ZedDriver : public oni::driver::DriverBase
{

public:
    ZedDriver(OniDriverServices* driverServices);
    virtual ~ZedDriver();

    virtual OniStatus initialize(
        DeviceConnectedCallback connectedCallback,
        DeviceDisconnectedCallback disconnectedCallback,
        DeviceStateChangedCallback deviceStateChangedCallback,
        void* cookie);

protected:
    void enumerateDevices();

    ZedDriver(const ZedDriver&);
    void operator=(const ZedDriver&);

private:
    std::mutex mStateMutex;
    std::mutex mDevicesMutex;

    std::vector<sl::DeviceProperties> mZedDevList;

    std::map<std::string, std::shared_ptr<ZedDevice>> mDevices;
};

} // namespace driver
} // namespace oni

#endif
