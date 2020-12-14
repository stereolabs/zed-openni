#ifndef ZEDDEVICE_HPP
#define ZEDDEVICE_HPP

#include <Driver/OniDriverAPI.h>

#include "zeddevice.hpp"

class ZedDevice : public oni::driver::DeviceBase {
    friend class ZedDriver;

public:
    ZedDevice(class ZedDriver* driver, class sl::Camera* zed);
    virtual ~ZedDevice();

};

#endif
