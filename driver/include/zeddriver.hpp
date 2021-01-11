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
