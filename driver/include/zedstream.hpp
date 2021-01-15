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

#ifndef ZEDSTREAM_HPP
#define ZEDSTREAM_HPP

#include <Driver/OniDriverAPI.h>

#include "zedtools.hpp"

namespace oni { namespace driver {

class ZedDevice;

struct ZedStreamProfileInfo
{
    sl::RESOLUTION zedRes;
    int sensorId;
    int profileId;
    OniSensorType streamType;
    OniPixelFormat format;
    size_t stride;
    size_t width;
    size_t height;
    int framerate;
};


class ZedStream : public StreamBase {
    friend class ZedDevice;

public:
    ZedStream(OniSensorType sensorType, bool verbose=false);
    virtual ~ZedStream();

    virtual OniStatus setProperty(int propertyId, const void* data, int dataSize) override;
    virtual OniStatus getProperty(int propertyId, void* data, int* dataSize) override;
    virtual OniBool isPropertySupported(int propertyId) override;

    inline bool isEnabled() const { return mEnabled; }

    virtual OniStatus start() override;
    virtual void stop() override;

    inline std::shared_ptr<ZedDevice> getDevice() { return mDevice; }

    inline OniSensorType getOniType() const { return mOniType; }
    inline OniVideoMode getVideoMode() const { return mVideoMode; }
    inline ZedStreamProfileInfo getProfile() const { return mProfile; }
    inline const std::vector<ZedStreamProfileInfo>* getProfiles() const {return &mProfiles;}
    inline int getSensorId() const { return mSensorId; }

protected:
    ZedStream(const ZedStream&);
    void operator=(const ZedStream&);

    OniStatus initialize(std::shared_ptr<ZedDevice> device, int sensorId,
                         int profileId, const std::vector<ZedStreamProfileInfo> *profiles);
    void shutdown();

    bool getTable(void* dst, int* size, const std::vector<uint16_t>& table);
    bool setTable(const void* src, int size, std::vector<uint16_t>& table);

    int isVideoModeSupported(OniVideoMode *mode);

private:
    bool mVerbose = false;

    std::shared_ptr<ZedDevice> mDevice;

    int mSensorId;
    bool mEnabled = false;

    ZedStreamProfileInfo mProfile;
    std::vector<ZedStreamProfileInfo> mProfiles;

    int mZedType;
    OniSensorType mOniType;
    OniVideoMode mVideoMode;

    float mFovX;
    float mFovY;

    std::vector<uint16_t> m_s2d;
    std::vector<uint16_t> m_d2s;
};

class ZedDepthStream : public ZedStream
{
public:
    ZedDepthStream() : ZedStream(ONI_SENSOR_DEPTH) {}
};

class ZedColorStream : public ZedStream
{
public:
    ZedColorStream() : ZedStream(ONI_SENSOR_COLOR) {}
};

class ZedGrayStream : public ZedStream
{
public:
    ZedGrayStream() : ZedStream(ONI_SENSOR_IR) {}
};

} } // namespace driver // namespace oni

#endif
