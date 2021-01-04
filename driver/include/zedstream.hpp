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
    int stride;
    int width;
    int height;
    int framerate;
};


class ZedStream : public StreamBase {
    friend class ZedDevice;

public:
    ZedStream(OniSensorType sensorType);
    virtual ~ZedStream();

    virtual OniStatus setProperty(int propertyId, const void* data, int dataSize) override;
    virtual OniStatus getProperty(int propertyId, void* data, int* dataSize) override;
    virtual OniBool isPropertySupported(int propertyId) override;

    inline bool isEnabled() const { return mEnabled; }

    virtual OniStatus start();
    virtual void stop();

    inline std::shared_ptr<ZedDevice> getDevice() { return mDevice; }

    inline OniSensorType getOniType() const { return mOniType; }
    inline OniVideoMode getVideoMode() const { return mVideoMode; }
    inline ZedStreamProfileInfo getProfile() const { return mProfile; }
    inline int getSensorId() const { return mSensorId; }

protected:
    ZedStream(const ZedStream&);
    void operator=(const ZedStream&);

    OniStatus initialize(std::shared_ptr<ZedDevice> device, int sensorId,
                         int profileId, std::vector<ZedStreamProfileInfo>* profiles);
    void shutdown();

    bool getTable(void* dst, int* size, const std::vector<uint16_t>& table);
    bool setTable(const void* src, int size, std::vector<uint16_t>& table);

private:
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
