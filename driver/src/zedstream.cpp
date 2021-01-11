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

#include "zedstream.hpp"
#include "zeddevice.hpp"
#include "PS1080.h"
#include "XnDepthShiftTables.h"

namespace oni { namespace driver {

ZedStream::ZedStream(OniSensorType sensorType, bool verbose)
    : mVerbose(verbose)
{
    if(mVerbose)
        zedLogDebug("+ZedStream sensorType: %s", streamType2Str(sensorType).c_str());

    mZedType = convertStreamType(sensorType);
    mOniType = sensorType;
}

ZedStream::~ZedStream()
{
    if(mVerbose)
        zedLogFunc("~Rs2Stream type=%d", (int)mZedType);

    shutdown();
}

OniStatus ZedStream::initialize(std::shared_ptr<ZedDevice> device, int sensorId,
                                int profileId, const std::vector<ZedStreamProfileInfo> *profiles)
{
    if(mVerbose)
        zedLogFunc("sensorId=%d profileId=%d", sensorId, profileId);

    mDevice = device;
    mSensorId = sensorId;
    mProfiles = *profiles;

    memset(&mVideoMode, 0, sizeof(mVideoMode));

    for (auto iter = mProfiles.begin(); iter != mProfiles.end(); ++iter)
    {
        ZedStreamProfileInfo& sp = *iter;

        if(sp.profileId==profileId)
        {
            mProfile = sp;
            break;
        }
    }

    if(mDevice->mRightMeasure)
    {
        mFovX = device->mZed.getCameraInformation().calibration_parameters.left_cam.h_fov;
        mFovY = device->mZed.getCameraInformation().calibration_parameters.left_cam.v_fov;
    }
    else
    {
        mFovX = device->mZed.getCameraInformation().calibration_parameters.right_cam.h_fov;
        mFovY = device->mZed.getCameraInformation().calibration_parameters.right_cam.v_fov;
    }

    mVideoMode.fps = mProfile.framerate;
    mVideoMode.pixelFormat = mProfile.format;
    mVideoMode.resolutionX = mProfile.width;
    mVideoMode.resolutionY = mProfile.height;

    if (mOniType == ONI_SENSOR_DEPTH)
    {
        // TODO: these tables should be calculated depending on sensor parameters, using hardcoded values as workaround for NITE
        setTable(S2D, sizeof(S2D), m_s2d); // XN_STREAM_PROPERTY_S2D_TABLE
        setTable(D2S, sizeof(D2S), m_d2s); // XN_STREAM_PROPERTY_D2S_TABLE
    }

    return ONI_STATUS_OK;
}

void ZedStream::shutdown()
{
    if(mVerbose)
        zedLogFunc("");
}

OniStatus ZedStream::start()
{
    if(!mEnabled)
    {
        if(mVerbose)
            zedLogFunc("type=%d sensorId=%d porfileId=%d", mZedType, mSensorId, mProfile.profileId);
        mEnabled = true;
    }

    return ONI_STATUS_OK;
}

void ZedStream::stop()
{
    if (mEnabled)
    {
        if(mVerbose)
            zedLogFunc("type=%d sensorId=%d porfileId=%d", mZedType, mSensorId, mProfile.profileId);
        mEnabled = false;
    }
}

int ZedStream::isVideoModeSupported(OniVideoMode* mode)
{
    for(size_t spiIdx=0; spiIdx<mProfiles.size(); spiIdx++)
    {
        ZedStreamProfileInfo spi = mProfiles[spiIdx];

        if(spi.width==mode->resolutionX &&
                spi.height==mode->resolutionY &&
                spi.framerate==mode->fps &&
                spi.format==mode->pixelFormat)
        {
            return spiIdx;
        }
    }
    return -1;
}

OniStatus ZedStream::setProperty(int propertyId, const void* data, int dataSize)
{
    if(mVerbose)
        zedLogFunc("propertyId=%d dataSize=%d", propertyId, dataSize);

    switch (propertyId)
    {
    //    case ONI_STREAM_PROPERTY_VIDEO_MODE:
    //    {
    //        if (data && (dataSize == sizeof(OniVideoMode)))
    //        {
    //            OniVideoMode* mode = (OniVideoMode*)data;
    //            zedLogFunc("set video mode: %dx%d @%d format=%d",
    //                       (int)mode->resolutionX, (int)mode->resolutionY, (int)mode->fps, (int)mode->pixelFormat);

    //            int spiIdx =isVideoModeSupported(mode);
    //            if(spiIdx!=-1)
    //            {
    //                getDevice()->changeVideoMode(&mProfiles[spiIdx]);
    //                return ONI_STATUS_OK;
    //            }
    //        }
    //        break;
    //    }

    case ONI_STREAM_PROPERTY_AUTO_WHITE_BALANCE:
    {
        if (data && dataSize == sizeof(OniBool) )
        {
            bool enable = (bool)(*(OniBool*)data);
            int setValue = enable?1:0;
            mDevice->mZed.setCameraSettings( sl::VIDEO_SETTINGS::WHITEBALANCE_AUTO, setValue);

            int getValue = mDevice->mZed.getCameraSettings(sl::VIDEO_SETTINGS::WHITEBALANCE_AUTO);

            if(getValue==setValue)
                return ONI_STATUS_OK;
            else
                return ONI_STATUS_ERROR;
        }
        break;
    }

    case ONI_STREAM_PROPERTY_AUTO_EXPOSURE:
    {
        if (data && dataSize == sizeof(OniBool))
        {
            bool enable = (bool)(*(OniBool*)data);
            int setValue = enable?1:0;
            mDevice->mZed.setCameraSettings( sl::VIDEO_SETTINGS::AEC_AGC, setValue);

            int getValue = mDevice->mZed.getCameraSettings(sl::VIDEO_SETTINGS::AEC_AGC);

            if(getValue==setValue)
                return ONI_STATUS_OK;
            else
                return ONI_STATUS_ERROR;
        }
        break;
    }

    case ONI_STREAM_PROPERTY_EXPOSURE:
    {
        if (data && dataSize == sizeof(int))
        {
            int setValue = *((int*)data);

            if((setValue<0) || (setValue>100))
            {
                zedLogError("Invalid Exposure value. Valid range: [0,100]");
                return ONI_STATUS_ERROR;
            }

            mDevice->mZed.setCameraSettings( sl::VIDEO_SETTINGS::EXPOSURE, setValue);

            int getValue = mDevice->mZed.getCameraSettings(sl::VIDEO_SETTINGS::EXPOSURE);

            if(getValue==setValue)
                return ONI_STATUS_OK;
            else
                return ONI_STATUS_ERROR;
        }
        break;
    }

    case ONI_STREAM_PROPERTY_GAIN:
    {
        if (data && dataSize == sizeof(int))
        {
            int setValue = *((int*)data);

            if((setValue<0) || (setValue>100))
            {
                zedLogError("Invalid Gain value. Valid range: [0,100]");
                return ONI_STATUS_ERROR;
            }

            mDevice->mZed.setCameraSettings( sl::VIDEO_SETTINGS::GAIN, setValue);

            int getValue = mDevice->mZed.getCameraSettings(sl::VIDEO_SETTINGS::GAIN);

            if(getValue==setValue)
                return ONI_STATUS_OK;
            else
                return ONI_STATUS_ERROR;
        }
        break;
    }

    case XN_STREAM_PROPERTY_S2D_TABLE:
    {
        if (data && mOniType == ONI_SENSOR_DEPTH)
        {
            if (setTable(data, dataSize, m_s2d))
            {
                return ONI_STATUS_OK;
            }
        }
        break;
    }

    case XN_STREAM_PROPERTY_D2S_TABLE:
    {
        if (data && mOniType == ONI_SENSOR_DEPTH)
        {
            if (setTable(data, dataSize, m_d2s))
            {
                return ONI_STATUS_OK;
            }
        }
        break;
    }

    default:
    {
        zedLogError("Not supported: propertyId=%d", propertyId);
        return ONI_STATUS_NOT_SUPPORTED;
    }
    }

    zedLogError("propertyId=%d dataSize=%d", propertyId, dataSize);
    return ONI_STATUS_ERROR;
}

OniStatus ZedStream::getProperty(int propertyId, void* data, int* dataSize)
{
#if 0
    zedLogFunc("propertyId=%d dataSize=%d", propertyId, *dataSize);
#endif

    switch (propertyId)
    {
    case ONI_STREAM_PROPERTY_CROPPING:
    {
        if (data && dataSize && *dataSize == sizeof(OniCropping))
        {
            OniCropping value;
            value.enabled = false;
            value.originX = 0;
            value.originY = 0;
            value.width = mVideoMode.resolutionX;
            value.height = mVideoMode.resolutionY;
            *((OniCropping*)data) = value;
#if 0
            zedLogFunc("\t Cropping: (%d,%d) %dx%d",value.originX,value.originY,value.width,value.height);
#endif
            return ONI_STATUS_OK;
        }
        break;
    }

    case ONI_STREAM_PROPERTY_HORIZONTAL_FOV:
    {
        if (data && dataSize && *dataSize == sizeof(float))
        {
            float val = mFovX * 0.01745329251994329576923690768489f;
            *((float*)data) = val;
#if 0
            zedLogFunc("\t H_FOV: %g",val);
#endif
            return ONI_STATUS_OK;
        }
        break;
    }

    case ONI_STREAM_PROPERTY_VERTICAL_FOV:
    {
        if (data && dataSize && *dataSize == sizeof(float))
        {
            float val = mFovY * 0.01745329251994329576923690768489f;
            *((float*)data) = val;
#if 0
            zedLogFunc("\t V_FOV: %g",val);
#endif
            return ONI_STATUS_OK;
        }
        break;
    }

    case ONI_STREAM_PROPERTY_VIDEO_MODE:
    {
        if (data && dataSize && *dataSize == sizeof(OniVideoMode))
        {
            *((OniVideoMode*)data) = mVideoMode;
            if(mVerbose)
                zedLogFunc("\t OniVideoMode: Format: %d, FPS: %d, %dx%d",(int)mVideoMode.pixelFormat,mVideoMode.fps, mVideoMode.resolutionX, mVideoMode.resolutionY);

            return ONI_STATUS_OK;
        }
        break;
    }

    case ONI_STREAM_PROPERTY_MAX_VALUE:
    {
        if (data && dataSize && *dataSize == sizeof(int) && mOniType == ONI_SENSOR_DEPTH)
        {
            *((int*)data) = ONI_MAX_DEPTH;
            return ONI_STATUS_OK;
        }
        break;
    }

    case ONI_STREAM_PROPERTY_MIN_VALUE:
    {
        if (data && dataSize && *dataSize == sizeof(int) && mOniType == ONI_SENSOR_DEPTH)
        {
            *((int*)data) = 0;
            return ONI_STATUS_OK;
        }
        break;
    }

    case ONI_STREAM_PROPERTY_STRIDE:
    {
        if (data && dataSize && *dataSize == sizeof(int))
        {
            int val = mVideoMode.resolutionX * getPixelFormatBytes(mVideoMode.pixelFormat);
            *((int*)data) = val;
#if 0
            zedLogFunc("\t Stride: %d",val);
#endif
            return ONI_STATUS_OK;
        }
        break;
    }

    case ONI_STREAM_PROPERTY_MIRRORING:
    {
        if (data && dataSize && *dataSize == sizeof(OniBool))
        {
            *((OniBool*)data) = false;
            return ONI_STATUS_OK;
        }
        break;
    }

    case ONI_STREAM_PROPERTY_AUTO_WHITE_BALANCE:
    {
        if (data && dataSize && *dataSize == sizeof(OniBool))
        {
            int getValue = mDevice->mZed.getCameraSettings(sl::VIDEO_SETTINGS::WHITEBALANCE_AUTO);
            *((OniBool*)data) = (int)getValue ? true : false;

            return ONI_STATUS_OK;
        }
        break;
    }

    case ONI_STREAM_PROPERTY_AUTO_EXPOSURE:
    {
        if (data && dataSize && *dataSize == sizeof(OniBool))
        {
            int getValue = mDevice->mZed.getCameraSettings(sl::VIDEO_SETTINGS::AEC_AGC);
            *((OniBool*)data) = (int)getValue ? true : false;

            return ONI_STATUS_OK;
        }
        break;
    }

    case ONI_STREAM_PROPERTY_EXPOSURE:
    {
        if (data && dataSize && *dataSize == sizeof(int))
        {
            int getValue = mDevice->mZed.getCameraSettings(sl::VIDEO_SETTINGS::EXPOSURE);
            *((int*)data) = (int)getValue;

            return ONI_STATUS_OK;
        }
        break;
    }

    case ONI_STREAM_PROPERTY_GAIN:
    {
        if (data && dataSize && *dataSize == sizeof(int))
        {
            int getValue = mDevice->mZed.getCameraSettings(sl::VIDEO_SETTINGS::GAIN);
            *((int*)data) = (int)getValue;

            return ONI_STATUS_OK;
        }
        break;
    }

    case XN_STREAM_PROPERTY_S2D_TABLE:
    {
        if (data && dataSize && mOniType == ONI_SENSOR_DEPTH)
        {
            if (getTable(data, dataSize, m_s2d))
            {
                return ONI_STATUS_OK;
            }
        }
        break;
    }

    case XN_STREAM_PROPERTY_D2S_TABLE:
    {
        if (data && dataSize && mOniType == ONI_SENSOR_DEPTH)
        {
            if (getTable(data, dataSize, m_d2s))
            {
                return ONI_STATUS_OK;
            }
        }
        break;
    }

    default:
    {
        zedLogError("Not supported: propertyId=%d", propertyId);
        return ONI_STATUS_NOT_SUPPORTED;
    }
    }

    zedLogError("propertyId=%d dataSize=%d", propertyId, *dataSize);
    return ONI_STATUS_ERROR;
}

OniBool ZedStream::isPropertySupported(int propertyId)
{
    if(mVerbose)
        zedLogFunc("propertyId=%d", propertyId);

    switch (propertyId)
    {
    case ONI_STREAM_PROPERTY_CROPPING:				// OniCropping*
    case ONI_STREAM_PROPERTY_HORIZONTAL_FOV:		// float: radians
    case ONI_STREAM_PROPERTY_VERTICAL_FOV:			// float: radians
    case ONI_STREAM_PROPERTY_VIDEO_MODE:			// OniVideoMode*
    case ONI_STREAM_PROPERTY_MAX_VALUE:				// int
    case ONI_STREAM_PROPERTY_MIN_VALUE:				// int
    case ONI_STREAM_PROPERTY_STRIDE:				// int
    case ONI_STREAM_PROPERTY_MIRRORING:				// OniBool
        return true;

    case ONI_STREAM_PROPERTY_NUMBER_OF_FRAMES:		// int
        return false;

    case ONI_STREAM_PROPERTY_AUTO_WHITE_BALANCE:	// OniBool
    case ONI_STREAM_PROPERTY_AUTO_EXPOSURE:			// OniBool
    case ONI_STREAM_PROPERTY_EXPOSURE:				// int
    case ONI_STREAM_PROPERTY_GAIN:					// int
        return true;

    case XN_STREAM_PROPERTY_S2D_TABLE:
    case XN_STREAM_PROPERTY_D2S_TABLE:
        return true;

    default:
        return false;
    }
}

bool ZedStream::getTable(void* dst, int* size, const std::vector<uint16_t>& table)
{
    const int tableSize = (int)(table.size() * sizeof(uint16_t));
    if (dst && size && *size >= tableSize)
    {
        if (tableSize > 0)
        {
            memcpy(dst, &table[0], tableSize);
        }
        *size = tableSize;
        return true;
    }
    return false;
}

bool ZedStream::setTable(const void* src, int size, std::vector<uint16_t>& table)
{
    if (src && size >= 0)
    {
        const int elemCount = size / sizeof(uint16_t);
        if (elemCount > 0)
        {
            table.resize(elemCount);
            memcpy(&table[0], src, elemCount * sizeof(uint16_t));
        }
        else
        {
            table.clear();
        }
        return true;
    }
    return false;
}

} } // namespace driver // namespace oni
