#include "zedstream.hpp"
#include "zeddevice.hpp"
#include "PS1080.h"
#include "XnDepthShiftTables.h"

namespace oni { namespace driver {

ZedStream::ZedStream(OniSensorType sensorType)
{
    zedLogDebug("+ZedStream sensorType: %s", streamType2Str(sensorType).c_str());
    mZedType = convertStreamType(sensorType);
    mOniType = sensorType;

}

ZedStream::~ZedStream()
{
    zedLogFunc("~Rs2Stream type=%d", (int)mZedType);

    shutdown();
}

OniStatus ZedStream::initialize(std::shared_ptr<ZedDevice> device, int sensorId,
                     int streamId, std::vector<ZedStreamProfileInfo> *profiles)
{
    zedLogFunc("type=%d sensorId=%d streamId=%d", mZedType, sensorId, streamId);

    mDevice = device;
    mSensorId = sensorId;
    mStreamId = streamId;
    mProfiles = *profiles;

    memset(&mVideoMode, 0, sizeof(mVideoMode));

    for (auto iter = mProfiles.begin(); iter != mProfiles.end(); ++iter)
    {
        ZedStreamProfileInfo& sp = *iter;

        if(sp.streamId==streamId)
        {
            mProfile = sp;
            break;
        }
    }

    mFovX = device->mZed.getCameraInformation().calibration_parameters.left_cam.h_fov;
    mFovY = device->mZed.getCameraInformation().calibration_parameters.left_cam.v_fov;

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
    // TODO
    zedLogFunc("");
}

OniStatus ZedStream::start()
{
    if(!mEnabled)
    {
        zedLogFunc("type=%d sensorId=%d streamId=%d", mZedType, mSensorId, mStreamId);
        mEnabled = true;
        getDevice()->updateConfiguration();
    }

    return ONI_STATUS_OK;
}

void ZedStream::stop()
{
    if (mEnabled)
    {
        zedLogFunc("type=%d sensorId=%d streamId=%d", mZedType, mSensorId, mStreamId);
        mEnabled = false;
        getDevice()->updateConfiguration();
    }
}

OniStatus ZedStream::setProperty(int propertyId, const void* data, int dataSize)
{
    zedLogFunc("propertyId=%d dataSize=%d", propertyId, dataSize);

    switch (propertyId)
    {
//		case ONI_STREAM_PROPERTY_VIDEO_MODE:
//		{
//			if (data && (dataSize == sizeof(OniVideoMode)))
//			{
//				OniVideoMode* mode = (OniVideoMode*)data;
//				rsLogDebug("set video mode: %dx%d @%d format=%d",
//					(int)mode->resolutionX, (int)mode->resolutionY, (int)mode->fps, (int)mode->pixelFormat);

//				if (isVideoModeSupported(mode))
//				{
//					m_videoMode = *mode;
//					return ONI_STATUS_OK;
//				}
//			}
//			break;
//		}

//		case ONI_STREAM_PROPERTY_AUTO_WHITE_BALANCE:
//		{
//			if (data && dataSize == sizeof(OniBool) && m_oniType == ONI_SENSOR_COLOR)
//			{
//				Rs2Error e;
//				float value = (float)*((OniBool*)data);
//				rs2_set_option((const rs2_options*)m_sensor, RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, value, &e);
//				if (e.success()) return ONI_STATUS_OK;
//			}
//			break;
//		}

//		case ONI_STREAM_PROPERTY_AUTO_EXPOSURE:
//		{
//			if (data && dataSize == sizeof(OniBool) && m_oniType == ONI_SENSOR_COLOR)
//			{
//				Rs2Error e;
//				float value = (float)*((OniBool*)data);
//				rs2_set_option((const rs2_options*)m_sensor, RS2_OPTION_ENABLE_AUTO_EXPOSURE, value, &e);
//				if (e.success()) return ONI_STATUS_OK;
//			}
//			break;
//		}

//		case ONI_STREAM_PROPERTY_EXPOSURE:
//		{
//			if (data && dataSize == sizeof(int) && m_oniType == ONI_SENSOR_COLOR)
//			{
//				Rs2Error e;
//				float value = (float)*((int*)data);
//				rs2_set_option((const rs2_options*)m_sensor, RS2_OPTION_EXPOSURE, value, &e);
//				if (e.success()) return ONI_STATUS_OK;
//			}
//			break;
//		}

//		case ONI_STREAM_PROPERTY_GAIN:
//		{
//			if (data && dataSize == sizeof(int) && m_oniType == ONI_SENSOR_COLOR)
//			{
//				Rs2Error e;
//				float value = (float)*((int*)data);
//				rs2_set_option((const rs2_options*)m_sensor, RS2_OPTION_GAIN, value, &e);
//				if (e.success()) return ONI_STATUS_OK;
//			}
//			break;
//		}

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
    zedLogFunc("propertyId=%d dataSize=%d", propertyId, *dataSize);

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
                zedLogFunc("\t Cropping: (%d,%d) %dx%d",value.originX,value.originY,value.width,value.height);
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
                zedLogFunc("\t H_FOV: %g",val);
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
                zedLogFunc("\t V_FOV: %g",val);
                return ONI_STATUS_OK;
            }
            break;
        }

        case ONI_STREAM_PROPERTY_VIDEO_MODE:
        {
            if (data && dataSize && *dataSize == sizeof(OniVideoMode))
            {
                *((OniVideoMode*)data) = mVideoMode;
                zedLogFunc("\t OniVideoMode: Format: %d, FPS: %d, %dx%d",(int)mVideoMode.pixelFormat,mVideoMode.fps, mVideoMode.resolutionX, mVideoMode.resolutionY);
                return ONI_STATUS_OK;
            }
            break;
        }

//		case ONI_STREAM_PROPERTY_MAX_VALUE:
//		{
//			if (data && dataSize && *dataSize == sizeof(int) && m_oniType == ONI_SENSOR_DEPTH)
//			{
//				*((int*)data) = ONI_MAX_DEPTH;
//				return ONI_STATUS_OK;
//			}
//			break;
//		}

//		case ONI_STREAM_PROPERTY_MIN_VALUE:
//		{
//			if (data && dataSize && *dataSize == sizeof(int) && m_oniType == ONI_SENSOR_DEPTH)
//			{
//				*((int*)data) = 0;
//				return ONI_STATUS_OK;
//			}
//			break;
//		}

        case ONI_STREAM_PROPERTY_STRIDE:
        {
            if (data && dataSize && *dataSize == sizeof(int))
            {
                int val = mVideoMode.resolutionX * getPixelFormatBytes(mVideoMode.pixelFormat);
                *((int*)data) = val;
                zedLogFunc("\t Stride: %d",val);
                return ONI_STATUS_OK;
            }
            break;
        }

//		case ONI_STREAM_PROPERTY_MIRRORING:
//		{
//			if (data && dataSize && *dataSize == sizeof(OniBool))
//			{
//				*((OniBool*)data) = false;
//				return ONI_STATUS_OK;
//			}
//			break;
//		}

//		case ONI_STREAM_PROPERTY_AUTO_WHITE_BALANCE:
//		{
//			if (data && dataSize && *dataSize == sizeof(OniBool) && m_oniType == ONI_SENSOR_COLOR)
//			{
//				Rs2Error e;
//				float value = rs2_get_option((const rs2_options*)m_sensor, RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, &e);
//				if (e.success())
//				{
//					*((OniBool*)data) = (int)value ? true : false;
//					return ONI_STATUS_OK;
//				}
//			}
//			break;
//		}

//		case ONI_STREAM_PROPERTY_AUTO_EXPOSURE:
//		{
//			if (data && dataSize && *dataSize == sizeof(OniBool) && m_oniType == ONI_SENSOR_COLOR)
//			{
//				Rs2Error e;
//				float value = rs2_get_option((const rs2_options*)m_sensor, RS2_OPTION_ENABLE_AUTO_EXPOSURE, &e);
//				if (e.success())
//				{
//					*((OniBool*)data) = (int)value ? true : false;
//					return ONI_STATUS_OK;
//				}
//			}
//			break;
//		}

//		case ONI_STREAM_PROPERTY_EXPOSURE:
//		{
//			if (data && dataSize && *dataSize == sizeof(int) && m_oniType == ONI_SENSOR_COLOR)
//			{
//				Rs2Error e;
//				float value = rs2_get_option((const rs2_options*)m_sensor, RS2_OPTION_EXPOSURE, &e);
//				if (e.success())
//				{
//					*((int*)data) = (int)value;
//					return ONI_STATUS_OK;
//				}
//			}
//			break;
//		}

//		case ONI_STREAM_PROPERTY_GAIN:
//		{
//			if (data && dataSize && *dataSize == sizeof(int) && m_oniType == ONI_SENSOR_COLOR)
//			{
//				Rs2Error e;
//				float value = rs2_get_option((const rs2_options*)m_sensor, RS2_OPTION_GAIN, &e);
//				if (e.success())
//				{
//					*((int*)data) = (int)value;
//					return ONI_STATUS_OK;
//				}
//			}
//			break;
//		}

//		case XN_STREAM_PROPERTY_GAIN:
//		{
//			if (data && dataSize && *dataSize == sizeof(unsigned long long) && m_oniType == ONI_SENSOR_DEPTH)
//			{
//				*((unsigned long long*)data) = GAIN_VAL;
//				return ONI_STATUS_OK;
//			}
//			break;
//		}

//        case XN_STREAM_PROPERTY_CONST_SHIFT:
//		{
//			if (data && dataSize && *dataSize == sizeof(unsigned long long) && m_oniType == ONI_SENSOR_DEPTH)
//			{
//				*((unsigned long long*)data) = CONST_SHIFT_VAL;
//				return ONI_STATUS_OK;
//			}
//			break;
//		}

//        case XN_STREAM_PROPERTY_MAX_SHIFT:
//		{
//			if (data && dataSize && *dataSize == sizeof(unsigned long long) && m_oniType == ONI_SENSOR_DEPTH)
//			{
//				*((unsigned long long*)data) = MAX_SHIFT_VAL;
//				return ONI_STATUS_OK;
//			}
//			break;
//		}

//        case XN_STREAM_PROPERTY_PARAM_COEFF:
//		{
//			if (data && dataSize && *dataSize == sizeof(unsigned long long) && m_oniType == ONI_SENSOR_DEPTH)
//			{
//				*((unsigned long long*)data) = PARAM_COEFF_VAL;
//				return ONI_STATUS_OK;
//			}
//			break;
//		}

//        case XN_STREAM_PROPERTY_SHIFT_SCALE:
//		{
//			if (data && dataSize && *dataSize == sizeof(unsigned long long) && m_oniType == ONI_SENSOR_DEPTH)
//			{
//				*((unsigned long long*)data) = SHIFT_SCALE_VAL;
//				return ONI_STATUS_OK;
//			}
//			break;
//		}

//        case XN_STREAM_PROPERTY_ZERO_PLANE_DISTANCE:
//		{
//			if (data && dataSize && *dataSize == sizeof(unsigned long long) && m_oniType == ONI_SENSOR_DEPTH)
//			{
//				*((unsigned long long*)data) = ZERO_PLANE_DISTANCE_VAL;
//				return ONI_STATUS_OK;
//			}
//			break;
//		}

//        case XN_STREAM_PROPERTY_ZERO_PLANE_PIXEL_SIZE:
//		{
//			if (data && dataSize && *dataSize == sizeof(double) && m_oniType == ONI_SENSOR_DEPTH)
//			{
//				*((double*)data) = ZERO_PLANE_PIXEL_SIZE_VAL;
//				return ONI_STATUS_OK;
//			}
//			break;
//		}

//        case XN_STREAM_PROPERTY_EMITTER_DCMOS_DISTANCE:
//		{
//			if (data && dataSize && *dataSize == sizeof(double) && m_oniType == ONI_SENSOR_DEPTH)
//			{
//				*((double*)data) = EMITTER_DCMOS_DISTANCE_VAL;
//				return ONI_STATUS_OK;
//			}
//			break;
//		}

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
    zedLogFunc("propertyId=%d", propertyId);

    switch (propertyId)
    {
//		case ONI_STREAM_PROPERTY_CROPPING:				// OniCropping*
//		case ONI_STREAM_PROPERTY_HORIZONTAL_FOV:		// float: radians
//		case ONI_STREAM_PROPERTY_VERTICAL_FOV:			// float: radians
//		case ONI_STREAM_PROPERTY_VIDEO_MODE:			// OniVideoMode*
//		case ONI_STREAM_PROPERTY_MAX_VALUE:				// int
//		case ONI_STREAM_PROPERTY_MIN_VALUE:				// int
//		case ONI_STREAM_PROPERTY_STRIDE:				// int
//		case ONI_STREAM_PROPERTY_MIRRORING:				// OniBool
//			return true;


        case ONI_STREAM_PROPERTY_NUMBER_OF_FRAMES:		// int
            return false;

//		case ONI_STREAM_PROPERTY_AUTO_WHITE_BALANCE:	// OniBool
//		case ONI_STREAM_PROPERTY_AUTO_EXPOSURE:			// OniBool
//		case ONI_STREAM_PROPERTY_EXPOSURE:				// int
//		case ONI_STREAM_PROPERTY_GAIN:					// int
//			return true;

//		case XN_STREAM_PROPERTY_GAIN:
//        case XN_STREAM_PROPERTY_CONST_SHIFT:
//        case XN_STREAM_PROPERTY_MAX_SHIFT:
//        case XN_STREAM_PROPERTY_PARAM_COEFF:
//        case XN_STREAM_PROPERTY_SHIFT_SCALE:
//        case XN_STREAM_PROPERTY_ZERO_PLANE_DISTANCE:
//        case XN_STREAM_PROPERTY_ZERO_PLANE_PIXEL_SIZE:
//        case XN_STREAM_PROPERTY_EMITTER_DCMOS_DISTANCE:
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
