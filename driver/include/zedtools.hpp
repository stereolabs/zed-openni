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

#ifndef ZEDTOOLS_HPP
#define ZEDTOOLS_HPP

#include <Driver/OniDriverAPI.h>
#include <sl/Camera.hpp>

#define EMULATE_PRIMESENSE_HARDWARE // HACK: NiTE only runs on PrimeSense SoC, so we simulate it

#define DEFAULT_INIT_PARAMS_FILE "./ZedInitConfig.yaml"
#define DEFAULT_RT_PARAMS_FILE "./ZedRuntimeConfig.yaml"

#define zedLogError(format, ...) printf("[ZED] ERROR at FILE %s LINE %d FUNC %s\n\t" format "\n", __FILE__, __LINE__, __FUNCTION__, ##  __VA_ARGS__);fflush(stdout)
#define zedLogFunc(format, ...) printf("[ZED] (%s) %s " format "\n", typeid(this).name(), __FUNCTION__,  ##  __VA_ARGS__);fflush(stdout)
#define zedLogDebug(format, ...) printf("[ZED] " format "\n", ## __VA_ARGS__);fflush(stdout)

static const uint16_t SL_USB_VENDOR = 0x2b03;        //!< Stereolabs Vendor ID

static const uint16_t SL_USB_PROD_ZED = 0xf582;      //!< CBS ZED Firmware Product ID
static const uint16_t SL_USB_PROD_ZED_M = 0xf682;    //!< CBS ZED-M Firmware Product ID
static const uint16_t SL_USB_PROD_ZED_2 = 0xf780;    //!< CBS ZED 2 Firmware Product ID

#define ONI_MAX_DEPTH 10000

namespace oni { namespace driver {

inline std::string streamType2Str(OniSensorType type)
{
    switch(type)
    {
    case ONI_SENSOR_DEPTH:
        return std::string("DEPTH");
        break;
    case ONI_SENSOR_COLOR:
        return std::string("COLOR");
        break;
    case ONI_SENSOR_IR:
        return std::string("GRAY");
        break;
    default:
        return std::string("NOT SUPPORTED");
        break;
    }
}

inline int convertStreamType(OniSensorType type)
{
    switch(type)
    {
    case ONI_SENSOR_DEPTH:
        return static_cast<int>(sl::MEASURE::DEPTH);
    case ONI_SENSOR_COLOR:
        return static_cast<int>(sl::VIEW::LEFT);
    case ONI_SENSOR_IR:
        return static_cast<int>(sl::VIEW::LEFT_GRAY);
    default:
        zedLogError("OpenNI Sensor type not correct: %d", type);
        return -1;
    }
}

inline OniSensorType convertStreamType(int type)
{
    switch(type)
    {
    case 1:
        return ONI_SENSOR_DEPTH;
    case 0:
        return ONI_SENSOR_COLOR;
    case 2:
        return ONI_SENSOR_IR;
    default:
        zedLogError("ZED Sensor type not correct: %d", type);
        return (OniSensorType)-1;
    }
}

inline int getPixelFormatBytes(OniPixelFormat type)
{
    switch (type)
    {
    // Depth
    case ONI_PIXEL_FORMAT_DEPTH_1_MM:
        return 2;

    // Color
    case ONI_PIXEL_FORMAT_RGB888:
        return 3;
    case ONI_PIXEL_FORMAT_GRAY8:
        return 1;
    }

    zedLogError("Invalid OniPixelFormat=%d", (int)type);
    return 0;
}

} } // namespace driver // namespace oni
#endif
