#ifndef ZEDTOOLS_HPP
#define ZEDTOOLS_HPP

#define EMULATE_PRIMESENSE_HARDWARE // HACK: NiTE only runs on PrimeSense SoC, so we simulate it

#define zedLogError(format, ...) printf("[ZED] ERROR at FILE %s LINE %d FUNC %s\n\t" format "\n", __FILE__, __LINE__, __FUNCTION__, ##  __VA_ARGS__);fflush(stdout)
#define zedLogFunc(format, ...) printf("[ZED] %s " format "\n", __FUNCTION__, ##  __VA_ARGS__);fflush(stdout)
#define zedLogDebug(format, ...) printf("[ZED] " format "\n", ## __VA_ARGS__);fflush(stdout)

static const uint16_t SL_USB_VENDOR = 0x2b03;        //!< Stereolabs Vendor ID

static const uint16_t SL_USB_PROD_ZED = 0xf582;      //!< CBS ZED Firmware Product ID
static const uint16_t SL_USB_PROD_ZED_M = 0xf682;    //!< CBS ZED-M Firmware Product ID
static const uint16_t SL_USB_PROD_ZED_2 = 0xf780;    //!< CBS ZED 2 Firmware Product ID

#endif
