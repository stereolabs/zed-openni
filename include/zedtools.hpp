#ifndef ZEDTOOLS_HPP
#define ZEDTOOLS_HPP

#define zedLogError(format, ...) printf("[ZED] ERROR at FILE %s LINE %d FUNC %s\n\t" format "\n", __FILE__, __LINE__, __FUNCTION__, ##  __VA_ARGS__)
#define zedLogFunc(format, ...) printf("[ZED] %s " format "\n", __FUNCTION__, ##  __VA_ARGS__)
#define zedLogDebug(format, ...) printf("[ZED] " format "\n", ## __VA_ARGS__)

#endif
