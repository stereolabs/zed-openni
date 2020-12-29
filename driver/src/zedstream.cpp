#include "zedstream.hpp"

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

void ZedStream::shutdown()
{
    // TODO
    zedLogFunc("");
}

} } // namespace driver // namespace oni
