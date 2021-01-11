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

#include <OpenNI.h>
#include "Viewer.h"
#include "tools.h"

int main(int argc, char** argv)
{
	openni::Status rc = openni::STATUS_OK;

	openni::Device device;
	openni::VideoStream depth, color;
	const char* deviceURI = openni::ANY_DEVICE;
	if (argc > 1)
	{
		deviceURI = argv[1];
	}

	rc = openni::OpenNI::initialize();
    if (rc != openni::STATUS_OK)
    {
        printf("SimpleViewer: Device initialize failed:\n%s\n", openni::OpenNI::getExtendedError());
        openni::OpenNI::shutdown();
        return 1;
    }

    printf("After initialization:\n%s\n", openni::OpenNI::getExtendedError());

    rc = device.open(deviceURI);
	if (rc != openni::STATUS_OK)
	{
		printf("SimpleViewer: Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
		openni::OpenNI::shutdown();
		return 1;
	}

    const openni::SensorInfo* colorSensInfo = device.getSensorInfo(openni::SENSOR_COLOR);
    const openni::Array<openni::VideoMode>& supportedColorModes = colorSensInfo->getSupportedVideoModes();

    const openni::SensorInfo* depthSensInfo = device.getSensorInfo(openni::SENSOR_DEPTH);
    const openni::Array<openni::VideoMode>& supportedDepthModes = depthSensInfo->getSupportedVideoModes();

    printf("\n * Available Color modes:\n");
    for( int i=0; i<supportedColorModes.getSize(); i++ )
    {
        printf("\t#%d %dx%d@%d\n", i,supportedColorModes[i].getResolutionX(),supportedColorModes[i].getResolutionY(),supportedColorModes[i].getFps());
    }

    printf(" * Available Depth modes:\n");
    for( int i=0; i<supportedDepthModes.getSize(); i++ )
    {
        printf("\t#%d %dx%d@%d\n", i,supportedDepthModes[i].getResolutionX(),supportedDepthModes[i].getResolutionY(),supportedDepthModes[i].getFps());
    }
    printf("\n");

    rc = color.create(device, openni::SENSOR_COLOR);
    if (rc == openni::STATUS_OK)
    {
        rc = color.start();
        if (rc != openni::STATUS_OK)
        {
            printf("SimpleViewer: Couldn't start color stream:\n%s\n", openni::OpenNI::getExtendedError());
            color.destroy();
        }
    }
    else
    {
        printf("SimpleViewer: Couldn't find color stream:\n%s\n", openni::OpenNI::getExtendedError());
    }

    rc = depth.create(device, openni::SENSOR_DEPTH);
	if (rc == openni::STATUS_OK)
	{
		rc = depth.start();
		if (rc != openni::STATUS_OK)
		{
			printf("SimpleViewer: Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
			depth.destroy();
		}
    }
	else
	{
		printf("SimpleViewer: Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());
    }

    if (!depth.isValid() || !color.isValid())
	{
		printf("SimpleViewer: No valid streams. Exiting\n");
		openni::OpenNI::shutdown();
		return 2;
	}   

    openni::CameraSettings* settings = color.getCameraSettings();
    bool aeg = settings->getAutoExposureEnabled();
    printf("Auto gain/exposure: %s\n", aeg?"TRUE":"FALSE");
    bool awb = settings->getAutoWhiteBalanceEnabled();
    printf("Auto white balance: %s\n", awb?"TRUE":"FALSE");

    if(awb==FALSE)
    {
        printf("Enabling auto white balance");
        settings->setAutoWhiteBalanceEnabled( true);
    }
    if(aeg==FALSE)
    {
        printf("Enabling auto gain/exposure");
        settings->setAutoExposureEnabled( true);
    }

	SampleViewer sampleViewer("Simple Viewer", device, depth, color);

	rc = sampleViewer.init(argc, argv);
	if (rc != openni::STATUS_OK)
	{
		openni::OpenNI::shutdown();
		return 3;
	}
	sampleViewer.run();
}
