/*****************************************************************************
*                                                                            *
*  OpenNI 2.x Alpha                                                          *
*  Copyright (C) 2012 PrimeSense Ltd.                                        *
*                                                                            *
*  This file is part of OpenNI.                                              *
*                                                                            *
*  Licensed under the Apache License, Version 2.0 (the "License");           *
*  you may not use this file except in compliance with the License.          *
*  You may obtain a copy of the License at                                   *
*                                                                            *
*      http://www.apache.org/licenses/LICENSE-2.0                            *
*                                                                            *
*  Unless required by applicable law or agreed to in writing, software       *
*  distributed under the License is distributed on an "AS IS" BASIS,         *
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  *
*  See the License for the specific language governing permissions and       *
*  limitations under the License.                                            *
*                                                                            *
*****************************************************************************/
#include <OpenNI.h>
#include "Viewer.h"

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
