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

#include "colorviz.h"
#include "tools.h"

// Constructor
ColorViz::ColorViz()
{
    // Initialize
    initialize();
}

// Destructor
ColorViz::~ColorViz()
{
    // Finalize
    finalize();
}

// Processing
void ColorViz::run()
{
    // Main Loop
    while( true ){
        // Update Data
        update();

        // Draw Data
        draw();

        // Show Data
        show();

        // Key Check
        const int32_t key = cv::waitKey( 1 );
        if( key == 'q' ){
            break;
        }
    }
}

// Initialize
void ColorViz::initialize()
{
    cv::setUseOptimized( true );

    // Initialize OpenNI2
    OPENNI_CHECK( openni::OpenNI::initialize() );

    // Initialize Device
    initializeDevice();

    // Initialize Color
    initializeColor();
}

// Initialize Device
inline void ColorViz::initializeDevice()
{
    // Open Device
    OPENNI_CHECK( device.open( openni::ANY_DEVICE ) );
}

// Initialize Color
inline void ColorViz::initializeColor()
{
    // Create Stream
    OPENNI_CHECK( color_stream.create( device, openni::SENSOR_COLOR ) );

    const openni::SensorInfo* colorSensInfo = device.getSensorInfo(openni::SENSOR_COLOR);
    const openni::Array<openni::VideoMode>& supportedColorModes = colorSensInfo->getSupportedVideoModes();

    printf("\n * Available Color modes:\n");
    for( int i=0; i<supportedColorModes.getSize(); i++ )
    {
        printf("\t#%d %dx%d@%d\n", i,supportedColorModes[i].getResolutionX(),supportedColorModes[i].getResolutionY(),supportedColorModes[i].getFps());
    }

    color_stream.setVideoMode(supportedColorModes[0]);

    // Start Stream
    OPENNI_CHECK( color_stream.start() );
}

// Finalize
void ColorViz::finalize()
{
    // Close Windows
    cv::destroyAllWindows();
}

// Update Data
void ColorViz::update()
{
    // Update Color
    updateColor();
}

// Update Color
inline void ColorViz::updateColor()
{
    // Update Frame
    OPENNI_CHECK( color_stream.readFrame( &color_frame ) );

    // Retrive Frame Size
    color_width = color_frame.getWidth();
    color_height = color_frame.getHeight();
}

// Draw Data
void ColorViz::draw()
{
    // Draw Color
    drawColor();
}

// Draw Color
inline void ColorViz::drawColor()
{
    // Create cv::Mat form Color Frame
    color_mat = cv::Mat( color_height, color_width, CV_8UC3, const_cast<void*>( color_frame.getData() ) );

    // Convert RGB to BGR
    cv::cvtColor( color_mat, color_mat, cv::COLOR_RGB2BGR );
}

// Show Data
void ColorViz::show()
{
    // Show Color
    showColor();
}

// Show Color
inline void ColorViz::showColor()
{
    if( color_mat.empty() ){
        return;
    }

    // Show Color Image
    cv::imshow( "Color", color_mat );
}
