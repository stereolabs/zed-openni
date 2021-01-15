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

#include "depthviz.h"
#include "tools.h"

// Constructor
DepthViz::DepthViz()
{
    // Initialize
    initialize();
}

// Destructor
DepthViz::~DepthViz()
{
    // Finalize
    finalize();
}

// Processing
void DepthViz::run()
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
        const int32_t key = cv::waitKey( 10 );
        if( key == 'q' ){
            break;
        }
    }
}

// Initialize
void DepthViz::initialize()
{
    cv::setUseOptimized( true );

    // Initialize OpenNI2
    OPENNI_CHECK( openni::OpenNI::initialize() );

    // Initialize Device
    initializeDevice();

    // Initialize Depth
    initializeDepth();

    // Initialize Color
    initializeDepth();
}

// Initialize Device
inline void DepthViz::initializeDevice()
{
    // Open Device
    OPENNI_CHECK( device.open( openni::ANY_DEVICE ) );

    // Set Registration Mode
    // Note: this is not required by ZED driver because depth and color are automatically registered
    OPENNI_CHECK( device.setImageRegistrationMode( openni::ImageRegistrationMode::IMAGE_REGISTRATION_DEPTH_TO_COLOR ) );
}

// Initialize Depth
inline void DepthViz::initializeDepth()
{
    // Create Stream
    OPENNI_CHECK( depth_stream.create( device, openni::SENSOR_DEPTH ) );

    // Start Stream
    OPENNI_CHECK( depth_stream.start() );
}

// Finalize
void DepthViz::finalize()
{
    // Close Windows
    cv::destroyAllWindows();
}

// Update Data
void DepthViz::update()
{
    // Update Depth
    updateDepth();
}

// Update Depth
inline void DepthViz::updateDepth()
{
    // Update Frame
    OPENNI_CHECK( depth_stream.readFrame( &depth_frame ) );

    // Retrive Frame Size
    depth_width = depth_frame.getWidth();
    depth_height = depth_frame.getHeight();
}

// Draw Data
void DepthViz::draw()
{
    // Draw Depth
    drawDepth();
}

// Draw Depth
inline void DepthViz::drawDepth()
{
    // Create cv::Mat form Depth Frame
    depth_mat = cv::Mat( depth_height, depth_width, CV_16UC1, const_cast<void*>( depth_frame.getData() ) );
}

// Show Data
void DepthViz::show()
{
    // Show Depth
    showDepth();
}

// Show Depth
inline void DepthViz::showDepth()
{
    if( depth_mat.empty() ){
        return;
    }

    // Scaling
    cv::Mat scale_mat;
    const uint32_t max_range = depth_stream.getMaxPixelValue();
    //depth_mat.convertTo( scale_mat, CV_8U, -255.0 / max_range, 255.0 ); // 0-max -> 255(white)-0(black)
    depth_mat.convertTo( scale_mat, CV_8U, 255.0 / max_range, 0.0 );

    // Apply False Colour
    cv::applyColorMap( scale_mat, scale_mat, cv::COLORMAP_JET);

    // Show Depth Image
    cv::imshow( "Depth", scale_mat );
}
