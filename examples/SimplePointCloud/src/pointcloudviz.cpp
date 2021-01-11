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

#include "pointcloudviz.h"
#include "tools.h"

// Constructor
PointcloudViz::PointcloudViz()
{
    // Initialize
    initialize();
}

// Destructor
PointcloudViz::~PointcloudViz()
{
    // Finalize
    finalize();
}

// Processing
void PointcloudViz::run()
{
    // Main Loop
    while( !viewer.wasStopped() ){
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
void PointcloudViz::initialize()
{
    cv::setUseOptimized( true );

    // Initialize OpenNI2
    OPENNI_CHECK( openni::OpenNI::initialize() );

    // Initialize Device
    initializeDevice();

    // Initialize Color
    initializeColor();

    // Initialize Depth
    initializeDepth();

    // Initialize Point Cloud
    initializeViewer();
}

// Initialize Device
inline void PointcloudViz::initializeDevice()
{
    // Open Device
    OPENNI_CHECK( device.open( openni::ANY_DEVICE ) );
}

// Initialize Depth
inline void PointcloudViz::initializeDepth()
{
    // Create Stream
    OPENNI_CHECK( depth_stream.create( device, openni::SENSOR_DEPTH ) );

    // Start Stream
    OPENNI_CHECK( depth_stream.start() );
}

// Initialize Color
inline void PointcloudViz::initializeColor()
{
    // Create Stream
    OPENNI_CHECK( color_stream.create( device, openni::SENSOR_COLOR ) );

    // Start Stream
    OPENNI_CHECK( color_stream.start() );
}

// Initialize Point Cloud
inline void PointcloudViz::initializeViewer()
{
    // Create Window
    viewer = cv::viz::Viz3d( "ZED Point Cloud" );
    viewer.setWindowSize( cv::Size(800,600));

    // Register Keyboard Callback Function
    viewer.registerKeyboardCallback( &keyboardCallback, this );
}

// Keyboard Callback Function
void PointcloudViz::keyboardCallback( const cv::viz::KeyboardEvent& event, void* cookie )
{
    // Exit Viewer when Pressed ESC key
    if( (event.code == 'q' || event.code=='Q' || event.code==27) && event.action == cv::viz::KeyboardEvent::Action::KEY_DOWN ){

        // Retrieve Viewer
        cv::viz::Viz3d viewer = static_cast<PointcloudViz*>( cookie )->viewer;

        // Close Viewer
        viewer.close();
    }
};

// Finalize
void PointcloudViz::finalize()
{
    // Close Windows
    cv::destroyAllWindows();
}

// Update Data
void PointcloudViz::update()
{
    // Update Color
    updateColor();

    // Update Depth
    updateDepth();
}

// Update Color
inline void PointcloudViz::updateColor()
{
    // Update Frame
    OPENNI_CHECK( color_stream.readFrame( &color_frame ) );

    // Retrive Frame Size
    width = color_frame.getWidth();
    height = color_frame.getHeight();
}

// Update Depth
inline void PointcloudViz::updateDepth()
{
    // Update Frame
    OPENNI_CHECK( depth_stream.readFrame( &depth_frame ) );

    // Retrieve Frame Size
    width = depth_frame.getWidth();
    height = depth_frame.getHeight();
}

// Draw Color
inline void PointcloudViz::drawColor()
{
    // Create cv::Mat form Color Frame
    color_mat = cv::Mat( height, width, CV_8UC3, const_cast<void*>( color_frame.getData() ) );

    // Convert RGB to BGR
    cv::cvtColor( color_mat, color_mat, cv::COLOR_RGB2BGR );
}

// Draw Data
void PointcloudViz::draw()
{
    // Draw color
    drawColor();

    // Draw Point Cloud
    drawPointCloud();
}

// Draw Point Cloud
inline void PointcloudViz::drawPointCloud()
{
    if( !depth_frame.isValid() ){
        return;
    }

    // Retrieve Depth
    const uint16_t* depth = static_cast<const uint16_t*>( depth_frame.getData() );

    // Create cv::Mat from Vertices and Texture
    vertices_mat = cv::Mat( height, width, CV_32FC3, cv::Vec3f::all( std::numeric_limits<float>::quiet_NaN() ) );

#pragma omp parallel for
    for( uint32_t y = 0; y < height; y++ ){
        for( uint32_t x = 0; x < width; x++ ){
            // Retrieve Depth
            const uint16_t z = depth[y * width + x];
            if( !z ){
                continue;
            }

            // Convert Depth to World
            float wx, wy, wz;
            OPENNI_CHECK( openni::CoordinateConverter::convertDepthToWorld( depth_stream, static_cast<float>( x ), static_cast<float>( y ), static_cast<float>( z ), &wx, &wy, &wz ) );

            // Add Point to Vertices
            vertices_mat.at<cv::Vec3f>( y, x ) = cv::Vec3f( wx, wy, wz );
        }
    }
}

// Show Data
void PointcloudViz::show()
{
    // Show Point Cloud
    showPointCloud();
}

// Show Point Cloud
inline void PointcloudViz::showPointCloud()
{
    if( vertices_mat.empty() ){
        return;
    }

    // Create Point Cloud
    cv::viz::WCloud cloud( vertices_mat, color_mat);

    // Show Point Cloud
    viewer.showWidget( "Cloud", cloud );
    viewer.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem(500.0));
    //viewer.resetCameraViewpoint("Cloud");

    viewer.spinOnce();
}
