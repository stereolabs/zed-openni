#include "device.h"
#include "util.h"

// Constructor
Device::Device()
{
    // Initialize
    initialize();
}

// Destructor
Device::~Device()
{
    // Finalize
    finalize();
}

// Processing
void Device::run()
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
void Device::initialize()
{
    cv::setUseOptimized( true );

    // Initialize OpenNI2
    OPENNI_CHECK( openni::OpenNI::initialize() );

    // Initialize Device
    initializeDevice();

    // Initialize Depth
    initializeDepth();
}

// Initialize Device
inline void Device::initializeDevice()
{
    // Open Device
    OPENNI_CHECK( device.open( openni::ANY_DEVICE ) );
}

// Initialize Depth
inline void Device::initializeDepth()
{
    // Create Stream
    OPENNI_CHECK( depth_stream.create( device, openni::SENSOR_DEPTH ) );

    /*
    // Set Video Mode
    openni::VideoMode depth_mode;
    depth_mode.setResolution( depth_width, depth_height );
    depth_mode.setFps( depth_fps );
    depth_mode.setPixelFormat( openni::PixelFormat::PIXEL_FORMAT_DEPTH_1_MM );
    OPENNI_CHECK( depth_stream.setVideoMode( depth_mode ) );
    */

    // Start Stream
    OPENNI_CHECK( depth_stream.start() );
}

// Finalize
void Device::finalize()
{
    // Close Windows
    cv::destroyAllWindows();
}

// Update Data
void Device::update()
{
    // Update Depth
    updateDepth();
}

// Update Depth
inline void Device::updateDepth()
{
    // Update Frame
    OPENNI_CHECK( depth_stream.readFrame( &depth_frame ) );

    // Retrive Frame Size
    depth_width = depth_frame.getWidth();
    depth_height = depth_frame.getHeight();
}

// Draw Data
void Device::draw()
{
    // Draw Depth
    drawDepth();
}

// Draw Depth
inline void Device::drawDepth()
{
    // Create cv::Mat form Depth Frame
    depth_mat = cv::Mat( depth_height, depth_width, CV_16UC1, const_cast<void*>( depth_frame.getData() ) );
}

// Show Data
void Device::show()
{
    // Show Depth
    showDepth();
}

// Show Depth
inline void Device::showDepth()
{
    if( depth_mat.empty() ){
        return;
    }

    // Scaling
    cv::Mat scale_mat;
    const uint32_t max_range = depth_stream.getMaxPixelValue();
    depth_mat.convertTo( scale_mat, CV_8U, -255.0 / max_range, 255.0 ); // 0-max -> 255(white)-0(black)

    // Apply False Colour
    cv::applyColorMap( scale_mat, scale_mat, cv::COLORMAP_JET);

    // Show Depth Image
    cv::imshow( "Depth", scale_mat );
}
