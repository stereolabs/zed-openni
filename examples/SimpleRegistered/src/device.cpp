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

    // Initialize Color
    initializeColor();

    // Initialize Depth
    initializeDepth();
}

// Initialize Device
inline void Device::initializeDevice()
{
    // Open Device
    OPENNI_CHECK( device.open( openni::ANY_DEVICE ) );

    // Set Registration Mode
    // Note: this is not required by ZED driver because depth and color are automatically registered
    OPENNI_CHECK( device.setImageRegistrationMode( openni::ImageRegistrationMode::IMAGE_REGISTRATION_DEPTH_TO_COLOR ) );
}

// Initialize Color
inline void Device::initializeColor()
{
    // Create Stream
    OPENNI_CHECK( color_stream.create( device, openni::SENSOR_COLOR ) );

    // Start Stream
    OPENNI_CHECK( color_stream.start() );
}

// Initialize Depth
inline void Device::initializeDepth()
{
    // Create Stream
    OPENNI_CHECK( depth_stream.create( device, openni::SENSOR_DEPTH ) );

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
    // Update Color
    updateColor();

    // Update Depth
    updateDepth();
}

// Update Color
inline void Device::updateColor()
{
    // Update Frame
    OPENNI_CHECK( color_stream.readFrame( &color_frame ) );

    // Retrive Frame Size
    color_width = color_frame.getWidth();
    color_height = color_frame.getHeight();
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
    // Draw Color
    drawColor();

    // Draw Depth
    drawDepth();
}

// Draw Color
inline void Device::drawColor()
{
    // Create cv::Mat form Color Frame
    color_mat = cv::Mat( color_height, color_width, CV_8UC3, const_cast<void*>( color_frame.getData() ) );

    // Convert RGB to BGR
    cv::cvtColor( color_mat, color_mat, cv::COLOR_RGB2BGR );
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
    // Show Color
    showColor();

    // Show Depth
    showDepth();
}

// Show Color
inline void Device::showColor()
{
    if( color_mat.empty() ){
        return;
    }

    // Show Color Image
    cv::imshow( "Color", color_mat );
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
