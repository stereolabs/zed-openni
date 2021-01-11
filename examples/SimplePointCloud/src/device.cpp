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

    // Initialize Point Cloud
    initializePointCloud();
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

    // Start Stream
    OPENNI_CHECK( depth_stream.start() );
}

// Initialize Color
inline void Device::initializeColor()
{
    // Create Stream
    OPENNI_CHECK( color_stream.create( device, openni::SENSOR_COLOR ) );

    // Start Stream
    OPENNI_CHECK( color_stream.start() );
}

// Initialize Point Cloud
inline void Device::initializePointCloud()
{
    // Create Window
    viewer = cv::viz::Viz3d( "ZED Point Cloud" );

    // Register Keyboard Callback Function
    viewer.registerKeyboardCallback( &keyboardCallback, this );
}

// Keyboard Callback Function
void Device::keyboardCallback( const cv::viz::KeyboardEvent& event, void* cookie )
{
    // Exit Viewer when Pressed ESC key
    if( (event.code == 'q' || event.code=='Q' || event.code==27) && event.action == cv::viz::KeyboardEvent::Action::KEY_DOWN ){

        // Retrieve Viewer
        cv::viz::Viz3d viewer = static_cast<Device*>( cookie )->viewer;

        // Close Viewer
        viewer.close();
    }
};

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
    width = color_frame.getWidth();
    height = color_frame.getHeight();
}

// Update Depth
inline void Device::updateDepth()
{
    // Update Frame
    OPENNI_CHECK( depth_stream.readFrame( &depth_frame ) );

    // Retrieve Frame Size
    width = depth_frame.getWidth();
    height = depth_frame.getHeight();
}

// Draw Color
inline void Device::drawColor()
{
    // Create cv::Mat form Color Frame
    color_mat = cv::Mat( height, width, CV_8UC3, const_cast<void*>( color_frame.getData() ) );

    // Convert RGB to BGR
    cv::cvtColor( color_mat, color_mat, cv::COLOR_RGB2BGR );
}

// Draw Data
void Device::draw()
{
    // Draw color
    drawColor();

    // Draw Point Cloud
    drawPointCloud();
}

// Draw Point Cloud
inline void Device::drawPointCloud()
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
void Device::show()
{
    // Show Point Cloud
    showPointCloud();
}

// Show Point Cloud
inline void Device::showPointCloud()
{
    if( vertices_mat.empty() ){
        return;
    }

    // Create Point Cloud
    cv::viz::WCloud cloud( vertices_mat, color_mat);

    // Show Point Cloud
    viewer.showWidget( "Cloud", cloud );
    viewer.spinOnce();
}
