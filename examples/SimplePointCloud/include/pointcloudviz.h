#ifndef POINTCLOUDVIZ_HPP
#define POINTCLOUDVIZ_HPP

#include <OpenNI.h>
#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>

class PointcloudViz
{
private:
    // Device
    openni::Device device;
    openni::VideoStream depth_stream;
    openni::VideoStream color_stream;

    // Depth Buffer
    openni::VideoFrameRef depth_frame;
    openni::VideoFrameRef color_frame;
    cv::Mat color_mat;
    uint32_t width = 640;
    uint32_t height = 480;
    uint32_t fps = 30;

    // Point Cloud Buffer
    cv::viz::Viz3d viewer;
    cv::Mat vertices_mat;

public:
    // Constructor
    PointcloudViz();

    // Destructor
    ~PointcloudViz();

    // Processing
    void run();

private:
    // Initialize
    void initialize();

    // Initialize Device
    inline void initializeDevice();

    // Initialize Depth
    inline void initializeDepth();

    // Initialize Color
    inline void initializeColor();

    // Initialize Point Cloud
    inline void initializePointCloud();

    // Keyboard Callback Function
    static void keyboardCallback( const cv::viz::KeyboardEvent& event, void* cookie );

    // Finalize
    void finalize();

    // Update Data
    void update();

    // Update Depth
    inline void updateDepth();

    // Update Color
    inline void updateColor();

    // Draw Data
    void draw();

    // Draw Color
    inline void drawColor();

    // Draw Point Cloud
    inline void drawPointCloud();

    // Show Data
    void show();

    // Show Point Cloud
    inline void showPointCloud();
};

#endif // POINTCLOUDVIZ_HPP
