#ifndef DEPTHVIZ_HPP
#define DEPTHVIZ_HPP

#include <OpenNI.h>
#include <opencv2/opencv.hpp>

class DepthViz
{
private:
    // Device
    openni::Device device;
    openni::VideoStream depth_stream;
    openni::VideoStream color_stream;

    // Depth Buffer
    openni::VideoFrameRef depth_frame;
    // Depth Buffer
    openni::VideoFrameRef color_frame;
    cv::Mat depth_mat;
    uint32_t depth_width = 640;
    uint32_t depth_height = 480;
    uint32_t depth_fps = 30;

public:
    // Constructor
    DepthViz();

    // Destructor
    ~DepthViz();

    // Processing
    void run();

private:
    // Initialize
    void initialize();

    // Initialize Device
    inline void initializeDevice();

    // Initialize Depth
    inline void initializeDepth();

    // Finalize
    void finalize();

    // Update Data
    void update();

    // Update Depth
    inline void updateDepth();

    // Draw Data
    void draw();

    // Draw Depth
    inline void drawDepth();

    // Show Data
    void show();

    // Show Depth
    inline void showDepth();
};

#endif // DEPTHVIZ_HPP
