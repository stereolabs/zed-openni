#ifndef COLORVIZ_HPP
#define COLORVIZ_HPP

#include <OpenNI.h>
#include <opencv2/opencv.hpp>

class ColorViz
{
private:
    // Device
    openni::Device device;
    openni::VideoStream color_stream;

    // Color Buffer
    openni::VideoFrameRef color_frame;
    cv::Mat color_mat;
    uint32_t color_width = 640;
    uint32_t color_height = 480;
    uint32_t color_fps = 30;

public:
    // Constructor
    ColorViz();

    // Destructor
    ~ColorViz();

    // Processing
    void run();

private:
    // Initialize
    void initialize();

    // Initialize Device
    inline void initializeDevice();

    // Initialize Color
    inline void initializeColor();

    // Finalize
    void finalize();

    // Update Data
    void update();

    // Update Color
    inline void updateColor();

    // Draw Data
    void draw();

    // Draw Color
    inline void drawColor();

    // Show Data
    void show();

    // Show Color
    inline void showColor();
};

#endif // COLORVIZ_HPP
