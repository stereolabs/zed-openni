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
    inline void initializeViewer();

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
