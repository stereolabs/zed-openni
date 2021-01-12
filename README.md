<h1 align="center">
  ZED OpenNI2 driver
</h1>

<h4 align="center">OpenNI2 driver for the ZED stereo camera family</h4>

<p align="center">
  <a href="#build">Build</a> •
  <a href="#examples">Examples</a> •
  <a href="#camera-configuration">Camera Configuration</a> •
  <a href="#license">License</a>
</p>
<br>

[OpenNI](https://github.com/occipital/openni2) provides a uniform interface that third party middleware developers can use to interact with depth sensors.
The ZED OpenNI driver transforms each ZED camera into a PrimeSense compatible depth sensors to use it with the [OpenNI 2 API](https://github.com/occipital/openni2).

## Build

### Install OpenNI2

The latest version of OpenNI2 is required. It can be downloaded as a precompiled library from the [Occipital website](https://structure.io/openni).
The ZED OpenNI2 driver supports Windows and Linux distributions.

#### Linux
Download the library for the correct architecture of your host system: 
[x86](https://s3.amazonaws.com/com.occipital.openni/OpenNI-Linux-x86-2.2.0.33.tar.bz2), 
[x64](https://s3.amazonaws.com/com.occipital.openni/OpenNI-Linux-x64-2.2.0.33.tar.bz2), 
[ARM](https://s3.amazonaws.com/com.occipital.openni/OpenNI-Linux-Arm-2.2.0.33.tar.bz2)

**Note:** For ARM64 devices the library must be compiled from sources.

Extract the library:

```
$ tar -xf OpenNI-Linux-<arch>-2.2.0.33.tar.bz2 
```

A new folder `OpenNI-Linux-<arch>-2.2` will be created containing the precompiled library, take note of its full path
because you will need in the next steps.

### Build and install the driver

#### Linux

OpenCV is optionally required to build all the examples.

_[Optional]_ Install OpenCV:

    $ sudo apt install libopencv-dev
    
Clone this repository: 

    $ git clone git@github.com:stereolabs/zed-openni.git

Build the driver (use the path to the OpenNI2 library noted above):

    $ cd zed-openni
    $ mkdir build
    $ cd build
    $ cmake .. -DOPENNI2_DIR='path-to-your-openni2-installation-folder' -DCMAKE_BUILD_TYPE=Release
    $ make -j$(nproc)

The library file of the driver will be automatically copied in the folder `<openni2-path>/Redist/OpenNI2/Drivers` and it will be automatically loaded by OpenNI2 during initialization.
Please be sure to have writing permission for the folder `<openni2-path>/Redist/OpenNI2/Drivers`.

## Examples

The driver comes with five examples demonstrating how to use it to retrieve and show color images, depth maps and point clouds.

* [SimpleMultiViewer](https://github.com/stereolabs/zed-openni/tree/main/examples/SimpleMultiViewer): creates an OpenGL window displaying the color/depth combined stream (key 1), the depth stream (key 2) and the color stream (key 3)
* [SimpleColor](https://github.com/stereolabs/zed-openni/tree/main/examples/SimpleColor): _[requires OpenCV]_ creates an OpenCV window displaying the color stream
* [SimpleDepth](https://github.com/stereolabs/zed-openni/tree/main/examples/SimpleDepth): _[requires OpenCV]_ creates an OpenCV window displaying the depth stream
* [SimpleRegistered](https://github.com/stereolabs/zed-openni/tree/main/examples/SimpleRegistered): _[requires OpenCV]_ creates two OpenCV windows displaying the synchronized color and depth streams
* [SimplePointCloud](https://github.com/stereolabs/zed-openni/tree/main/examples/SimplePointCloud): _[requires OpenCV]_ creates an OpenCV 3D window displaying the color stream mappend on a 3D point cloud

## Camera configuration


## License

This library is licensed under the LGPL License.

## Support
If you need assistance go to our Community site at https://community.stereolabs.com/

    








