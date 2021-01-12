![](./images/Picto+STEREOLABS_Black.jpg)

# Stereolabs ZED Camera - OpenNI2 driver

[OpenNI](https://github.com/occipital/openni2) provides a uniform interface that third party middleware developers can use to interact with depth sensors.
The ZED OpenNI driver transforms each ZED camera into a PrimeSense compatible depth sensors to use it with the [OpenNI 2 API](https://github.com/occipital/openni2).

## Installation

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

A new folder `OpenNI-Linux-<arch>-2.2` will be created containing the precompiled library, take note of its full path.

### Build and install the driver

#### Linux

OpenCV is required to build all the examples

* [Optional] Install OpenCV:
    `$ sudo apt install libopencv-dev`

* Clone this repository: 
    `$ git clone git@github.com:stereolabs/zed-openni.git`

* Build the driver:

```
    $ cd zed-openni
    $ mkdir build
    $ cd build
    $ cmake .. -DOPENNI2_DIR='path-to-your-openni2-installation-folder' -DCMAKE_BUILD_TYPE=Release
    $ make -j$(nproc)
```

The driver library file will be automatically copied in the folder `<openni2-path>/Redist/OpenNI2/Drivers` and it will be automatically loaded by OpenNI2 during initialization.

## Examples


    








