# Vessel State and Coordinate Conversion

A small C++ library for representing vessel state and converting it between different coordinate conventions used by robotics and simulation tools.

## Overview

This project defines a `VesselInformation` data structure and conversion logic for translating vessel pose and motion data between common frame conventions, including:

- Gazebo / ENU_FLU
- NED_FRD
- EUN_FUL (Unity-style)
- NEU_FRU (Unreal-style)

The library is designed to handle the subtle differences between world-frame and body-frame velocities, quaternion rotations, and axis re-labeling while preserving physically correct motion data.

## Features

- Represent vessel pose, time, velocity, and entity metadata in a unified structure
- Convert from Gazebo state to xDyn, Unity, and Unreal conventions
- Convert xDyn data back into the Gazebo convention
- Validate conversions with unit tests built with GoogleTest
- Support for correct handling of angular velocity pseudovector transforms

## Project structure

- `vessel.hpp` — public declarations and coordinate convention definitions
- `vessel.cpp` — conversion logic and frame transformation implementation
- `vessel_test.cpp` — unit tests covering conversion correctness
- `CMakeLists.txt` — CMake build configuration

## Supported conventions

The project uses the following coordinate conventions:

- `GAZEBO` — ENU world frame and FLU body frame
- `ENU_FLU` — East-North-Up world, Forward-Left-Up body
- `NED_FRD` — North-East-Down world, Forward-Right-Down body
- `EUN_FUL` — Unity convention
- `NEU_FRU` — Unreal convention

## Build

Requirements:

- CMake 3.16+
- Gazebo Sim development packages
- GoogleTest

From the project root:

```bash
mkdir -p build
cd build
cmake ..
make
```

## Run tests

```bash
cd build
ctest --output-on-failure
```

Alternatively, the test binary can be run directly:

```bash
./vessel_test
```

## Example usage

```cpp
#include "vessel.hpp"

VesselInformation state(
    Convention::GAZEBO,
    0.0,
    gz::math::Pose3d(gz::math::Vector3d(10.0, 20.0, 30.0), gz::math::Quaterniond::Identity),
    gz::math::Vector3d(1.0, 2.0, 3.0),
    gz::math::Vector3d(0.1, 0.2, 0.3));

VesselInformation xdyn_state = state.to_xdyn();
VesselInformation unity_state = state.to_unity();
VesselInformation unreal_state = state.to_unreal();
```

## Notes

This project is focused on correctness of coordinate transforms rather than GUI or simulation runtime behavior. The conversion code is explicitly written to preserve orientation and velocity semantics when moving between conventions with different axis orderings and handedness.
