#pragma once

// Coordinate frame tags — used as template parameters or documentation markers
// to make frame membership explicit without runtime overhead.
//
// W = world / ceiling map frame  (X,Y in ceiling plane metres, Z=0 at marker plane)
// C = OpenCV camera frame        (x right, y down, z forward)
// I = IMU frame                  (physical BNO085 frame as calibrated by Kalibr)
// O = output frame               (FreeD / UI convention)

struct WorldFrame  {};
struct CameraFrame {};
struct ImuFrame    {};
struct OutputFrame {};
