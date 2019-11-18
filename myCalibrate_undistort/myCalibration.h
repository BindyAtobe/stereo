#ifndef QUESTION8_MYCALIBRATION_H
#define QUESTION8_MYCALIBRATION_H

#include "opencv2/core.hpp"

using namespace cv;

double myCalibrateCamera(InputArrayOfArrays _objectPoints,
                         InputArrayOfArrays _imagePoints,
                         InputOutputArray _cameraMatrix, InputOutputArray _distCoeffs,
                         OutputArrayOfArrays _rvecs, OutputArrayOfArrays _tvecs);

#endif //QUESTION8_MYCALIBRATION_H
