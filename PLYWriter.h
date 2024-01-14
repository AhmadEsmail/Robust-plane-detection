/* PLY format writer

 */

#ifndef PLY_WRITER
#define PLY_WRITER

#include <iostream>
#include <fstream>
#include "stdlib.h"
#include <opencv2/opencv.hpp>


#include <vector>


using namespace std;
using namespace cv;


void WritePLY(const char*,vector<Point3f>,vector<Point3i>);

#endif
