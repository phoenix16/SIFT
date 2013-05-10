#ifndef SIFT_H
#define SIFT_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ml/ml.hpp>
#include <iostream>
#include <stdio.h>
#include <vector>
#include <string>
#include <math.h>

#define ANTIALIAS_SIGMA         0.5
#define KERNEL_SIZE             5
#define CONTRAST_THRESHOLD      0.03
#define INIT_SIGMA              1.4142135	//sqrt(2)
#define PREBLUR_SIGMA           1.0
#define PI                      3.1415926535897932384626433832795
#define R_CURVATURE             10.0

using namespace cv;
using namespace std;

typedef std::vector<Mat> Array;
typedef std::vector<Array> TwoDArray;

class sift
{
private:
    Mat source;
    int num_octaves;
    int num_scales;

    // Create 2D arrays of Mats
    TwoDArray ScaleSpace, DoG, DoG_Keypts, Magnitude, Orientation;

    void CreateScaleSpace();
    void DoGExtrema();
    void FilterDoGExtrema();
    void AssignOrientations();
public:
    sift(Mat source, int num_octaves, int num_scales);
    void doSift();
    Mat append_images(Mat image1, Mat image2);
    void display_images(int oct, int sc, int mode);

};

#endif // SIFT_H

