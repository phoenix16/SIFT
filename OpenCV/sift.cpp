#include "sift.h"

// Constructor that sets number of octaves and scales
sift::sift(Mat source, int num_octaves, int num_scales)
    :  source(source), num_octaves(num_octaves), num_scales(num_scales)
{
    // Allocate memory for Scale Space, DoGs
    for (int oct = 0; oct < num_octaves; ++oct)
    {
        ScaleSpace.push_back(Array(num_scales+3));
        DoG.push_back(Array(num_scales+2));       // 1 less than number of scales
        DoG_Keypts.push_back(Array(num_scales));  // 2 less that number of DoGs
        Magnitude.push_back(Array(num_scales));     // = number of scales with Keypts
        Orientation.push_back(Array(num_scales));   // = number of scales with Keypts
    }
}

// Public Function that does complete SIFT
void sift::doSift()
{
    CreateScaleSpace();
    DoGExtrema();
    FilterDoGExtrema();
    AssignOrientations();
}

// Public Function to append two images Left-Right
Mat sift::append_images(Mat image1, Mat image2)
{
    Mat appended; int max_rows;
    if (image1.rows > image2.rows)
    {
        max_rows = image1.rows;
    }
    else
    {
        max_rows = image2.rows;
    }
    appended.create(max_rows, image1.cols + image2.cols, image1.type());
    image1.copyTo(appended(Range(0, image1.rows), Range(0, image1.cols)));
    image2.copyTo(appended(Range(0, image2.rows), Range(image1.cols, image1.cols + image2.cols)));
    return appended;
}


// Private Function to create Scale Space
void sift::CreateScaleSpace()
{
    cout << "\n\nCreating Scale Space..." << endl;
    Size ksize(KERNEL_SIZE, KERNEL_SIZE);
    double sigma; Mat src_antialiased, up, down;

    // For octave 1, scale 1 image, source --> gaussian blur with sigma = 0.5 --> double size
    GaussianBlur(source, src_antialiased, ksize, ANTIALIAS_SIGMA);
    pyrUp(src_antialiased, up);
    up.copyTo(ScaleSpace[0][0]);

    //Pre-blur Octave 1, Scale 1 image
    GaussianBlur(ScaleSpace[0][0], ScaleSpace[0][0], ksize, PREBLUR_SIGMA);

    //imshow("after", ScaleSpace[0][0]);
    // Populate rest of the Scale Spaces
    for (int oct = 0; oct < num_octaves; oct++)
    {
        sigma = INIT_SIGMA;  // reset sigma for each octave
        for (int sc = 0; sc < num_scales+2; sc++)
        {
            sigma = sigma * pow(2.0,sc/2.0) ;

            // Apply blur to get next scale in same octave
            GaussianBlur(ScaleSpace[oct][sc], ScaleSpace[oct][sc+1], ksize, sigma);

            // DoG = Difference of Adjacent scales
            DoG[oct][sc] = ScaleSpace[oct][sc] - ScaleSpace[oct][sc+1];

//            cout << "Octave : " << oct << "   Scale : " << sc << "   ScaleSpace size : " << ScaleSpace[oct][sc].rows << "x" << ScaleSpace[oct][sc].cols << endl;
        }

        //Create the next octave image if not reached the last octave
        //Next octave's first scale = prev octave's first scale downsized by half
        if (oct < num_octaves-1)
        {
            pyrDown(ScaleSpace[oct][0], down);
            down.copyTo(ScaleSpace[oct+1][0]);
        }
    }
}


// Private Function to find the extrema keypoints given 3 matrices
// A pixel is a keypoint if it is the extremum of its 26 neighbors
// (8 in current, and 9 each in top and bottom)
void sift::DoGExtrema()
{
    cout << "\n\nFinding DoG Extrema..." << endl;
    Mat local_maxima, local_minima, extrema, current, top, down;

    for (int oct = 0; oct < num_octaves; oct++)
    {
        for (int sc = 0; sc < num_scales; sc++)
        {
            // Initialize DoG_keypts matrix with zeros (needed for 1 pixel border)
            DoG_Keypts[oct][sc] = Mat::zeros(DoG[oct][sc].size(), DoG[oct][sc].type());
            top     = DoG[oct][sc];
            current = DoG[oct][sc+1];
            down    = DoG[oct][sc+2];
            int sx = current.rows; int sy = current.cols;

            // Look for local maxima
            // Check the 8 neighbors around the pixel in the same image
            // Range is [lower_bound, upper_bound)
            local_maxima = (current(Range(1,sx-1),Range(1,sy-1)) > current(Range(0,sx-2),Range(0,sy-2))) &
                           (current(Range(1,sx-1),Range(1,sy-1)) > current(Range(0,sx-2),Range(1,sy-1))) &
                           (current(Range(1,sx-1),Range(1,sy-1)) > current(Range(0,sx-2),Range(2,sy  ))) &
                           (current(Range(1,sx-1),Range(1,sy-1)) > current(Range(1,sx-1),Range(0,sy-2))) &
                           (current(Range(1,sx-1),Range(1,sy-1)) > current(Range(1,sx-1),Range(2,sy  ))) &
                           (current(Range(1,sx-1),Range(1,sy-1)) > current(Range(2,sx  ),Range(0,sy-2))) &
                           (current(Range(1,sx-1),Range(1,sy-1)) > current(Range(2,sx  ),Range(1,sy-1))) &
                           (current(Range(1,sx-1),Range(1,sy-1)) > current(Range(2,sx  ),Range(2,sy  ))) ;

            // Check the 9 neighbors in the image above it
            local_maxima = local_maxima & (current(Range(1,sx-1),Range(1,sy-1)) > top(Range(0,sx-2),Range(0,sy-2))) &
                                          (current(Range(1,sx-1),Range(1,sy-1)) > top(Range(0,sx-2),Range(1,sy-1))) &
                                          (current(Range(1,sx-1),Range(1,sy-1)) > top(Range(0,sx-2),Range(2,sy  ))) &
                                          (current(Range(1,sx-1),Range(1,sy-1)) > top(Range(1,sx-1),Range(0,sy-2))) &
                                          (current(Range(1,sx-1),Range(1,sy-1)) > top(Range(1,sx-1),Range(1,sy-1))) &  // same pixel in top
                                          (current(Range(1,sx-1),Range(1,sy-1)) > top(Range(1,sx-1),Range(2,sy  ))) &
                                          (current(Range(1,sx-1),Range(1,sy-1)) > top(Range(2,sx  ),Range(0,sy-2))) &
                                          (current(Range(1,sx-1),Range(1,sy-1)) > top(Range(2,sx  ),Range(1,sy-1))) &
                                          (current(Range(1,sx-1),Range(1,sy-1)) > top(Range(2,sx  ),Range(2,sy  )));

            // Check the 9 neighbors in the image below it
            local_maxima = local_maxima & (current(Range(1,sx-1),Range(1,sy-1)) > down(Range(0,sx-2),Range(0,sy-2))) &
                                          (current(Range(1,sx-1),Range(1,sy-1)) > down(Range(0,sx-2),Range(1,sy-1))) &
                                          (current(Range(1,sx-1),Range(1,sy-1)) > down(Range(0,sx-2),Range(2,sy  ))) &
                                          (current(Range(1,sx-1),Range(1,sy-1)) > down(Range(1,sx-1),Range(0,sy-2))) &
                                          (current(Range(1,sx-1),Range(1,sy-1)) > down(Range(1,sx-1),Range(1,sy-1))) &  // same pixel in down
                                          (current(Range(1,sx-1),Range(1,sy-1)) > down(Range(1,sx-1),Range(2,sy  ))) &
                                          (current(Range(1,sx-1),Range(1,sy-1)) > down(Range(2,sx  ),Range(0,sy-2))) &
                                          (current(Range(1,sx-1),Range(1,sy-1)) > down(Range(2,sx  ),Range(1,sy-1))) &
                                          (current(Range(1,sx-1),Range(1,sy-1)) > down(Range(2,sx  ),Range(2,sy  )));

            // Look for local minima
            // Check the 8 neighbors around the pixel in the same image
            local_minima = (current(Range(1,sx-1),Range(1,sy-1)) < current(Range(0,sx-2),Range(0,sy-2))) &
                           (current(Range(1,sx-1),Range(1,sy-1)) < current(Range(0,sx-2),Range(1,sy-1))) &
                           (current(Range(1,sx-1),Range(1,sy-1)) < current(Range(0,sx-2),Range(2,sy  ))) &
                           (current(Range(1,sx-1),Range(1,sy-1)) < current(Range(1,sx-1),Range(0,sy-2))) &
                           (current(Range(1,sx-1),Range(1,sy-1)) < current(Range(1,sx-1),Range(2,sy  ))) &
                           (current(Range(1,sx-1),Range(1,sy-1)) < current(Range(2,sx  ),Range(0,sy-2))) &
                           (current(Range(1,sx-1),Range(1,sy-1)) < current(Range(2,sx  ),Range(1,sy-1))) &
                           (current(Range(1,sx-1),Range(1,sy-1)) < current(Range(2,sx  ),Range(2,sy  ))) ;

            // Check the 9 neighbors in the image above it
            local_minima = local_minima & (current(Range(1,sx-1),Range(1,sy-1)) < top(Range(0,sx-2),Range(0,sy-2))) &
                                          (current(Range(1,sx-1),Range(1,sy-1)) < top(Range(0,sx-2),Range(1,sy-1))) &
                                          (current(Range(1,sx-1),Range(1,sy-1)) < top(Range(0,sx-2),Range(2,sy  ))) &
                                          (current(Range(1,sx-1),Range(1,sy-1)) < top(Range(1,sx-1),Range(0,sy-2))) &
                                          (current(Range(1,sx-1),Range(1,sy-1)) < top(Range(1,sx-1),Range(1,sy-1))) &  // same pixel in top
                                          (current(Range(1,sx-1),Range(1,sy-1)) < top(Range(1,sx-1),Range(2,sy  ))) &
                                          (current(Range(1,sx-1),Range(1,sy-1)) < top(Range(2,sx  ),Range(0,sy-2))) &
                                          (current(Range(1,sx-1),Range(1,sy-1)) < top(Range(2,sx  ),Range(1,sy-1))) &
                                          (current(Range(1,sx-1),Range(1,sy-1)) < top(Range(2,sx  ),Range(2,sy  )));

            // Check the 9 neighbors in the image below it
            local_minima = local_minima & (current(Range(1,sx-1),Range(1,sy-1)) < down(Range(0,sx-2),Range(0,sy-2))) &
                                          (current(Range(1,sx-1),Range(1,sy-1)) < down(Range(0,sx-2),Range(1,sy-1))) &
                                          (current(Range(1,sx-1),Range(1,sy-1)) < down(Range(0,sx-2),Range(2,sy  ))) &
                                          (current(Range(1,sx-1),Range(1,sy-1)) < down(Range(1,sx-1),Range(0,sy-2))) &
                                          (current(Range(1,sx-1),Range(1,sy-1)) < down(Range(1,sx-1),Range(1,sy-1))) &  // same pixel in down
                                          (current(Range(1,sx-1),Range(1,sy-1)) < down(Range(1,sx-1),Range(2,sy  ))) &
                                          (current(Range(1,sx-1),Range(1,sy-1)) < down(Range(2,sx  ),Range(0,sy-2))) &
                                          (current(Range(1,sx-1),Range(1,sy-1)) < down(Range(2,sx  ),Range(1,sy-1))) &
                                          (current(Range(1,sx-1),Range(1,sy-1)) < down(Range(2,sx  ),Range(2,sy  )));

            extrema = local_maxima | local_minima;

            // extrema has 2 rows and 2 cols less than corresponding ScaleSpace image. So copy it to correct location
            extrema.copyTo(DoG_Keypts[oct][sc](Range(1, DoG_Keypts[oct][sc].rows-1), Range(1, DoG_Keypts[oct][sc].cols-1)));
//            cout << "Octave : " << oct << "   Scale : " << sc << "   DoG_Keypts size : " << DoG_Keypts[oct][sc].rows << "x" << DoG_Keypts[oct][sc].cols << endl;
        }
    }
}

// Private function to filter out extrema with low contrast or on edges
void sift::FilterDoGExtrema()
{
    cout << "\n\nRejecting keypoints with low contrast or on edges...\n\n";
    Mat locs; int x, y, rx, ry, fxx, fxy, fyy, deter;
    float trace, curvature;
    float curv_threshold = ((R_CURVATURE+1)*(R_CURVATURE+1))/R_CURVATURE;

    for (int oct = 0; oct < num_octaves; oct++)
    {
        for (int sc = 0; sc < num_scales; sc++)
        {
            int reject_contrast_count = 0, reject_edge_count = 0;
            cv::findNonZero(DoG_Keypts[oct][sc], locs);       // locations of key points
            int num_keypts = locs.rows;                       // number of Keypoints
            Mat_<uchar> current = DoG[oct][sc+1];

            for (int k = 0; k < num_keypts; k++)
            {
                x = locs.at<int>(k,0);
                y = locs.at<int>(k,1);

                // Discard low contrast points
                if (abs(current(x+1,y+1)) < CONTRAST_THRESHOLD)
                {
                    DoG_Keypts[oct][sc].at<uchar>(x,y) = 0;
                    reject_contrast_count++;
                }
                // Discard extrema on edges
                else
                {
                    rx = x+1;
                    ry = y+1;
                    // Get the elements of the 2x2 Hessian Matrix
                    fxx = current(rx-1,ry) + current(rx+1,ry) - 2*current(rx,ry);   // 2nd order derivate in x direction
                    fyy = current(rx,ry-1) + current(rx,ry+1) - 2*current(rx,ry);   // 2nd order derivate in y direction
                    fxy = current(rx-1,ry-1) + current(rx+1,ry+1) - current(rx-1,ry+1) - current(rx+1,ry-1); // Partial derivate in x and y direction
                    // Find Trace and Determinant of this Hessian
                    trace = (float)(fxx + fyy);
                    deter = (fxx*fyy) - (fxy*fxy);
                    curvature = (float)(trace*trace/deter);
                    if (deter < 0 || curvature > curv_threshold)   // Reject edge points if curvature condition is not satisfied
                    {
                        DoG_Keypts[oct][sc].at<uchar>(x,y) = 0;
                        reject_edge_count++;
                    }
                }
            }
            printf("\tOctave : %d  Scale : %d   Keypoints : %5d  Rejected (%4d contrast, %4d edge): %5d\n", oct+1, sc+1, num_keypts, reject_contrast_count, reject_edge_count, reject_contrast_count + reject_edge_count);
        }
    }
}

// Private function to assign gradient direction and magnitude of the gaussian pyramid images in Scale Space
void sift::AssignOrientations()
{
    cout << "\nAssigning Orientations to keypoints...\n\n";

    for (int oct = 0; oct < num_octaves; oct++)
    {
        for (int sc = 0; sc < num_scales; sc++)
        {
            Mat_<uchar> current = ScaleSpace[oct][sc+1];
            Magnitude[oct][sc] = Mat::zeros(current.size(), CV_64FC1);
            Orientation[oct][sc] = Mat::zeros(current.size(), CV_64FC1);

            for (int x = 1; x < current.rows-1; x++)
            {
                for (int y = 1; y < current.cols-1; y++)
                {
                    // compute x and y derivatives using pixel differences
                    double dx = current(x+1,y) - current(x-1,y);
                    double dy = current(x,y+1) - current(x,y-1);

                    // compute the magnitude and orientation of the gradient
                    Magnitude[oct][sc].at<double>(x,y)   = sqrt(dx * dx + dy * dy);
                    Orientation[oct][sc].at<double>(x,y) = (atan2(dy,dx) == PI)? -PI : atan2(dy,dx);

//                    printf("\tLocation[%d][%d]: Magnitude = %.3f \t Orientation = %.3f\n", x, y, Magnitude[oct][sc].at<uchar>(x,y), Orientation[oct][sc].at<uchar>(x,y));
                }
            }
        }
    }
}






// Public function to display the requested image
void sift::display_images(int oct, int sc, int mode)
{
    stringstream ss_oct, ss_sc;
    ss_oct << oct; ss_sc << sc;

    switch (mode)
    {
        case 1:        // Display the Scale Space images
        {
        string wSS = "SS Octave " + ss_oct.str() + " Scale " + ss_sc.str();
            imshow(wSS, ScaleSpace[oct-1][sc-1]);
            break;
        }
        case 2:        // Display the DoG images
        {
            string wDoG = "DoG Octave " + ss_oct.str() + " Scale " + ss_sc.str();
            imshow(wDoG, DoG[oct-1][sc-1]);
            break;
        }
        case 3:       // Display DoG Keypoints
        {
            string wDoGKP = "DoG Keypts Octave " + ss_oct.str() + " Scale " + ss_sc.str();
            imshow(wDoGKP, DoG_Keypts[oct-1][sc-1]);
            break;
        }
    }
}

