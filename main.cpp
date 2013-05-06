/*
 * Developer : Prakriti Chintalapoodi - c.prakriti@gmail.com 
*/

#include "sift.h"

#define NUM_OCTAVES 4
#define NUM_SCALES 5


int main(int argc, char* argv[])
{
	if (argc != 3)
	{
        printf("\nUsage: %s [source image] [image to be matched]", argv[0]);
		exit(1);
	}

	// Read the source and input to be matched as grayscale images
	Mat source = imread(argv[1], 0);
	Mat input = imread(argv[2], 0);

	sift feat(source, NUM_OCTAVES, NUM_SCALES);
	
	// Append the source and input images Left-Right and display
    Mat appended = feat.append_images(source, input);
    imshow("Appended", appended);

//    feat.doSift();
//    feat.display_images(2,4,3); // SS, DoG, DoGKP



//    Point2f a(0.3f, 0.f), b(0.f, 0.4f);
//    Point center = (a + b)*10.f;

//     int thickness = -1;
//     int lineType = 8;
//     float radius  = 0.3;

//     circle( source,
//             center,
//             radius,
//             Scalar( 0, 0, 255 ),
//             thickness,
//             lineType );


    waitKey(0);
	return 0;
}
