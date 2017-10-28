//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

using namespace cv;
using namespace std;

//images
Mat src, src_gray;
Mat dst, blurred, cdst;
//threshold values, who knows what they mean
int edgeThreshold = 1;
int lowThreshold = 100;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;
//name of the image window
char* edgemap_window_name = "Edge Map";
char* lines_window_name = "Hough Lines";

/*
 * Comparator for sorting vec4i by line length, longest first
 */
bool vec4_linelength_comparator(Vec4i i, Vec4i j) {
    int i_length = sqrt ( pow((i[0] - i[2]), 2) + pow((i[1] - i[3]), 2) );
    int j_length = sqrt ( pow((j[0] - j[2]), 2) + pow((j[1] - j[3]), 2) );
    return (i_length > j_length);
}

int main( int argc, char **argv ) {
    //check if you've provided an image name
    if (argc != 2) {
        cout << "Provide an image filename as an argument" << endl;
        return -1;
    }
    // load the image
    src = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    // see if the image loading worked 
    if (!src.data) {
        cout << "Could not find image. " << std::endl;
        return -1;
    }
    //make a window for the image
    namedWindow(edgemap_window_name, CV_NORMAL);
    namedWindow(lines_window_name, CV_NORMAL);
    // make a matrix the same size as src
    //dst.create( src.size(), src.type() );
    // make a grayscale image 
    cvtColor( src, src_gray, CV_BGR2GRAY );
    // do a gaussian? blur
    blur(src_gray, blurred, Size(3,3));
    // do the edge detection
    Canny(blurred, dst, lowThreshold, lowThreshold*ratio, kernel_size);
    if (!dst.empty()) {
        cout << dst.channels();
        //make a vector of lines and do Hough Lines stuff 
        vector<Vec4i> lines;
        HoughLinesP(dst, lines, 1, CV_PI/180, 80, 30, 10);
        // sort the lines
        sort ( lines.begin(), lines.end(), vec4_linelength_comparator);
        Size s = dst.size();
        cdst = Mat::zeros(s.height, s.width, CV_8UC3);
        //display the lines I guess
        float total_slope = 0;
        float total_intercept = 0;
        for( size_t i = 0; i < 10; i++ )
          {
            Vec4i l = lines[i];
            line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
            float slope = (l[0] - l[2]) / (l[1] - l[3]);
            float intercept = l[0] - slope*l[1];
            printf("%f\n", slope);
            if (slope < 10.0) {
                total_slope += slope;
                total_intercept += intercept;
            }
          }
        // average of the top two lines
        float avg_x = (lines[0][0] + lines[1][0]) / 2.0;
        float avg_y = (lines[0][1] + lines[1][1]) / 2.0;
        // midpoint of the image
        float mid_x = cdst.size().width / 2.0;

        float avg_slope = total_slope / 10.0;
        printf("%f", avg_slope);
        //std::cout << avg_slope;
        float avg_intercept = total_intercept / 10.0;
        //line( cdst, Point(mid_x, cdst.size().height), Point(avg_x, avg_y), Scalar(0,255,0), 3, CV_AA);
        line( cdst, Point(mid_x, cdst.size().height), Point(mid_x + (avg_slope*cdst.size().height), 0), Scalar(255, 0, 0), 3, CV_AA);
        //show the image
        imshow(edgemap_window_name, dst);

        imshow(lines_window_name, cdst);

        waitKey(0);
        return 0;
    } else {
        cout << "no output array";
        return -1;
    }
}
