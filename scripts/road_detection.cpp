//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <unordered_set>
//#include <tr1/unordered_map>

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

/*
 * Given two lines, see if they are almost colinear, that is, there slopes are close
 * and they aren't close to the same part of the image as well. Should make it easier
 * to work with Hough Transforms that aren't quite tuned perfectly.
 */
bool are_almost_colinear(Vec4i l1, Vec4i l2) {
    // thresholds for how close the slope and y' distance should be for it to count
    int slope_threshold = 2;
    int dist_threshold = 10;
    //slopes
    int slope_l1 = (l1[1] - l1[3]) / (l1[0] - l1[2]);
    int slope_l2 = (l2[1] - l2[3]) / (l2[0] - l2[2]);
    //check whether slopes are close enough
    if (( slope_l1 + slope_threshold > slope_l2 ) and ( slope_l1 - slope_threshold < slope_l2 )) {
        //TODO check if order of endpoints is significant and make sure
        // we get the endpoint of each thats closest to eachother
        int a = l2[0];
        int b = l2[1];
        int c = l1[2];
        int d = l1[3];
        //The algebraic bs that I worked out, based on the following system of equations:
        // y - d = 1/slope_l1(x - c)
        // y - b = slope_l2(x - a)
        int x = ( d - b + slope_l2*a - (1/slope_l1)*c ) / ( slope_l2 - (1/slope_l1) );
        int y = slope_l2*(x - c) + d;
        int dist = sqrt ( pow( (x-c), 2 ) + pow( (y-d), 2 ) );
        if ( dist < dist_threshold ) {
            return true;
        }
    }
    return false;
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
        // getting average slope of the lines
        float total_slope = 0;
        float total_intercept = 0;
        // make a hashmap of the consolidated lines
        unordered_set<int> consolidated_lines;
        // display the top lines
        for( size_t i = 0; i < 10; i++ )
          {
            Vec4i l = lines[i];
            line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
            float slope = (l[0] - l[2]) / (l[1] - l[3]);
            float intercept = l[0] - slope*l[1];
            // hard coding a thing that avoids the horizon
            if (slope < 10.0) {
                total_slope += slope;
                total_intercept += intercept;
            }
            for ( auto it = consolidated_lines.begin(); it != consolidated_lines.end(); ++it ) {
                int l1_index = *it;
                Vec4i l1 = lines[l1_index];
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
