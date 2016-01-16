#ifndef __FILTER_H__
#include "filter.h"
#endif
int main( int argc, char** argv )
{
    /// Load the source image
    Mat src, thres_src, dst;
    string window_name[] = {"Input Image", "Threshed Image", "Output Image"};
    src = imread( argv[1], CV_LOAD_IMAGE_UNCHANGED ); 
    if (src.empty()) //check whether the image is loaded or not
     {
          cout << "Error : Image cannot be loaded..!!" << endl;
          //system("pause"); //wait for a key press
          return -1;
     }
    Filter filter(src, "output");
    filter.computeFilter();
    filter.out.convertTo(dst, CV_8U);
    cout << "Filter Done" << endl;
    waitKey(0);
    // Threshold the input image
   /* for (int i = 0 ; i < 3; i++){
        destroyWindow(window_name[i]);
    }*/

    return 0;
}
