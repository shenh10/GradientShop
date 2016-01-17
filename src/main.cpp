#ifndef __FILTER_H__
#include "sharpen_filter.h"
#include "npr_filter.h"
#endif
#include "sys/stat.h"
int main( int argc, char** argv )
{
    if(argc < 6){
        cout << "Usage: ./GradientShop [path/to/image] [sharpen b c1 c2]/[npr b c1 c2 sigma]";
        return 1;
    }
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
    Filter *filter;
    string s = string(argv[2]);
    ostringstream outpath;  
    outpath << "output/" << s << "/" ;
    if( s == "sharpen" ){
        int b = atoi(argv[3]);
        double c1 = atof(argv[4]);
        double c2 = atof(argv[5]);
        for(int i = 3; i < 6; i++){
            outpath << argv[i] << "_";
        }
        string command = "mkdir -p " + outpath.str();
 
        system( command.c_str()  );
        filter = new SharpenFilter(src, outpath.str(), b, c1, c2);
    }else if (s == "npr"){

        int b = atoi(argv[3]);
        double c1 = atof(argv[4]);
        double c2 = atof(argv[5]);
        double sigma = atof(argv[6]);
        for(int i = 3; i < 7; i++){
            outpath << argv[i] << "_";
        }
        string command = "mkdir -p " + outpath.str();
 
        system( command.c_str()  );
        filter = new NPRFilter(src, outpath.str(), b, c1,c2, sigma);
    }else{
        cout << "Not support."<< endl;
        return 1;

    }
    filter->computeFilter();
    filter->out.convertTo(dst, CV_8U);
    cout << "Filter Done" << endl;
    waitKey(0);
    // Threshold the input image
    /* for (int i = 0 ; i < 3; i++){
       destroyWindow(window_name[i]);
       }*/

    return 0;
}
