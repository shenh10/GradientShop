#include "sharpen_filter.h"
void SharpenFilter::computeFilter(){
    for(int n = 0; n < nchannels; n ++){
        gs[n].computeSaliency();  
        Mat d = gs[n].u;
        Mat wd = c1* Mat::ones(row, col, CV_64F);
        Mat wx , wy, tmp;
        tmp = 1./(abs(gs[n].saliency_x)+1);
        cv::pow(tmp, sensitiveB, wx);
        tmp = 1./(abs(gs[n].saliency_x)+1);
        cv::pow(tmp, sensitiveB, wy);
        Mat gx = gs[n].u_x + c2* gs[n].saliency_x;
        Mat gy = gs[n].u_y + c2* gs[n].saliency_y;
        vector<double> v_x = cgSolver(d, gx, gy, wd, wx, wy);
        Mat pixel_out = Mat::zeros(row, col, CV_64F);
        for(int y = 0; y < row; y ++){
            for(int x = 0; x < col; x++){
                int index = y*col + x;
                setMatValue(out, n, x, y, v_x[index]); 
                setMatValue(pixel_out, n, x,y, v_x[index]);
            }
        }
        pixel_out.convertTo(tmp, CV_8U);
        gs[n].writeCSV(tmp, "pixel_out");
        gs[n].writeImage(pixel_out, "pixel_out");
    }
    gs[0].writeImage(out, "out");
}
