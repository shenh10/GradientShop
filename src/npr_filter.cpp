#include "npr_filter.h"
void NPRFilter::computeFilter(){
    for(int n = 0; n < nchannels; n ++){
        gs[n].computeSaliency();  
        Mat d = gs[n].u;
        Mat wd = c1* Mat::ones(row, col, CV_64F);
        Mat wx , wy, tmp;
        tmp = 1./(abs(gs[n].saliency_x)+1);
        cv::pow(tmp, sensitiveB, wx);
        tmp = 1./(abs(gs[n].saliency_x)+1);
        cv::pow(tmp, sensitiveB, wy);
        cv::pow(gs[n].len_edge, 2, tmp);
        tmp =  - tmp / 2 / pow(sigma, 2); 
        cv::exp(tmp,tmp);
        Mat N = c2* (1 - tmp);
        // compute cos(Matrix)
        // gx = < < u_x , cos^2(orient)> , n>
        mo.cosMat((gs[n].orient)*PI/180, tmp);
        Mat gx = (gs[n].u_x).mul(N).mul(tmp);
        cout << gx << endl;
        mo.sinMat((gs[n].orient)*PI/180, tmp);
        Mat gy = (gs[n].u_y).mul(N).mul(tmp);
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
