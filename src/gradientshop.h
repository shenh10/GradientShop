#ifndef __GRADIENTSHOP_H__
#define __GRADIENTSHOP_H__

#ifndef __COMMON_H__
#include "common.h"
#endif
#ifndef __OPENCV_H_
#include "opencv.h"
#endif
class GradientShop{
private:
    int row_len;
    int col_len;
    int nchannels;
    bool isInRange(double x, double y, double width);
    double getColor(int x, int y);
    double retrieveColor(Mat &_src, int _channel, int x, int y);
    double gradient_x(int x, int y);
    double gradient_y(int x, int y);
    double gradientMagnitude(int px, int py);
    int gradientOrientation( int x, int y);
    double window_normalize( int x, int y , int width);
    int angleToPi(int angle);
    void bilinearInterpolation( Mat &m, int px, int py, double x, double y);
    void localEdgeLength();
    void localGradientSaliency();
    void _computePixel();
    void _computeGradientMagnitude(); 
    void _computeGradient();
    void _computeOrient();
    void _computeNormalizedMagnitude();
    void _computeEdgeLength();
    void _computeSaliency();
public:
    Mat src, orient, normaled_mag, len_edge, saliency_x, saliency_y, u_x, u_y, u, u_mag;

    string outdir;     
    int channel;
    int ITERATION ;
    GradientShop();
    GradientShop(Mat input, int c , string outdir);
    void initialize();
    void computeSaliency();
    void showImage(Mat src, string window_name);
    void writeImage(Mat src, string window_name);
};

#endif
