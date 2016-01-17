#ifndef __FILTER_H__
#define __FILTER_H__

#ifndef __GRADIENTSHOP_H__
#include "gradientshop.h"
#endif

#ifndef __MATRIXOPER_H__
#include "matrixoper.h"
#endif

class Filter{
protected:
    MatrixOper mo;
    int row, col;
    int iMAX;
    int reAdjust;
    int nchannels ;
    vector<GradientShop> gs;
public:
    Mat src, out;

    Filter(Mat input, string outdir);
    
    
    bool isInRange(int x, int y, int width);
    double getMatValue(Mat &_src, int _channel, int x, int y);

    void setMatValue(Mat &_src, int _channel, int x, int y, double val);
    double gradient_x(Mat &_src, int _channel, int x, int y);
    double gradient_y(Mat &_src, int _channel, int x, int y);
    virtual void computeFilter() = 0;
    vector<double> cgSolver(Mat & _d, Mat & _gx, Mat & _gy, Mat &_wd, Mat &_wx, Mat &_wy);
    double conjugateGradient(SparseMatrix &A,vector<double> &b, vector<double> &v_x);
};

#endif
