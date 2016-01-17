#ifndef __SHARPEN_FILTER_H_
#define __SHARPEN_FILTER_H_
#include "filter.h"
class SharpenFilter: public Filter{
    int sensitiveB ;
    double c1, c2;
public:
    SharpenFilter();
    SharpenFilter(Mat input, string outdir, int b=5, double c1=0.03, double c2=1): Filter(input, outdir), sensitiveB(b), c1(c1), c2(c2) {}
    void computeFilter();

};


#endif
