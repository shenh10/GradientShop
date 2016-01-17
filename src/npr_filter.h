#ifndef __NPR_FILTER_H_
#define __NPR_FILTER_H_
#include "filter.h"
class NPRFilter: public Filter{
    double c1, c2, sigma;
    int sensitiveB;
public:
    NPRFilter();
    NPRFilter(Mat input, string outdir, int b = 9, double c1 = 0.019, double c2 = 1, double sigma = 20): Filter(input, outdir), c1(c1), c2(c2), sigma(sigma), sensitiveB(b){}
    void computeFilter();

};


#endif
