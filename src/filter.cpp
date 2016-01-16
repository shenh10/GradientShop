#include "filter.h"
Filter::Filter(Mat input, string outdir){
    iMAX = 1000;
    reAdjust = 50;
    nchannels = input.channels();
    gs.resize(nchannels);
    src = input;
    row = src.rows;
    col = src.cols;
    outdir = outdir;
    for(int i = 0; i < nchannels; i ++){
        gs[i].channel = i;
        gs[i].src = src;
        gs[i].outdir = outdir;
        gs[i].initialize(); 
    }
    src.convertTo(out, CV_64F);
}


bool Filter::isInRange(int x, int y, int width){
    if (x >=  width && x < col - width && y >= width && y < row - width)
        return true;
    else
        return false;
}
double Filter::getMatValue(Mat &_src, int _channel, int x, int y){
    if (_src.channels() == 1){
        return _src.at<double>(y, x);
    }
    Vec3b intensity = _src.at<Vec3b>(y, x);
    return intensity.val[_channel];
}

void Filter::setMatValue(Mat &_src, int _channel, int x, int y, double val){
    if (_src.channels() == 1){
        _src.at<uchar>(y, x) = val;
    }
    Vec3b intensity = _src.at<Vec3b>(y, x);
    intensity.val[_channel] = val;
}
double Filter::gradient_x(Mat &_src, int _channel, int x, int y){
    if( ! isInRange(x, y, 1) ) return 0;
    double dx = getMatValue( _src, _channel, x + 1, y) - getMatValue( _src, _channel, x - 1, y);
    return dx;
}   
double Filter::gradient_y(Mat &_src, int _channel, int x, int y){
    if( ! isInRange(x, y, 1) ) return 0;
    double dy = getMatValue( _src, _channel, x, y - 1 ) - getMatValue( _src, _channel,  x , y + 1);
    return dy;
}   

void Filter::computeFilter(){
    for(int n = 0; n < nchannels; n ++){
        gs[n].computeSaliency();  
        double c1 = 2, c2 = 2;
        Mat d = gs[n].u;
        Mat wd = c1* Mat::ones(row, col, CV_64F);
        Mat wx =  Mat::ones(row, col, CV_64F);
        Mat wy =  Mat::ones(row, col, CV_64F);
        Mat gx = gs[n].u_x + c2* gs[n].saliency_x;
        Mat gy = gs[n].u_y + c2* gs[n].saliency_y;
        vector<double> v_x = cgSolver(d, gx, gy, wd, wx, wy);
        for(int y = 0; y < row; y ++){
            for(int x = 0; x < col; x++){
                int index = y*col + x;
                setMatValue(out, n, x, y, v_x[index]); 
            }
        }
        cout << out << endl;
        gs[0].writeImage(out, "output");
    }
}

vector<double> Filter::cgSolver(Mat & _d, Mat & _gx, Mat & _gy, Mat &_wd, Mat &_wx, Mat &_wy){
    int row = src.rows, col = src.cols, vlen = row*col;
    SparseMatrix A(vlen, vlen);
    vector<double> b, v_x;
    b.resize(vlen);
    v_x.resize(vlen);
    cout << "Counting A, b, initialize x..." << endl;
    for(int y = 1; y < row-1; y++ ){
        for(int x = 1; x < col-1; x ++){
            int index = y*col + x;
            A.insert( make_pair(index, index),  2* ( _wd.at<double>(y, x) + _wx.at<double>(y, x-1) + _wx.at<double>(y, x + 1) + _wy.at<double>(y-1, x) + _wy.at<double>( y+1, x) ));
            b[index] = 4* ( _wd.at<double>(y,x) * _d.at<double>(y,x) + _wx.at<double>(y,x-1) * _gx.at<double>(y,x-1) + _wy.at<double>(y+1,x) * _gy.at<double>(y+1,x) - _wx.at<double>(y,x+1) * _gx.at<double>(y,x + 1) - _wy.at<double>(y-1,x) * _gy.at<double>(y-1,x));
            A.insert(make_pair( (y-1)*col+x, (y+1)*col+x),  -4 * _wy.at<double>(y,x));
            A.insert( make_pair((y+1)*col+x, (y-1)*col+x) , -4 * _wy.at<double>(y,x));
            A.insert( make_pair( y*col+x+1, y*col+x-1) , -4 * _wx.at<double>(y,x));
            A.insert( make_pair( y*col+x-1, y*col+x+1) , -4 * _wx.at<double>(y,x));
            v_x[index] = _d.at<double>(y,x);
        }
    }
    cout << "Start conjugate gradient optimization..." << endl;
    printVec(b);
    printVec(v_x);
    double sigma =  conjugateGradient(A, b, v_x );
    cout << "Optimization done, residual norm: "<< sigma <<"..." << endl;
    printVec(v_x);
    return v_x;
}
double Filter::conjugateGradient(SparseMatrix &A,vector<double> &b, vector<double> &v_x){
    int i = 0;
    vector<double>  r, d, tmp;
    double sigma, sigma0, epsilon = 0.00000001;
    // r = b - A*v_x
    cout << "Initial r..." << endl;
    if (! mo.matrixVectorProduct(A, v_x, tmp )) return -1;
    if (! mo.vectorSubtract( b , tmp, r )) return -1;
    // d = r
    cout << "Initial d..." << endl;
    d.assign(r.begin(), r.end());
    // sigma = r^Tr
    cout << "Initial sigma..." << endl;
    if ( ! mo.vectorInnerProduct(r, r , sigma) ) return -1;
    sigma0 = sigma;
    while( i < iMAX && sigma > epsilon* sigma0){
        cout << "Iteration: " << i << "... Residual norm: " << sigma << "..."<< endl;
        vector<double> q;
        double alpha, sigma_old;
        // q = Ad
        if( ! mo.matrixVectorProduct( A , d, q) ) return -1;
        // alpha = sigma/(d^Tq)
        if( ! mo.vectorInnerProduct(d, q, alpha )) return -1;
        alpha =  alpha != 0 ? sigma / alpha : sigma / 0.0001;
        // v_x =  v_x + alpha *d 
        mo.vectorScalarProd(d, tmp, alpha);
        if( !mo.vectorSelfAdd(v_x, tmp) ) return -1;
        if( i % reAdjust == 0){
            // r = b - A*v_x 
            if (! mo.matrixVectorProduct(A, v_x, tmp )) return -1;
            if (! mo.vectorSubtract( b , tmp, r )) return -1;
        }else{
            // r = r - alpha*q
            mo.vectorScalarProd(q, tmp, alpha);
            if (! mo.vectorSelfSubtract(r, tmp)  ) return -1;
        }
        // sigma_old = sigma          
        sigma_old = sigma;
        // sigma = r^Tr 
        if ( ! mo.vectorInnerProduct(r, r , sigma) ) return -1;
        // d = r + sigma/sigma_old * d
        mo.vectorScalarProd(d, tmp, sigma/sigma_old);
        if ( ! mo.vectorAdd(r, tmp, d) ) return -1;
        // i++
        i++;
    }
    return sigma;
}

