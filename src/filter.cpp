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
        _src.at<double>(y, x) = val;
        return;
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
        Mat wx , wy, tmp;
        tmp = 1./(abs(gs[n].saliency_x)+1);
        cv::pow(tmp, sensitiveB, wx);
        tmp = 1./(abs(gs[n].saliency_x)+1);
        cv::pow(tmp, sensitiveB, wy);
        Mat gx = gs[n].u_x + c2* gs[n].saliency_x;
        Mat gy = gs[n].u_y + c2* gs[n].saliency_y;
        vector<double> v_x = cgSolver(d, gx, gy, wd, wx, wy);
        Mat pixel_out = Mat::zeros(row, col, CV_64F);
        cout << "Pixel Out Channel"<< pixel_out.channels() << endl;
        for(int y = 0; y < row; y ++){
            for(int x = 0; x < col; x++){
                int index = y*col + x;
                cout << v_x[index] << ",";
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

vector<double> Filter::cgSolver(Mat & _d, Mat & _gx, Mat & _gy, Mat &_wd, Mat &_wx, Mat &_wy){
    int row = src.rows, col = src.cols, vlen = row*col;
    SparseMatrix A(vlen, vlen);
    vector<double> b, v_x;
    b.resize(vlen);
    v_x.resize(vlen);
    const int neighIter = 4;
    int dx[neighIter] = {0, -1,  0, 1};
    int dy[neighIter] = {-1, 0,  1, 0};
    cout << "Counting A, b, initialize x..." << endl;
    for(int y = 0; y < row; y++ ){
        for(int x = 0; x < col; x ++){
            int index = y*col + x;
            double a_sum = 0.0, b_sum = 0.0;
            a_sum += _wd.at<double>(y,x);
            b_sum +=  _wd.at<double>(y,x) * _d.at<double>(y,x); 
            for(int i = 0; i < neighIter; i++){
                int new_x = x + dx[i];
                int new_y = y + dy[i];
                if(! isInRange(new_x, new_y, 0) ) continue;
                if(new_x != x){
                    a_sum += _wx.at<double>(new_y, new_x);   
                    if(new_x > x){
                        b_sum += - _wx.at<double>(new_y, new_x)* _gx.at<double>(new_y, new_x);
                    }else{
                        b_sum +=  _wx.at<double>(new_y, new_x)* _gx.at<double>(new_y, new_x);
                    }
                }
                if(new_y != y){
                    a_sum += _wy.at<double>(new_y, new_x); 
                    if(new_y > y){
                        b_sum +=  _wy.at<double>(new_y, new_x)* _gy.at<double>(new_y, new_x);
                    }else{
                        b_sum += - _wy.at<double>(new_y, new_x)* _gy.at<double>(new_y, new_x);
                    }

                }
            }
            A.insert( make_pair(index, index),  a_sum);
            b[index] = b_sum; 
            for(int i = 0; i < neighIter/2; i++){
                int new_x_l = x + dx[i];
                int new_y_l = y + dy[i];
                int new_x_r = x + dx[i+neighIter/2];
                int new_y_r = y + dy[i+neighIter/2];
                if(! isInRange(new_x_l, new_y_l, 0) || ! Range(new_x_r, new_y_r) ) continue;
                if(new_x_l != x){
                    A.insert(make_pair(y*col + new_x_l, y*col + new_x_r), - _wx.at<double>(y,x));
                    A.insert(make_pair(y*col + new_x_r, y*col + new_x_l), - _wx.at<double>(y,x));
                }
                if(new_y_l != y){
                    A.insert(make_pair(new_y_l*col + x, new_y_r*col + x), - _wy.at<double>(y,x));
                    A.insert(make_pair(new_y_r*col + x, new_y_l*col + x), - _wy.at<double>(y,x));

                }
            }
            v_x[index] = _d.at<double>(y,x);
        }
    }
    cout << "Start conjugate gradient optimization..." << endl;
    double sigma =  conjugateGradient(A, b, v_x );
    cout << "Optimization done, residual norm: "<< sigma <<"..." << endl;
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

