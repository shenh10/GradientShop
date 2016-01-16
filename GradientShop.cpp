#include "gradientshop.h"
using namespace std;
using namespace cv;
bool GradientShop::isInRange(int x, int y, int width){
    if (x >=  width && x < row_len - width && y >= width && y < col_len - width)
        return true;
    else
        return false;
}
uchar GradientShop::getColor(int x, int y){
    return retrieveColor(src, channel, x, y);
}
uchar GradientShop::retrieveColor(Mat &_src, int _channel, int x, int y){
    if (_src.channels() == 1){
        return uchar(_src.at<uchar>(y, x));
    }
    Vec3b intensity = _src.at<Vec3b>(y, x);
    return intensity.val[_channel];
}
double GradientShop::gradient_x(int x, int y){
    if( ! isInRange(x, y, 1) ) return 0;
    uchar dx = getColor( x + 1, y) - getColor( x - 1, y);
    return double(dx);
}   
double GradientShop::gradient_y(int x, int y){
    if( ! isInRange(x, y, 1) ) return 0;
    uchar dy = getColor( x, y - 1 ) - getColor( x , y + 1);
    return double(dy);
}   
double GradientShop::gradientMagnitude(int px, int py){
    return sqrt(px^2 + py ^2);
}
int GradientShop::gradientOrientation( int x, int y){
    double dx = gradient_x( x, y), dy = gradient_y( x, y);
    if(dx == 0){
        if(dy > 0)  return 90;
        else if(dy < 0) return -90;
        else return 0;
    }
    double theta = atan(dy/dx)*180/PI;
    if(theta < 0) return floor(theta);
    else return ceil(theta);
}
double GradientShop::window_normalize( int x, int y , int width){
    CV_Assert(width % 2 != 0 && width > 1);
    if( isInRange( x, y, (width -1 )/2 )) {
        double avg = 0.0, sigma = 0.0, epsilon = 0.01, offset = width/2;
        Mat window = Mat::zeros(width, width, CV_8U);
        for (int i = - width/2; i < width/2 + 1 ; i++)
            for (int j = - width/2; j < width/2 + 1 ; j++)
                window.at<uchar>(offset + j, offset + i) = getColor( x + i, y + j);
        for (int i = - width/2; i < width/2 + 1 ; i++)
            for (int j = - width/2; j < width/2 + 1 ; j++)
                avg  += window.at<uchar>(offset + j , offset + i);
        avg /= ( width * width );
        for (int i = - width/2; i < width/2 + 1 ; i++)
            for (int j = - width/2; j < width/2 + 1 ; j++)
                sigma  += pow( double( window.at<uchar>(offset + j, offset + i) - avg ) ,2);
        sigma = sqrt(sigma/(width * width));
        return (window.at<uchar>(offset , offset) - avg)/(sigma + epsilon);
    }
    return 0;

}
int GradientShop::angleToPi(int angle){
    return angle * PI / 180.0;
}
void GradientShop::bilinearInterpolation( Mat &m, int px, int py, double x, double y){
    int lx = floor(x),rx = ceil(x), ly = floor(y), ry = ceil(y);
    vector<double>  w_alpha, w_theta;
    w_alpha.resize(4);
    w_theta.resize(4);
    w_alpha.push_back(1.0*(rx - x)*(ry - y)/((rx - lx)*(ry - ly)));
    w_alpha.push_back(1.0*(x - lx)*(ry - y)/((rx - lx)*(ry - ly)));
    w_alpha.push_back(1.0*(rx - x)*(y - ly)/((rx - lx)*(ry - ly)));
    w_alpha.push_back(1.0*(x - lx)*(y - ly)/((rx - lx)*(ry - ly)));
    double orient_p = angleToPi( orient.at<int>( py, px ) ); 
    w_theta.push_back(exp(-pow( ( orient_p- angleToPi(orient.at<int>( ly, lx ))),2)/2/PI*5));
    w_theta.push_back(exp(-pow( ( orient_p- angleToPi(orient.at<int>( ly, rx ))),2)/2/PI*5));
    w_theta.push_back(exp(-pow( ( orient_p- angleToPi(orient.at<int>( ry, lx ))),2)/2/PI*5));
    w_theta.push_back(exp(-pow( ( orient_p- angleToPi(orient.at<int>( ry, rx ))),2)/2/PI*5));
    double tmp = 0;
    tmp += w_alpha[0] * w_theta[0] * ( normaled_mag.at<double>(ly, lx) + m.at<double>(ly,lx) );
    tmp += w_alpha[1] * w_theta[1] * ( normaled_mag.at<double>(ly, rx) + m.at<double>(ly,rx) );
    tmp += w_alpha[2] * w_theta[2] * ( normaled_mag.at<double>(ry, lx) + m.at<double>(ry,lx) );
    tmp += w_alpha[3] * w_theta[3] * ( normaled_mag.at<double>(ry, rx) + m.at<double>(ry,rx) );
    m.at<double>(py, px) = tmp;
}   
void GradientShop::localEdgeLength(){
    int iteration = ITERATION;
    Mat m0 = Mat::zeros( row_len, col_len, CV_64F); 
    Mat m1 = Mat::zeros( row_len, col_len, CV_64F); 
    while(iteration > 0){
        for(int i = 1; i < row_len -1 ; i++){
            for(int j = 1; j < col_len -1; j++){
                int angle = gradientOrientation( i, j);
                double pos_i = i + sqrt(2)*cos(angleToPi( angle ) ) ;
                double pos_j = i + sqrt(2)*sin(angleToPi(angle)) ;
                double neg_i = i + sqrt(2)*cos( PI +  angleToPi(angle) ) ;
                double neg_j = i + sqrt(2)*sin( PI + angleToPi(angle)) ;
                bilinearInterpolation( m0, i, j , pos_i, pos_j);
                bilinearInterpolation( m1, i, j , neg_i, neg_j);
            }
        }
        iteration --;

    }

    for(int j = 1; j < row_len -1; j++){
        for(int i = 1; i < col_len -1 ; i++){
            len_edge.at<double>(j, i) = m0.at<double>(j,i) + m1.at<double>(j, i) + normaled_mag.at<double>(j, i);
        }
    }
}
void GradientShop::localGradientSaliency(){
    for(int y = 1 ; y < row_len - 1; y ++ ){
        for (int x = 1; x < col_len - 1; x ++){
            int theta = orient.at<double>(y, x);
            int len = len_edge.at<double>(y, x);
            saliency_x.at<double>(y,x) =   pow( cos( angleToPi(theta) ) ,2) * len * u_x.at<double>(y,x);
            saliency_y.at<double>(y,x) = pow( sin( angleToPi(theta) ) ,2) * len * u_y.at<double>(y,x);
        }
    } 
}
void GradientShop::_computePixel(){
    for(int y = 0 ; y < row_len ; y ++ ){
        for (int x = 0; x < col_len ; x ++){
            u.at<double>(y,x)= getColor( x, y);
        }
    }

}

void GradientShop::_computeGradient(){
    for(int y = 0 ; y < row_len ; y ++ ){
        for (int x = 0; x < col_len ; x ++){
            u_x.at<double>(y,x)= gradient_x( x, y);
            u_y.at<double>(y,x)= gradient_y( x, y);
        }
    }

}
void GradientShop::_computeOrient(){
    for(int y = 1 ; y < row_len - 1; y ++ ){
        for (int x = 1; x < col_len - 1; x ++){
            orient.at<double>(y,x)= gradientOrientation( x, y);
        }
    }

}
void GradientShop::_computeNormalizedMagnitude(){
    for(int y = 1 ; y < row_len - 1; y ++ ){
        for (int x = 1; x < col_len - 1; x ++){
            normaled_mag.at<double>(y, x) = window_normalize( x, y, 3);
        }
    }
}
void GradientShop::_computeEdgeLength(){
    localEdgeLength();
}
void GradientShop::_computeSaliency(){
    localGradientSaliency();
}
GradientShop::GradientShop(){}
GradientShop::GradientShop(Mat input, int c ){
    src = input;
    channel = c;
    initialize();
}
void GradientShop::initialize(){
    /* 
     * Matrice: compute orient, normaled_mag, len_edge, saliency_x, saliency_y;
     * src:              Type: Mat(row, col, CV_8U_C3)
     *                   Description: RGB input image.
     * orient:           Type: Mat(row, col, CV_8U)
     *                   Description: Gradient orientation of input image.
     * normaled_mag:     Type: Mat(row, col, CV_64F)
     *                   Description: Normalized local maginitude inside window (eg. 3 * 3)
     * len_edge:         Type: Mat(row, col, CV_64F)
     *                   Description: Edge length in edge detection for each pixel, using message passing.
     * saliency_x, saliency_y:     Type: Mat(row, col, CV_64F)
     *                   Description: Saliency for x, y dimension 
     * u_x, u_y:         Type: Mat(row, col, CV_64F)
     *                   Description: gradient in x, y demension for each pixel
     */ 

    row_len = src.rows;
    col_len = src.cols;
    ITERATION = 60;
    CV_Assert(channel >= 0 && channel <= 2);
    nchannels = src.channels();
    orient = Mat::zeros( row_len, col_len, CV_8U); 
    normaled_mag = Mat::zeros( row_len, col_len, CV_64F); 
    len_edge = Mat::zeros( row_len, col_len, CV_64F);
    saliency_x = Mat::zeros( row_len, col_len, CV_64F);
    saliency_y = Mat::zeros( row_len, col_len, CV_64F);
    u_x = Mat::zeros( row_len, col_len, CV_64F);
    u_y = Mat::zeros( row_len, col_len, CV_64F);
    u = Mat::zeros( row_len, col_len, CV_64F);
}
void GradientShop::computeSaliency(){
    string window_name[] = {"pixel", "gradient_x","gradient_y", "orient", "normaled_mag", "edge", "saliency_x"};
    Mat dst;
    _computePixel();
    cout << "Pixel computed" <<endl;
    cout << u <<endl;
    u.convertTo(dst, CV_8U);
    cout<< dst<<endl;
    namedWindow(window_name[0], CV_WINDOW_AUTOSIZE);
    imshow(window_name[0], dst);
    _computeGradient();
    cout << "Gradient computed" <<endl;
    u_x.convertTo(dst, CV_8U);
    namedWindow(window_name[1], CV_WINDOW_AUTOSIZE);
    imshow(window_name[1], dst);
    u_y.convertTo(dst, CV_8U);
    namedWindow(window_name[2], CV_WINDOW_AUTOSIZE);
    imshow(window_name[2], dst);
    _computeOrient();
    cout << "Orient computed" <<endl;
    _computeNormalizedMagnitude();
    cout << "Normalized Magnitude computed" <<endl;
    _computeEdgeLength();
    cout << "Edge computed" <<endl;
    _computeSaliency();
    cout << "Saliency computed" <<endl;
}

