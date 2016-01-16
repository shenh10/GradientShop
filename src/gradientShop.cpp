#include "gradientshop.h"
bool GradientShop::isInRange(double x, double y, double width){
    if (x >=  width && x < col_len - width && y >= width && y < row_len - width)
        return true;
    else
        return false;
}
double GradientShop::getColor(int x, int y){
    return retrieveColor(src, channel, x, y);
}
double GradientShop::retrieveColor(Mat &_src, int _channel, int x, int y){
    if (_src.channels() == 1){
        return double(_src.at<double>(y, x));
    }
    Vec3b intensity = _src.at<Vec3b>(y, x);
    return double(intensity.val[_channel]);
}
double GradientShop::gradient_x(int x, int y){
    if( ! isInRange(x, y, 1) ) return 0;
    double dx = getColor( x + 1, y) - getColor( x - 1, y);
    return double(dx);
}   
double GradientShop::gradient_y(int x, int y){
    if( ! isInRange(x, y, 1) ) return 0;
    double dy = getColor( x, y - 1 ) - getColor( x , y + 1);
    return double(dy);
}   
double GradientShop::gradientMagnitude(int px, int py){
    return sqrt(pow(px,2) + pow(py, 2));
}
int GradientShop::gradientOrientation( int x, int y){
    double dx = gradient_x( x, y), dy = gradient_y( x, y);
    if(dx == 0){
        if(dy > 0)  return 90;
        else if(dy < 0) return 270;
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
        Mat window = Mat::zeros(width, width, CV_64F);
        for (int i = - width/2; i < width/2 + 1 ; i++)
            for (int j = - width/2; j < width/2 + 1 ; j++){
                window.at<double>(offset + j, offset + i) = u_mag.at<double>( y + j, x + i);
                avg  += window.at<double>(offset + j , offset + i);
            }
        avg /= ( width * width );
        for (int i = - width/2; i < width/2 + 1 ; i++)
            for (int j = - width/2; j < width/2 + 1 ; j++)
                sigma  += pow( double( window.at<double>(offset + j, offset + i) - avg ) ,2);
        sigma = sqrt(sigma/(width * width));
        //cout << window << endl;
        //cout << "Avg:" << avg << ", Sigma: "<< sigma<< endl;
        return (window.at<double>(offset , offset) - avg)/(sigma + epsilon);
    }
    return 0;

}
int GradientShop::angleToPi(int angle){
    return angle * PI / 180.0;
}
void GradientShop::bilinearInterpolation( Mat &m, int px, int py, double x, double y){
    int lx = floor(x),rx = ceil(x), ly = floor(y), ry = ceil(y);
    if(! isInRange(lx, ly, 0) || ! isInRange(rx, ry, 0)) return;
    vector<double>  w_alpha, w_theta;
    w_alpha.resize(4);
    w_theta.resize(4);
    w_alpha.push_back(1.0*(rx - x)*(ry - y)/((rx - lx)*(ry - ly)));
    w_alpha.push_back(1.0*(x - lx)*(ry - y)/((rx - lx)*(ry - ly)));
    w_alpha.push_back(1.0*(rx - x)*(y - ly)/((rx - lx)*(ry - ly)));
    w_alpha.push_back(1.0*(x - lx)*(y - ly)/((rx - lx)*(ry - ly)));
    double orient_p = angleToPi( orient.at<double>( py, px ) ); 
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
        for(int j = 1; j < row_len -1; j++){
            for(int i = 1; i < col_len -1 ; i++){
                int angle = gradientOrientation( i, j);
                double pos_i = i + sqrt(2)*cos(angleToPi( angle ) ) ;
                double pos_j = j + sqrt(2)*sin(angleToPi(angle)) ;
                double neg_i = i + sqrt(2)*cos( PI +  angleToPi(angle) ) ;
                double neg_j = j + sqrt(2)*sin( PI + angleToPi(angle)) ;
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
            double len = len_edge.at<double>(y, x);
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

void GradientShop::_computeGradientMagnitude(){
    for(int y = 0 ; y < row_len ; y ++ ){
        for (int x = 0; x < col_len ; x ++){
            u_mag.at<double>(y,x)= gradientMagnitude(u_x.at<double>(y,x), u_y.at<double>(y,x));
        }
    }
}
void GradientShop::_computeOrient(){
    for(int y = 1 ; y < row_len - 1; y ++ ){
        for (int x = 1; x < col_len - 1; x ++){
            int c =  gradientOrientation( x, y);

            orient.at<double>(y,x) = double(c);
        }
    }

}
void GradientShop::_computeNormalizedMagnitude(){
    for(int y = 1 ; y < row_len - 1; y ++ ){
        for (int x = 1; x < col_len - 1; x ++){
            normaled_mag.at<double>(y, x) = window_normalize( x, y, 5);
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
GradientShop::GradientShop(Mat input, int c , string outdir){
    src = input;
    outdir = outdir;
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
    u_mag = Mat::zeros( row_len, col_len, CV_64F);
}
void GradientShop::showImage(Mat src, string window_name){
    Mat dst;
    cout << window_name << " computed" <<endl;
    src.convertTo(dst, CV_8U);
    namedWindow(window_name, CV_WINDOW_AUTOSIZE);
    imshow(window_name, dst);
}
void GradientShop::writeImage(Mat src, string window_name){
    cout << window_name << " computed" <<endl;
    vector<int> compression_params; 
    compression_params.push_back(CV_IMWRITE_JPEG_QUALITY); 
    compression_params.push_back(100);

    Mat dst;
    src.convertTo(dst, CV_8U);
    cout << dst << endl;
    bool bSuccess = imwrite( outdir + "/"+ window_name + ".jpg", dst, compression_params); 
    if ( !bSuccess )
    {
        cout << "ERROR : Failed to save the image" << endl;
        //system("pause"); //wait for a key press
    }
}

void GradientShop::computeSaliency(){
    string window_name[] = {"pixel", "gradient_x","gradient_y","gradientMagnitude",  "normaled_mag",  "saliency"};
    _computePixel();
    writeImage(u, window_name[0]);
    _computeGradient();
    writeImage(abs(u_x), window_name[1]);
    writeImage(abs(u_y), window_name[2]);
    _computeGradientMagnitude();
    writeImage(u_mag, window_name[3]);
    _computeOrient();
    writeImage(orient, "orient");
    _computeNormalizedMagnitude();
    cout << normaled_mag << endl;
    double min, max;
    minMaxLoc(normaled_mag, &min, &max);
    cout << "min:"<< min << ", max:" << max <<endl;
    writeImage(((normaled_mag-min)/(max-min)*255), window_name[4]);
    _computeEdgeLength();
    _computeSaliency();
    Mat dst = Mat::zeros(row_len, col_len, CV_64F);
    for(int y = 1 ; y < row_len - 1; y ++ ){
        for (int x = 1; x < col_len - 1; x ++){
            dst.at<double>(y,x) = gradientMagnitude(saliency_x.at<double>(y,x), saliency_y.at<double>(y,x));
        }
    }
    writeImage(dst, window_name[5]);
}

