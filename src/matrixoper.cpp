#include "matrixoper.h"
bool MatrixOper::matrixVectorProduct( SparseMatrix &A, vector<double> &x , vector<double> &prod){
    if(A.empty()) {
        cout <<  "Matrix empty." << endl;
        if ( x.empty() )
            return true;
        else
            return false;
    }
    int m = A.getRowLen(), n = A.getColLen();
    if( x.size() != n ) {
        cout << "Vector Demension not matched with Matrix." << endl;
        return false;
    }
    prod.resize(m);
    for(auto it = A.iterator(); it != A.end(); it++){
        pair<int, int> p = it->first;
        // prod[i] = a_i1 * x_1 + ... + a_in * x*n
        prod[p.first] += it->second * x[p.second];
    }
    return true;
}
void MatrixOper::matrixTranspose(SparseMatrix &A, SparseMatrix &At){
    if(A.empty()) return ;
    int m = A.getRowLen(), n = A.getColLen();
    At.setColLen(m);
    At.setRowLen(n);
    for(auto it = A.iterator(); it != A.end(); it++){
        pair<int, int> p = it->first;
        At.insert(make_pair(p.second, p.first), it->second);
    }
}

bool MatrixOper::vectorInnerProduct(vector<double> &x1, vector<double> &x2, double &prod){
    if(x1.size() != x2.size() ) { 
        cout << "Vector Demension not matched." << endl;
        return false;
    }
    prod = 0;
    for(int i = 0; i < x2.size(); i++ ){
        prod += x1[i] * x2[i];
    }
    return true;
}
bool MatrixOper::vectorAdd(vector<double> &x1, vector<double> &x2, vector<double> &res){
    if(x1.size() != x2.size() ) { 
        cout << "Vector Demension not matched." << endl;
        return false;
    }
    res.resize(x1.size());
    for(int i = 0; i < x1.size(); i++ ){
        res[i] = x1[i] + x2[i];
    }
    return true;
}
bool MatrixOper::vectorSelfAdd(vector<double> &x1, vector<double> &x2){
    if(x1.size() != x2.size() ) { 
        cout << "Vector Demension not matched." << endl;
        return false;
    }
    for(int i = 0; i < x1.size(); i++ ){
        x1[i] += x2[i];
    }
    return true;
}

bool MatrixOper::vectorSubtract(vector<double> &x1, vector<double> &x2, vector<double> &res){
    if(x1.size() != x2.size() ) { 
        cout << "Vector Demension not matched." << endl;
        return false;
    }
    res.resize(x1.size());
    for(int i = 0; i < x1.size(); i++ ){
        res[i] = x1[i] - x2[i];
    }
    return true;
}
bool MatrixOper::vectorSelfSubtract(vector<double> &x1, vector<double> &x2){
    if(x1.size() != x2.size() ) { 
        cout << "Vector Demension not matched." << endl;
        return false;
    }
    for(int i = 0; i < x1.size(); i++ ){
        x1[i] -= x2[i];
    }
    return true;
}
void MatrixOper::vectorScalarProd(vector<double> &x1, vector<double> & res, double s){
    for (int i = 0; i < x1.size(); i++){
        res[i] = x1[i]*s;
    }
}


