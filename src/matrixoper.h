#ifndef __MATRIXOPER_H__
#define __MATRIXOPER_H__

#ifndef __COMMON_H__
# include "common.h"
#endif
#include "sparsematrix.h"
class MatrixOper{
public:
    bool matrixVectorProduct( SparseMatrix &A, vector<double> &x , vector<double> &prod);
    void matrixTranspose( SparseMatrix &A, SparseMatrix &At);

    bool vectorInnerProduct(vector<double> &x1, vector<double> &x2, double &prod);
    bool vectorAdd(vector<double> &x1, vector<double> &x2, vector<double> &res);
    bool vectorSelfAdd(vector<double> &x1, vector<double> &x2);

    bool vectorSubtract(vector<double> &x1, vector<double> &x2, vector<double> &res);
    bool vectorSelfSubtract(vector<double> &x1, vector<double> &x2);
    void vectorScalarProd(vector<double> &x1, vector<double> & res, double s);
};
#endif
