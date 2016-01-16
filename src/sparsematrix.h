#ifndef __SPARSEMATRIX_H_H
#define __SPARSEMATRIX_H_H
#include "common.h"

class SparseMatrix{
    private:
        map<pair<int, int>, double> kvMap;
        int rowLen, colLen;
    public:
        SparseMatrix(){
            rowLen = 0;
            colLen = 0;
        }
//        SparseMatrix(SparseMatrix &a){
//            kvMap = a.kvMap;
//            rowLen = a.rowLen;
//            colLen = a.colLen;
//        }
        SparseMatrix(int r, int c){
            rowLen = r;
            colLen = c;
        }
        int getRowLen(){
            return rowLen;
        }
        int getColLen(){
            return colLen;
        }
        void setRowLen(int row){
            rowLen = row;
        }
        void setColLen(int col){
            colLen = col;
        }
        bool inRange(pair<int,int> key){
            if( key.first < rowLen && key.first >= 0 && key.second >=0 && key.second < colLen)
                return true;
            return false;
        }
        void insert(pair<int, int> key, double val){
            if(! inRange(key)) return;
            auto a = kvMap.find(key);
            if( a != kvMap.end() ){
                a->second = a->second + val;
                return;
            }
            kvMap[key] = val;
        } 
        double get(pair<int, int> key){
            if(! inRange(key)) return 0;
            auto a = kvMap.find(key);
            if( a != kvMap.end() ){
                return a->second;
            }
            return 0;
        }
        bool empty(){
            if(kvMap.empty())
                return true;
            return false;
        }
        map<pair<int, int>, double>::iterator  begin(){
            return kvMap.begin();
        }
         map<pair<int, int>, double>::iterator end(){
            return kvMap.end();
        }
        map<pair<int, int>, double>::iterator iterator(){
            auto it = kvMap.begin();
            return it;
        }
        void printMatrix(){
           for(auto it = kvMap.begin(); it != kvMap.end(); it++){
                auto p = it->first;
                cout << "("<< p.first << ", " << p.second <<")" << ":" << it->second << endl;
            } 
        }

};
#endif
