#ifndef __COMMON_H__
#define __COMMON_H__

#include "iostream"
#include <math.h> 
#include <string>
#include <vector>
#include <utility>
#include <map>
#include <stdlib.h>
#define PI 3.14159265

using namespace std;
template <typename T>
void printVec(vector<T> vec, int number){
    cout<<"[";
    for (int i = 0; i< ( vec.size()< number? vec.size():number); i++){
        cout<< vec[i]<<" ";

    }

    cout<<"]"<<endl;
}

#endif
