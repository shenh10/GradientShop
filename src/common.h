#ifndef __COMMON_H__
#define __COMMON_H__

#include "iostream"
#include <math.h> 
#include <string>
#include <vector>
#include <utility>
#include <map>
#define PI 3.14159265

using namespace std;
template <typename T>
void printVec(vector<T> vec){
    cout<<"[";
    for (int i = 0; i< vec.size(); i++){
        cout<< vec[i]<<" ";

    }

    cout<<"]"<<endl;
}

#endif
