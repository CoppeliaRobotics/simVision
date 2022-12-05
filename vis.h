#pragma once

#include "simLib.h"
#include <vector>

static double* getVisionSensorImage(int h)
{
    int res[2];
    unsigned char* img=simGetVisionSensorImg(h,0,0.0,nullptr,nullptr,res);
    double* image=(double*)simCreateBuffer(res[0]*res[1]*3*sizeof(double));
    for (int i=0;i<res[0]*res[1]*3;i++)
        image[i]=double(img[i])/255.0;
    simReleaseBuffer(img);
    return(image);
}

static void setVisionSensorImage(int h,const double* image,bool greyscale=false)
{
    int res[2];
    simGetVisionSensorRes(h,res);
    std::vector<unsigned char> img;
    img.resize(res[0]*res[1]*3);
    for (int i=0;i<res[0]*res[1]*3;i++)
        img[i]=(unsigned char)(image[i]*255.1);
    int opt=0;
    if (greyscale)
        opt=1;
    simSetVisionSensorImg(h,img.data(),opt,nullptr,nullptr);
}

static double* getVisionSensorDepth(int h)
{
    int res[2];
    float* d=simGetVisionSensorDepth(h,0,nullptr,nullptr,res);
    double* depth=(double*)simCreateBuffer(res[0]*res[1]*sizeof(double));
    for (int i=0;i<res[0]*res[1];i++)
        depth[i]=double(d[i]);
    simReleaseBuffer(d);
    return(depth);
}

static void setVisionSensorDepth(int h,const double* depth)
{
    int res[2];
    simGetVisionSensorRes(h,res);
    std::vector<float> d;
    d.resize(res[0]*res[1]);
    for (int i=0;i<res[0]*res[1];i++)
        d[i]=(float)depth[i];
    _simSetVisionSensorDepth(h,0,d.data());
}

