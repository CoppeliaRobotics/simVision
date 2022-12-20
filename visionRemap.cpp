#include "visionRemap.h"
#include "vis.h"
#include <math.h>

#define PI_VAL (3.14159265f)

CVisionRemap::CVisionRemap(int scriptHandle,int sensorHandle,const int* map,const double* scalings)
{
    _scriptHandle=scriptHandle;
    _sensorHandle=sensorHandle;
    int r[2];
    simGetVisionSensorRes(_sensorHandle,r);
    _pixelCount=r[0]*r[1];
    _sensorImage=new double[_pixelCount*3];
    _mapP=new int[_pixelCount];
    for (int i=0;i<_pixelCount;i++)
        _mapP[i]=map[i];
    _mapI=nullptr;
    if (scalings!=nullptr)
    {
        _mapI=new double[_pixelCount];
        for (int i=0;i<_pixelCount;i++)
            _mapI[i]=scalings[i];
    }
}

CVisionRemap::~CVisionRemap()
{
    delete[] _sensorImage;
    delete[] _mapP;
    delete[] _mapI;
}

int CVisionRemap::getRelatedScriptHandle() const
{
    return(_scriptHandle);
}

bool CVisionRemap::isSame(int scriptHandle,const int* map,const double* scalings) const
{
    if (scriptHandle!=_scriptHandle)
        return(false);
    int r[2]={0,0};
    simGetVisionSensorRes(_sensorHandle,r);
    if (r[0]*r[1]!=_pixelCount)
        return(false);
    for (int i=0;i<_pixelCount;i++)
    {
        if (map[i]!=_mapP[i])
            return(false);
    }
    if (scalings==nullptr)
    {
        if (_mapI!=nullptr)
            return(false);
    }
    else
    {
        if (_mapI==nullptr)
            return(false);
        for (int i=0;i<_pixelCount;i++)
        {
            if (scalings[i]!=_mapI[i])
                return(false);
        }
    }
    return(true);
}

int CVisionRemap::getSensorHandle() const
{
    return(_sensorHandle);
}

void CVisionRemap::handleObject()
{
    double* img=getVisionSensorImage(_sensorHandle);
    double* depth=nullptr;
    double np,fp,fmn;
    if (_mapI!=nullptr)
    {
        simGetObjectFloatParam(_sensorHandle,sim_visionfloatparam_near_clipping,&np);
        simGetObjectFloatParam(_sensorHandle,sim_visionfloatparam_far_clipping,&fp);
        depth=getVisionSensorDepth(_sensorHandle);
        fmn=fp-np;
    }
    int c,p;
    for (int i=0;i<_pixelCount;i++)
    {
        c=3*i;
        p=3*_mapP[i];
        _sensorImage[c+0]=img[p+0];
        _sensorImage[c+1]=img[p+1];
        _sensorImage[c+2]=img[p+2];
    }
    setVisionSensorImage(_sensorHandle,_sensorImage);
    simReleaseBuffer((char*)img);
    if (_mapI!=nullptr)
    {
        for (int i=0;i<_pixelCount;i++)
        {
            p=_mapP[i];
            double d=(np+fmn*depth[p])*_mapI[i];
            d=(d-np)/fmn;
            if (d>1.0f)
                d=1.0f;
            _sensorImage[i]=d;
        }
        setVisionSensorDepth(_sensorHandle,_sensorImage);
        simReleaseBuffer((char*)depth);
    }
}
