#pragma once

#include "simLib.h"
#include <vector>

class CVisionSensorData
{
    public:
    CVisionSensorData()
    {
        _sensorHandle=-1;
        _scriptHandle=-1;
        resolution[0]=0;
        resolution[1]=0;
        workImg=nullptr;
        buff1Img=nullptr;
        buff2Img=nullptr;
    }
    ~CVisionSensorData()
    {
        if (workImg!=nullptr)
            delete[] workImg;
        if (buff1Img!=nullptr)
            delete[] buff1Img;
        if (buff2Img!=nullptr)
            delete[] buff2Img;
    }
    int getSensorHandle()
    {
        return(_sensorHandle);
    }
    int getScriptHandle()
    {
        return(_scriptHandle);
    }
    void setHandles(int sensorHandle,int scriptHandle)
    {
        _sensorHandle=sensorHandle;
        _scriptHandle=scriptHandle;
        simGetVisionSensorResolution(_sensorHandle,resolution);
    }

    int resolution[2];
    float* workImg;
    float* buff1Img;
    float* buff2Img;

private:
    int _sensorHandle;
    int _scriptHandle;
};

class CVisionCont
{
public:
    CVisionCont();
    virtual ~CVisionCont();

    CVisionSensorData* getImageObject(int sensorHandle);
    bool removeImageObject(int sensorHandle);
    bool removeImageObjectFromScriptHandle(int scriptHandle);
    void setImageObject(int sensorHandle,int scriptHandle,CVisionSensorData* object);
    void removeAll();

private:
    std::vector<CVisionSensorData*> _allObjects;
};
