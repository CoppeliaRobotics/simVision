#pragma once

#include <vector>

class CVisionRemap
{
public:
    CVisionRemap(int scriptHandle,int sensorHandle,const int* map,const float* scalings);
    virtual ~CVisionRemap();

    bool isSame(int scriptHandle,const int* map,const float* scalings) const;
    int getSensorHandle() const;
    int getRelatedScriptHandle() const;
    void handleObject();

private:
    int _sensorHandle;
    int _scriptHandle;
    int _pixelCount;

    float* _sensorImage;
    int* _mapP;
    float* _mapI;
};
