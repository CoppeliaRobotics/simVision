#pragma once

#include "visionVelodyneHDL64E.h"
#include <vector>

class CVisionVelodyneHDL64ECont  
{
public:
    CVisionVelodyneHDL64ECont();
    virtual ~CVisionVelodyneHDL64ECont();

    int addObject(CVisionVelodyneHDL64E* obj);
    CVisionVelodyneHDL64E* getObject(int velodyneHandle);
    bool removeObjectFromSensorHandle(int velodyneHandle);
    bool removeObjectFromScriptHandle(int h);
    void removeAll();

private:
    std::vector<CVisionVelodyneHDL64E*> _allObjects;
};
