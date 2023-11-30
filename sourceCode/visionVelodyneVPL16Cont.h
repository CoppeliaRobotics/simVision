#pragma once

#include "visionVelodyneVPL16.h"
#include <vector>

class CVisionVelodyneVPL16Cont  
{
public:
    CVisionVelodyneVPL16Cont();
    virtual ~CVisionVelodyneVPL16Cont();

    int addObject(CVisionVelodyneVPL16* obj);
    CVisionVelodyneVPL16* getObject(int velodyneHandle);
    bool removeObjectFromSensorHandle(int velodyneHandle);
    bool removeObjectFromScriptHandle(int h);
    void removeAll();

private:
    std::vector<CVisionVelodyneVPL16*> _allObjects;
};
