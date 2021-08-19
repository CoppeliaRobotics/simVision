#pragma once

#include "visionTransf.h"
#include <vector>

class CVisionTransfCont  
{
public:
    CVisionTransfCont();
    virtual ~CVisionTransfCont();

    int addObject(CVisionTransf* obj);
    bool removeObjectFromPassiveSensorHandle(int passiveVisionSensorHandle);
    bool removeObjectFromScriptHandle(int h);
    void removeInvalidObjects();
    void removeAll();
    CVisionTransf* getVisionTransfFromReferencePassiveVisionSensor(int passiveVisionSensorHandle);

private:
    std::vector<CVisionTransf*> _allObjects;
};
