#pragma once

#include "visionRemap.h"
#include <vector>

class CVisionRemapCont
{
public:
    CVisionRemapCont();
    virtual ~CVisionRemapCont();

    int addObject(CVisionRemap* obj);
    bool removeObjectFromSensorHandle(int h);
    bool removeObjectFromScriptHandle(int h);
    void removeAll();
    CVisionRemap* getObjectFromSensorHandle(int h);

private:
    std::vector<CVisionRemap*> _allObjects;
};
