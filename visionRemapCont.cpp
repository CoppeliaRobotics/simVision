#include "visionRemapCont.h"
#include <simLib/simLib.h>

CVisionRemapCont::CVisionRemapCont()
{
}

CVisionRemapCont::~CVisionRemapCont()
{
    removeAll();
}

int CVisionRemapCont::addObject(CVisionRemap* obj)
{
    _allObjects.push_back(obj);
    return(obj->getSensorHandle());
}

bool CVisionRemapCont::removeObjectFromSensorHandle(int h)
{
    for (size_t i=0;i<_allObjects.size();i++)
    {
        if (_allObjects[i]->getSensorHandle()==h)
        {
            delete _allObjects[i];
            _allObjects.erase(_allObjects.begin()+i);
            return(true);
        }
    }
    return(false);
}

bool CVisionRemapCont::removeObjectFromScriptHandle(int h)
{
    for (size_t i=0;i<_allObjects.size();i++)
    {
        if (_allObjects[i]->getRelatedScriptHandle()==h)
        {
            delete _allObjects[i];
            _allObjects.erase(_allObjects.begin()+i);
            return(true);
        }
    }
    return(false);
}

void CVisionRemapCont::removeAll()
{
    for (size_t i=0;i<_allObjects.size();i++)
        delete _allObjects[i];
    _allObjects.clear();
}

CVisionRemap* CVisionRemapCont::getObjectFromSensorHandle(int h)
{
    for (size_t i=0;i<_allObjects.size();i++)
    {
        if (_allObjects[i]->getSensorHandle()==h)
            return(_allObjects[i]);
    }
    return(0);
}
