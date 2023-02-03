#include "visionVelodyneVPL16Cont.h"
#include <simLib.h>

CVisionVelodyneVPL16Cont::CVisionVelodyneVPL16Cont()
{
}

CVisionVelodyneVPL16Cont::~CVisionVelodyneVPL16Cont()
{
    removeAll();
}

int CVisionVelodyneVPL16Cont::addObject(CVisionVelodyneVPL16* obj)
{
    _allObjects.push_back(obj);
    return(obj->getVelodyneHandle());
}

bool CVisionVelodyneVPL16Cont::removeObjectFromSensorHandle(int velodyneHandle)
{
    for (size_t i=0;i<_allObjects.size();i++)
    {
        if (_allObjects[i]->getVelodyneHandle()==velodyneHandle)
        {
            delete _allObjects[i];
            _allObjects.erase(_allObjects.begin()+i);
            return(true);
        }
    }
    return(false);
}

bool CVisionVelodyneVPL16Cont::removeObjectFromScriptHandle(int h)
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

void CVisionVelodyneVPL16Cont::removeAll()
{
    for (size_t i=0;i<_allObjects.size();i++)
        delete _allObjects[i];
    _allObjects.clear();
}

CVisionVelodyneVPL16* CVisionVelodyneVPL16Cont::getObject(int velodyneHandle)
{
    for (size_t i=0;i<_allObjects.size();i++)
    {
        if (_allObjects[i]->getVelodyneHandle()==velodyneHandle)
            return(_allObjects[i]);
    }
    return(0);
}
