#include "visionVelodyneHDL64ECont.h"
#include <simLib.h>

CVisionVelodyneHDL64ECont::CVisionVelodyneHDL64ECont()
{
}

CVisionVelodyneHDL64ECont::~CVisionVelodyneHDL64ECont()
{
    removeAll();
}

int CVisionVelodyneHDL64ECont::addObject(CVisionVelodyneHDL64E* obj)
{
    _allObjects.push_back(obj);
    return(obj->getVelodyneHandle());
}

bool CVisionVelodyneHDL64ECont::removeObjectFromSensorHandle(int velodyneHandle)
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

bool CVisionVelodyneHDL64ECont::removeObjectFromScriptHandle(int h)
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

void CVisionVelodyneHDL64ECont::removeAll()
{
    for (size_t i=0;i<_allObjects.size();i++)
        delete _allObjects[i];
    _allObjects.clear();
}

CVisionVelodyneHDL64E* CVisionVelodyneHDL64ECont::getObject(int velodyneHandle)
{
    for (size_t i=0;i<_allObjects.size();i++)
    {
        if (_allObjects[i]->getVelodyneHandle()==velodyneHandle)
            return(_allObjects[i]);
    }
    return(0);
}
