#include "visionVelodyneHDL64ECont.h"
#include "v_repLib.h"

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

bool CVisionVelodyneHDL64ECont::removeObject(int velodyneHandle)
{
    for (int i=0;i<int(_allObjects.size());i++)
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

void CVisionVelodyneHDL64ECont::removeAll()
{
    for (int i=0;i<int(_allObjects.size());i++)
        delete _allObjects[i];
    _allObjects.clear();
}

CVisionVelodyneHDL64E* CVisionVelodyneHDL64ECont::getObject(int velodyneHandle)
{
    for (int i=0;i<int(_allObjects.size());i++)
    {
        if (_allObjects[i]->getVelodyneHandle()==velodyneHandle)
            return(_allObjects[i]);
    }
    return(0);
}
