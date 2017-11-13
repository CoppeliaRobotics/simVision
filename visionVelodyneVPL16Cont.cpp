#include "visionVelodyneVPL16Cont.h"
#include "v_repLib.h"

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

bool CVisionVelodyneVPL16Cont::removeObject(int velodyneHandle)
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

void CVisionVelodyneVPL16Cont::removeAll()
{
    for (int i=0;i<int(_allObjects.size());i++)
        delete _allObjects[i];
    _allObjects.clear();
}

CVisionVelodyneVPL16* CVisionVelodyneVPL16Cont::getObject(int velodyneHandle)
{
    for (int i=0;i<int(_allObjects.size());i++)
    {
        if (_allObjects[i]->getVelodyneHandle()==velodyneHandle)
            return(_allObjects[i]);
    }
    return(0);
}
