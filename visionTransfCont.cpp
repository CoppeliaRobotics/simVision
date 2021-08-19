#include "visionTransfCont.h"
#include "simLib.h"

CVisionTransfCont::CVisionTransfCont()
{
}

CVisionTransfCont::~CVisionTransfCont()
{
    removeAll();
}

int CVisionTransfCont::addObject(CVisionTransf* obj)
{
    _allObjects.push_back(obj);
    return(obj->getReferencePassiveVisionSensorHandle());
}

bool CVisionTransfCont::removeObjectFromPassiveSensorHandle(int passiveVisionSensorHandle)
{
    for (size_t i=0;i<_allObjects.size();i++)
    {
        if (_allObjects[i]->getReferencePassiveVisionSensorHandle()==passiveVisionSensorHandle)
        {
            delete _allObjects[i];
            _allObjects.erase(_allObjects.begin()+i);
            return(true);
        }
    }
    return(false);
}

bool CVisionTransfCont::removeObjectFromScriptHandle(int h)
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

void CVisionTransfCont::removeAll()
{
    for (size_t i=0;i<_allObjects.size();i++)
        delete _allObjects[i];
    _allObjects.clear();
}

void CVisionTransfCont::removeInvalidObjects()
{
    for (size_t i=0;i<_allObjects.size();i++)
    {
        if (!_allObjects[i]->doAllObjectsExistAndAreVisionSensors())
        {
            delete _allObjects[i];
            _allObjects.erase(_allObjects.begin()+i);
            i--; // reprocess this position
        }
    }
}

CVisionTransf* CVisionTransfCont::getVisionTransfFromReferencePassiveVisionSensor(int passiveVisionSensorHandle)
{
    for (size_t i=0;i<_allObjects.size();i++)
    {
        if (_allObjects[i]->getReferencePassiveVisionSensorHandle()==passiveVisionSensorHandle)
            return(_allObjects[i]);
    }
    return(0);
}
