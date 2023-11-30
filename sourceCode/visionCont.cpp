#include "visionCont.h"
#include "vis.h"

CVisionCont::CVisionCont()
{
}

CVisionCont::~CVisionCont()
{
    removeAll();
}

CVisionSensorData* CVisionCont::getImageObject(int sensorHandle)
{
    for (size_t i=0;i<_allObjects.size();i++)
    {
        if (_allObjects[i]->getSensorHandle()==sensorHandle)
            return(_allObjects[i]);
    }
    return(nullptr);
}

bool CVisionCont::removeImageObject(int sensorHandle)
{
    for (size_t i=0;i<_allObjects.size();i++)
    {
        if (_allObjects[i]->getSensorHandle()==sensorHandle)
        {
            delete _allObjects[i];
            _allObjects.erase(_allObjects.begin()+i);
            return(true);
        }
    }
    return(false);
}

bool CVisionCont::removeImageObjectFromScriptHandle(int scriptHandle)
{
    for (size_t i=0;i<_allObjects.size();i++)
    {
        if (_allObjects[i]->getScriptHandle()==scriptHandle)
        {
            delete _allObjects[i];
            _allObjects.erase(_allObjects.begin()+i);
            return(true);
        }
    }
    return(false);
}

void CVisionCont::setImageObject(int sensorHandle,int scriptHandle,CVisionSensorData* object)
{
    removeImageObject(sensorHandle);
    object->setHandles(sensorHandle,scriptHandle);
    _allObjects.push_back(object);
}

void CVisionCont::removeAll()
{
    for (size_t i=0;i<_allObjects.size();i++)
        delete _allObjects[i];
    _allObjects.clear();
}
