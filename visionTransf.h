#pragma once

#include <vector>

class CVisionTransf  
{
public:
    CVisionTransf(int scriptHandle,int passiveVisionSensorHandle,const int activeVisionSensorHandles[6],float horizontalAngle,float verticalAngle,int passiveVisionSensorHandleForDepth);
    virtual ~CVisionTransf();

    bool isActiveVisionSensorResolutionCorrect();
    bool areRGBAndDepthVisionSensorResolutionsCorrect() const;
    bool areActiveVisionSensorsExplicitelyHandled() const;
    bool doAllObjectsExistAndAreVisionSensors() const;
    bool isSame(int scriptHandle,const int activeVisionSensorHandles[6],float horizontalAngle,float verticalAngle,int passive1,int passive2) const;
    int getReferencePassiveVisionSensorHandle() const;
    int getRelatedScriptHandle() const;

    void disableSpecularLightComponent(bool d);
    void handleObject();
    void releaseActiveVisionSensorImages();

private:
    void _calculateMapping();

    int _referencePassiveVisionSensorHandle;
    int _passiveVisionSensorHandle;
    int _passiveVisionSensorHandleForDepth;

    int _scriptHandle;
    int _activeVisionSensorHandles[6];
    bool _usedActiveVisionSensors[6];
    float _horizontalAngle;
    float _verticalAngle;
    int _passiveVisionSensorResolution[2];
    int _activeVisionSensorResolutionXY;

    float* _activeVisionSensorImages[6];

    float* _passiveVisionSensorImage;

    int* _mapP;
    float* _mapI;
    unsigned char* _mapV;
};
