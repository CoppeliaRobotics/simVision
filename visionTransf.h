#pragma once

#include <vector>

class CVisionTransf  
{
public:
    CVisionTransf(int passiveVisionSensorHandle,const int activeVisionSensorHandles[6],float horizontalAngle,float verticalAngle,int passiveVisionSensorHandleForDepth);
    virtual ~CVisionTransf();

    bool isActiveVisionSensorResolutionCorrect();
    bool areRGBAndDepthVisionSensorResolutionsCorrect();
    bool areActiveVisionSensorsExplicitelyHandled();
    bool doAllObjectsExistAndAreVisionSensors();

    bool isSame(const int activeVisionSensorHandles[6],float horizontalAngle,float verticalAngle,int passive1,int passive2);

    int getReferencePassiveVisionSensorHandle();

    void disableSpecularLightComponent(bool d);

    void handleObject();
    void releaseActiveVisionSensorImages();

private:
    void _calculateMapping();

    int _referencePassiveVisionSensorHandle;
    int _passiveVisionSensorHandle;
    int _passiveVisionSensorHandleForDepth;

    int _activeVisionSensorHandles[6];
    bool _usedActiveVisionSensors[6];
    float _horizontalAngle;
    float _verticalAngle;
    int _passiveVisionSensorResolution[2];
    int _activeVisionSensorResolutionXY;

    float* _activeVisionSensorImages[6];

    float* _passiveVisionSensorImage;

    int* _mapP;
    unsigned char* _mapV;
};
