#pragma once

#include <vector>

class CVisionVelodyneVPL16  
{
public:
    CVisionVelodyneVPL16(const int visionSensorHandles[4],float frequency,int options,float pointSize,float _coloringDistances[2],float scalingFactor,int newPointCloudHandle);
    virtual ~CVisionVelodyneVPL16();

    int getVelodyneHandle();
    bool areVisionSensorsExplicitelyHandled();
    bool doAllObjectsExistAndAreVisionSensors();
    bool handle(float dt,std::vector<float>& pts);

private:
    void _removePointsBetween(float lowAngle,float range);
    void _getColorFromIntensity(float intensity,unsigned char col[3]);

    int _visionSensorHandles[4];
    float _frequency;
    int _velodyneHandle;
    float _displayScalingFactor;
    float _pointSize;
    bool _displayPts;
    bool _emissivePts;
    bool _displayOnlyCurrent;
    bool _cartesianCoords;
    int _ptCloudHandle;
    int _newPtCloudHandle;
    float lastScanAngle;
    float _coloringDistances[2];
    std::vector<float> _displayPtsXyz;
    std::vector<float> _displayPtsA;
    std::vector<unsigned char> _displayPtsCol;

    static int _nextVelodyneHandle;
};
