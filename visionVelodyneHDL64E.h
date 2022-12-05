#pragma once

#include <vector>

class CVisionVelodyneHDL64E  
{
public:
    CVisionVelodyneHDL64E(int scriptHandle,const int visionSensorHandles[4],double frequency,int options,double pointSize,double _coloringDistances[2],double scalingFactor,int newPointCloudHandle);
    virtual ~CVisionVelodyneHDL64E();

    int getVelodyneHandle() const;
    int getRelatedScriptHandle() const;
    bool areVisionSensorsExplicitelyHandled() const;
    bool doAllObjectsExistAndAreVisionSensors() const;
    bool handle(double dt,std::vector<double>& pts,bool getAbsPts,std::vector<unsigned char>& retCols);

private:
    void _removePointsBetween(double lowAngle,double range);
    void _getColorFromIntensity(double intensity,unsigned char col[3]);

    int _scriptHandle;
    int _visionSensorHandles[4];
    double _frequency;
    int _velodyneHandle;
    double _displayScalingFactor;
    double _pointSize;
    bool _displayPts;
    bool _emissivePts;
    bool _displayOnlyCurrent;
    bool _cartesianCoords;
    int _ptCloudHandle;
    int _newPtCloudHandle;
    double lastScanAngle;
    double _coloringDistances[2];
    std::vector<double> _displayPtsXyz;
    std::vector<double> _displayPtsA;
    std::vector<unsigned char> _displayPtsCol;

    static int _nextVelodyneHandle;
};
