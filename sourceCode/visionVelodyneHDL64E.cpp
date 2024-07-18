#include "visionVelodyneHDL64E.h"
#include <simLib/simLib.h>
#include <math.h>

#define PI_VAL (3.14159265f)

int CVisionVelodyneHDL64E::_nextVelodyneHandle=0;

CVisionVelodyneHDL64E::CVisionVelodyneHDL64E(int scriptHandle,const int visionSensorHandles[4],double frequency,int options,double pointSize,double coloringDistances[2],double scalingFactor,int newPointCloudHandle)
{
    _scriptHandle=scriptHandle;
    for (int i=0;i<4;i++)
        _visionSensorHandles[i]=visionSensorHandles[i];
    _frequency=frequency;
    _displayScalingFactor=1.0f;
    _displayPts=(options&1)==0;
    _displayOnlyCurrent=(options&2)!=0;
    _cartesianCoords=(options&4)==0;
    _emissivePts=(options&8)!=0;
    _pointSize=pointSize;
    _displayScalingFactor=scalingFactor;
    _ptCloudHandle=-1;
    _newPtCloudHandle=newPointCloudHandle;
    _coloringDistances[0]=coloringDistances[0];
    _coloringDistances[1]=coloringDistances[1];
    lastScanAngle=0.0f;
    _velodyneHandle=_nextVelodyneHandle++;
}

CVisionVelodyneHDL64E::~CVisionVelodyneHDL64E()
{
    if (_ptCloudHandle>=0)
        simModifyPointCloud(_ptCloudHandle,0,0,0);
    if (_newPtCloudHandle>=0)
        simRemovePointsFromPointCloud(_newPtCloudHandle,0,0,0,0.0,0);
}

int CVisionVelodyneHDL64E::getVelodyneHandle() const
{
    return(_velodyneHandle);
}

int CVisionVelodyneHDL64E::getRelatedScriptHandle() const
{
    return(_scriptHandle);
}

bool CVisionVelodyneHDL64E::areVisionSensorsExplicitelyHandled() const
{
    for (int i=0;i<4;i++)
    {
        int r=simGetExplicitHandling(_visionSensorHandles[i]);
        if (r==-1)
            return(false);
        if ((r&1)==0)
            return(false);
    }
    return(true);
}

bool CVisionVelodyneHDL64E::doAllObjectsExistAndAreVisionSensors() const
{
    for (int i=0;i<4;i++)
    {
        if (simGetObjectType(_visionSensorHandles[i])!=sim_sceneobject_visionsensor)
            return(false);
    }
    if (_newPtCloudHandle!=-1)
    {
        if (simGetObjectType(_newPtCloudHandle)!=sim_sceneobject_pointcloud)
            return(false);
        double maxVoxelS;
        int maxPtsPerVoxel;
        int opt;
        double ptS;
        simGetPointCloudOptions(_newPtCloudHandle,&maxVoxelS,&maxPtsPerVoxel,&opt,&ptS,0);
        opt|=16;
        opt-=16;
        if (_emissivePts)
            opt|=16;
        simSetPointCloudOptions(_newPtCloudHandle,maxVoxelS,maxPtsPerVoxel,opt,_pointSize,0);
    }
    return(true);
}

bool CVisionVelodyneHDL64E::handle(double dt,std::vector<double>& pts,bool getAbsPts,std::vector<unsigned char>& retCols)
{
    pts.clear();
    retCols.clear();
    bool retVal=true;
    if (doAllObjectsExistAndAreVisionSensors()&&areVisionSensorsExplicitelyHandled())
    {
        double scanRange=_frequency*dt*2.0f*PI_VAL;
        double startAnglePlusMinusPi=lastScanAngle-PI_VAL;
        if (scanRange>=2.0f*PI_VAL)
            scanRange=2.0f*PI_VAL;
        if (_displayPts)
            _removePointsBetween(startAnglePlusMinusPi,scanRange);

        double quadrantsLowLimits[8]={-0.25f*PI_VAL,0.25f*PI_VAL,0.75f*PI_VAL,-0.75f*PI_VAL};

        double mainSensTr[12];
        double mainSensTrInv[12];
        simGetObjectMatrix(_visionSensorHandles[0],-1,mainSensTr);
        simGetObjectMatrix(_visionSensorHandles[0],-1,mainSensTrInv);
        simInvertMatrix(mainSensTrInv);
        if (_ptCloudHandle>=0)
            simModifyPointCloud(_ptCloudHandle,0,0,0);
        if (_newPtCloudHandle>=0)
            simRemovePointsFromPointCloud(_newPtCloudHandle,0,0,0,0.0,0);
        _ptCloudHandle=-1;
        int existingDisplayPointsSize=int(_displayPtsXyz.size());
        double m0[12];
        simGetObjectMatrix(_visionSensorHandles[0],-1,m0);
        for (int i=0;i<4;i++)
        {
            bool doIt=false;
            double dal=scanRange/8.0f;
            double quadrantL=quadrantsLowLimits[i];
            for (int ml=0;ml<8;ml++)
            {
                double ll=startAnglePlusMinusPi+dal*double(ml);
                if (ll>=PI_VAL)
                    ll-=2.0f*PI_VAL;
                if (   ((ll>=quadrantL)&&(ll<quadrantL+PI_VAL*0.5f)) || ((ll<=quadrantL)&&(ll<quadrantL-1.5f*PI_VAL))   )
                {
                    doIt=true;
                    break;
                }
            }
            if (doIt)
            {
                double* data;
                int* dataSize;
                if (0<=simHandleVisionSensor(_visionSensorHandles[i],&data,&dataSize))
                {
                    double farClippingPlane;
                    simGetObjectFloatParam(_visionSensorHandles[i],1001,&farClippingPlane);
                    double RR=(farClippingPlane*0.99f)*(farClippingPlane*0.99f);
                    double m[12];
                    simGetObjectMatrix(_visionSensorHandles[i],-1,m);
                    if (dataSize[0]>1)
                    {
                        int off=dataSize[1];
                        if (dataSize[2]>1)
                        {
                            int collOff=off+dataSize[2];
                            if ( (dataSize[0]<=2)||((dataSize[3])/3<(dataSize[2]-2)/16) )
                                collOff=0;
                            int ptsX=int(data[off+0]+0.5f);
                            int ptsY=int(data[off+1]+0.5f);
                            off+=2;
                            unsigned char col[3];
                            unsigned char dcol[3];
                            for (int j=0;j<ptsX*ptsY;j++)
                            {
                                double p[3]={data[off+4*j+0],data[off+4*j+1],data[off+4*j+2]};
                                if (collOff!=0)
                                {
                                    col[0]=((unsigned char*)data)[4*collOff+3*j+0];
                                    col[1]=((unsigned char*)data)[4*collOff+3*j+1];
                                    col[2]=((unsigned char*)data)[4*collOff+3*j+2];
                                }
                                double rr=p[0]*p[0]+p[1]*p[1]+p[2]*p[2];
                                if (rr<RR)
                                {
                                    double dp[3]={p[0],p[1],p[2]};
                                    simTransformVector(m,p);
                                    simTransformVector(mainSensTrInv,p);
                                    double a=atan2(p[0],p[2]);
                                    if (   ((a>=startAnglePlusMinusPi)&&(a<startAnglePlusMinusPi+scanRange)) || ((a<=startAnglePlusMinusPi)&&(a<startAnglePlusMinusPi+scanRange-2.0f*PI_VAL))   )
                                    {
                                        double r=sqrt(rr);
                                        if (_cartesianCoords)
                                        {
                                            if (getAbsPts)
                                                simTransformVector(m0,p);
                                            pts.push_back(p[0]);
                                            pts.push_back(p[1]);
                                            pts.push_back(p[2]);
                                        }
                                        else
                                        {
                                            pts.push_back(a);
                                            pts.push_back(0.5f*PI_VAL-atan2(p[1],sqrt(p[0]*p[0]+p[2]*p[2])));
                                            pts.push_back(r);
                                        }
                                        if (collOff!=0)
                                        {
                                            retCols.push_back(col[0]);
                                            retCols.push_back(col[1]);
                                            retCols.push_back(col[2]);
                                        }
                                        if (_displayPts)
                                        {   
                                            dp[0]*=_displayScalingFactor;
                                            dp[1]*=_displayScalingFactor;
                                            dp[2]*=_displayScalingFactor;
                                            simTransformVector(m,dp);
                                            _displayPtsA.push_back(a);
                                            _displayPtsXyz.push_back(dp[0]);
                                            _displayPtsXyz.push_back(dp[1]);
                                            _displayPtsXyz.push_back(dp[2]);
                                             _getColorFromIntensity(1.0f-((r-_coloringDistances[0])/(_coloringDistances[1]-_coloringDistances[0])),dcol);
                                            _displayPtsCol.push_back(dcol[0]);
                                            _displayPtsCol.push_back(dcol[1]);
                                            _displayPtsCol.push_back(dcol[2]);
                                        }
                                    }
                                }
                            }
                        }
                        else
                            retVal=false;
                    }
                    else
                        retVal=false;
                    simReleaseBuffer((char*)data);
                    simReleaseBuffer((char*)dataSize);
                }
                else
                    retVal=false;
            }
        }
        if (_displayPts&&(_displayPtsXyz.size()>0))
        {
            char zeroCol[12]={0,0,0,0,0,0,0,0,0,0,0,0};
            int options=2;
            if (_emissivePts)
                options|=4;
            if (_displayOnlyCurrent)
            {
                if (int(_displayPtsXyz.size())>existingDisplayPointsSize)
                {
                    // Using the new or old pt cloud functionality?
                    if (_newPtCloudHandle>=0)
                        simInsertPointsIntoPointCloud(_newPtCloudHandle,2,&_displayPtsXyz[existingDisplayPointsSize],((int)_displayPtsXyz.size()/3)-existingDisplayPointsSize/3,&_displayPtsCol[existingDisplayPointsSize],0);
                    else
                        _ptCloudHandle=simAddPointCloud(0,255,-1,options,_pointSize,((int)_displayPtsXyz.size()/3)-existingDisplayPointsSize/3,&_displayPtsXyz[existingDisplayPointsSize],zeroCol,(char*)&_displayPtsCol[existingDisplayPointsSize],0);
                }
            }
            else
            {
                    // Using the new or old pt cloud functionality?
                    if (_newPtCloudHandle>=0)
                        simInsertPointsIntoPointCloud(_newPtCloudHandle,2,&_displayPtsXyz[0],(int)_displayPtsXyz.size()/3,&_displayPtsCol[0],0);
                    else
                        _ptCloudHandle=simAddPointCloud(0,255,-1,options,_pointSize,(int)_displayPtsXyz.size()/3,&_displayPtsXyz[0],zeroCol,(char*)&_displayPtsCol[0],0);
            }
        }
        lastScanAngle=fmod(lastScanAngle+scanRange,2.0f*PI_VAL);
    }
    else
        retVal=false;
    return(retVal);
}

void CVisionVelodyneHDL64E::_removePointsBetween(double lowAngle,double range)
{
    std::vector<double> displayPtsXyz(_displayPtsXyz);
    std::vector<double> displayPtsA(_displayPtsA);
    std::vector<unsigned char> displayPtsCol(_displayPtsCol);
    _displayPtsXyz.clear();
    _displayPtsA.clear();
    _displayPtsCol.clear();
    for (int i=0;i<int(displayPtsA.size());i++)
    {
        if ( ((displayPtsA[i]<lowAngle)||(displayPtsA[i]>=lowAngle+range)) && ((displayPtsA[i]>lowAngle)||(displayPtsA[i]>=lowAngle+range-2.0f*PI_VAL)) )
        {
            _displayPtsA.push_back(displayPtsA[i]);
            _displayPtsXyz.push_back(displayPtsXyz[3*i+0]);
            _displayPtsXyz.push_back(displayPtsXyz[3*i+1]);
            _displayPtsXyz.push_back(displayPtsXyz[3*i+2]);
            _displayPtsCol.push_back(displayPtsCol[3*i+0]);
            _displayPtsCol.push_back(displayPtsCol[3*i+1]);
            _displayPtsCol.push_back(displayPtsCol[3*i+2]);
        }
    }
}

void CVisionVelodyneHDL64E::_getColorFromIntensity(double intensity,unsigned char col[3])
{
    if (intensity>1.0f)
        intensity=1.0f;
    if (intensity<0.0f)
        intensity=0.0f;
    const double c[12]={0.0f,0.0f,1.0f,1.0f,0.0f,1.0f,1.0f,0.0f,0.0f,1.0f,1.0f,0.0f};
    int d=int(intensity*3);
    if (d>2)
        d=2;
    double r=(intensity-double(d)/3.0f)*3.0f;
    col[0]=(unsigned char)(255.0f*(c[3*d+0]*(1.0f-r)+c[3*(d+1)+0]*r));
    col[1]=(unsigned char)(255.0f*(c[3*d+1]*(1.0f-r)+c[3*(d+1)+1]*r));
    col[2]=(unsigned char)(255.0f*(c[3*d+2]*(1.0f-r)+c[3*(d+1)+2]*r));
}
