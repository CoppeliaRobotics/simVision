#include "visionTransf.h"
#include "simLib.h"
#include <math.h>

#define PI_VAL (3.14159265f)

CVisionTransf::CVisionTransf(int scriptHandle,int passiveVisionSensorHandle,const int activeVisionSensorHandles[6],float horizontalAngle,float verticalAngle,int passiveVisionSensorHandleForDepth)
{
    _scriptHandle=scriptHandle;
    _passiveVisionSensorHandle=passiveVisionSensorHandle;
    _passiveVisionSensorHandleForDepth=passiveVisionSensorHandleForDepth;
    _referencePassiveVisionSensorHandle=_passiveVisionSensorHandle;
    if (_passiveVisionSensorHandle==-1)
        _referencePassiveVisionSensorHandle=passiveVisionSensorHandleForDepth;
    for (int i=0;i<6;i++)
        _activeVisionSensorHandles[i]=activeVisionSensorHandles[i];
    _horizontalAngle=horizontalAngle;
    _verticalAngle=verticalAngle;
    if (_horizontalAngle<5.0f*PI_VAL/180.0f)
        _horizontalAngle=5.0f*PI_VAL/180.0f;
    if (_horizontalAngle>2.0f*PI_VAL)
        _horizontalAngle=2.0f*PI_VAL;
    if (_verticalAngle<5.0f*PI_VAL/180.0f)
        _verticalAngle=5.0f*PI_VAL/180.0f;
    if (_verticalAngle>PI_VAL)
        _verticalAngle=PI_VAL;
    if (doAllObjectsExistAndAreVisionSensors())
    {
        disableSpecularLightComponent(true);
        isActiveVisionSensorResolutionCorrect();
    }
    for (int i=0;i<6;i++)
        _activeVisionSensorImages[i]=0;

    _passiveVisionSensorResolution[0]=0;
    _passiveVisionSensorResolution[1]=0;
    simGetVisionSensorResolution(_referencePassiveVisionSensorHandle,_passiveVisionSensorResolution);
    
    _passiveVisionSensorImage=new float[_passiveVisionSensorResolution[0]*_passiveVisionSensorResolution[1]*3];

    _mapP=new int[_passiveVisionSensorResolution[0]*_passiveVisionSensorResolution[1]];
    _mapI=new float[_passiveVisionSensorResolution[0]*_passiveVisionSensorResolution[1]];
    _mapV=new unsigned char[_passiveVisionSensorResolution[0]*_passiveVisionSensorResolution[1]];

    _calculateMapping();
}

CVisionTransf::~CVisionTransf()
{
    releaseActiveVisionSensorImages();
    delete[] _passiveVisionSensorImage;
    delete[] _mapP;
    delete[] _mapI;
    delete[] _mapV;
    disableSpecularLightComponent(false);
}

int CVisionTransf::getRelatedScriptHandle() const
{
    return(_scriptHandle);
}

bool CVisionTransf::isActiveVisionSensorResolutionCorrect()
{
    int res=-1;
    for (int i=0;i<6;i++)
    {
        int r[2];
        if (simGetVisionSensorResolution(_activeVisionSensorHandles[i],r)==-1)
            return(false);
        if (res==-1)
            res=r[0];
        if ((res!=r[0])||(res!=r[1]))
            return(false);
    }
    _activeVisionSensorResolutionXY=res;
    return(_activeVisionSensorResolutionXY!=-1);
}

bool CVisionTransf::areRGBAndDepthVisionSensorResolutionsCorrect() const
{
    if (_passiveVisionSensorHandle!=-1)
    {
        int r[2];
        if (simGetVisionSensorResolution(_passiveVisionSensorHandle,r)==-1)
            return(false);
        if ((_passiveVisionSensorResolution[0]!=r[0])||(_passiveVisionSensorResolution[1]!=r[1]))
            return(false);
    }
    if (_passiveVisionSensorHandleForDepth!=-1)
    {
        int r[2];
        if (simGetVisionSensorResolution(_passiveVisionSensorHandleForDepth,r)==-1)
            return(false);
        if ((_passiveVisionSensorResolution[0]!=r[0])||(_passiveVisionSensorResolution[1]!=r[1]))
            return(false);
    }
    return(true);
}

bool CVisionTransf::areActiveVisionSensorsExplicitelyHandled() const
{
    for (int i=0;i<6;i++)
    {
        int r=simGetExplicitHandling(_activeVisionSensorHandles[i]);
        if (r==-1)
            return(false);
        if ((r&1)==0)
            return(false);
    }
    return(true);
}

bool CVisionTransf::doAllObjectsExistAndAreVisionSensors() const
{
    if (_passiveVisionSensorHandle!=-1)
    {
        if (simGetObjectType(_passiveVisionSensorHandle)!=sim_object_visionsensor_type)
            return(false);
    }
    if (_passiveVisionSensorHandleForDepth!=-1)
    {
        if (simGetObjectType(_passiveVisionSensorHandleForDepth)!=sim_object_visionsensor_type)
            return(false);
    }
    for (int i=0;i<6;i++)
    {
        if (simGetObjectType(_activeVisionSensorHandles[i])!=sim_object_visionsensor_type)
            return(false);
    }
    return(true);
}

bool CVisionTransf::isSame(int scriptHandle,const int activeVisionSensorHandles[6],float horizontalAngle,float verticalAngle,int passive1,int passive2) const
{
    if (scriptHandle!=_scriptHandle)
        return(false);
    if (horizontalAngle!=_horizontalAngle)
        return(false);
    if (verticalAngle!=_verticalAngle)
        return(false);
    for (int i=0;i<6;i++)
    {
        if (activeVisionSensorHandles[i]!=_activeVisionSensorHandles[i])
            return(false);
    }
    if (passive1!=_passiveVisionSensorHandle)
        return(false);
    if (passive2!=_passiveVisionSensorHandleForDepth)
        return(false);
    int r[2]={0,0};
    simGetVisionSensorResolution(_referencePassiveVisionSensorHandle,r);
    if ((r[0]!=_passiveVisionSensorResolution[0])||(r[1]!=_passiveVisionSensorResolution[1]))
        return(false);
    return(true);
}

int CVisionTransf::getReferencePassiveVisionSensorHandle() const
{
    return(_referencePassiveVisionSensorHandle);
}

void CVisionTransf::releaseActiveVisionSensorImages()
{
    for (int i=0;i<6;i++)
    {
        simReleaseBuffer((char*)_activeVisionSensorImages[i]);
        _activeVisionSensorImages[i]=0;
    }
}


void CVisionTransf::handleObject()
{
    for (int i=0;i<6;i++)
    {
        if (_usedActiveVisionSensors[i])
            simHandleVisionSensor(_activeVisionSensorHandles[i],0,0);
    }

    // For the RGB part:
    if (_passiveVisionSensorHandle!=-1)
    {
        for (int i=0;i<6;i++)
        {
            if (_usedActiveVisionSensors[i])
                _activeVisionSensorImages[i]=simGetVisionSensorImage(_activeVisionSensorHandles[i]);
        }

        int ptCnt=_passiveVisionSensorResolution[0]*_passiveVisionSensorResolution[1];
        int c,p,v;
        for (int i=0;i<ptCnt;i++)
        {
            c=3*i;
            p=3*_mapP[i];
            v=_mapV[i];
            _passiveVisionSensorImage[c+0]=_activeVisionSensorImages[v][p+0];
            _passiveVisionSensorImage[c+1]=_activeVisionSensorImages[v][p+1];
            _passiveVisionSensorImage[c+2]=_activeVisionSensorImages[v][p+2];
        }
        simSetVisionSensorImage(_passiveVisionSensorHandle,_passiveVisionSensorImage);
        releaseActiveVisionSensorImages();
    }

    // For the depth part:
    if (_passiveVisionSensorHandleForDepth!=-1)
    {
        float np,fp;
        simGetObjectFloatParameter(_activeVisionSensorHandles[0],sim_visionfloatparam_near_clipping,&np);
        simGetObjectFloatParameter(_activeVisionSensorHandles[0],sim_visionfloatparam_far_clipping,&fp);
        float fmn=fp-np;

        for (int i=0;i<6;i++)
        {
            if (_usedActiveVisionSensors[i])
                _activeVisionSensorImages[i]=simGetVisionSensorDepthBuffer(_activeVisionSensorHandles[i]);
        }

        int ptCnt=_passiveVisionSensorResolution[0]*_passiveVisionSensorResolution[1];
        int p,v;
        for (int i=0;i<ptCnt;i++)
        {
            p=_mapP[i];
            v=_mapV[i];

            float d=(np+fmn*_activeVisionSensorImages[v][p])*_mapI[i];
            d=(d-np)/fmn;
            if (d>1.0f)
                d=1.0f;

            _passiveVisionSensorImage[i]=d;
        }
        simSetVisionSensorImage(_passiveVisionSensorHandleForDepth+sim_handleflag_greyscale,_passiveVisionSensorImage);
        releaseActiveVisionSensorImages();
    }
}

void CVisionTransf::disableSpecularLightComponent(bool d)
{
    int v=0;
    if (d)
        v=4;
    for (int i=0;i<6;i++)
        simSetObjectIntParameter(_activeVisionSensorHandles[i],1006,v);
}


void CVisionTransf::_calculateMapping()
{
    for (int i=0;i<6;i++)
        _usedActiveVisionSensors[i]=false;

    float halfAngleX=_horizontalAngle/2.0f;
    float halfAngleY=_verticalAngle/2.0f;
    float dx=0.0f;
    float dy=0.0f;
    if (_passiveVisionSensorResolution[0]>1)
        dx=_horizontalAngle/(_passiveVisionSensorResolution[0]-1);
    else
        halfAngleX=0.0f;
    if (_passiveVisionSensorResolution[1]>1)
        dy=_verticalAngle/(_passiveVisionSensorResolution[1]-1);
    else
        halfAngleY=0.0f;
    for (int j=0;j<_passiveVisionSensorResolution[1];j++)
    {
        for (int i=0;i<_passiveVisionSensorResolution[0];i++)
        {
            int cnt=(i+j*_passiveVisionSensorResolution[0]);
            
            float angleX=-halfAngleX+i*dx;
            float angleY=-halfAngleY+j*dy;
            
            float x=cos(angleX)*cos(angleY);
            float y=sin(angleX)*cos(angleY);
            float z=sin(angleY);

            float maxV=fabs(x);
            if (fabs(y)>maxV)
                maxV=fabs(y);
            if (fabs(z)>maxV)
                maxV=fabs(z);

            x/=maxV;
            y/=maxV;
            z/=maxV;

            int imgIndex=0;
            if ((fabs(x)>=fabs(y))&&(fabs(x)>=fabs(z)))
            { // front or back
                if (x>=0.0f)
                    imgIndex=0;
                else
                    imgIndex=2;
            }
            if ((fabs(y)>fabs(x))&&(fabs(y)>=fabs(z)))
            { // left or right
                if (y>=0.0f)
                    imgIndex=4;
                else
                    imgIndex=5;
            }
            if ((fabs(z)>fabs(x))&&(fabs(z)>fabs(y)))
            { // top or bottom
                if (z>=0.0f)
                    imgIndex=1;
                else
                    imgIndex=3;
            }

            float xl,yl;
            if (imgIndex==0)
            { // front
                xl=-y;
                yl=z;
            }
            if (imgIndex==1)
            { // top
                xl=-y;
                yl=-x;
            }
            if (imgIndex==2)
            { // back
                xl=-y;
                yl=-z;
            }
            if (imgIndex==3)
            { // bottom
                xl=-y;
                yl=x;
            }
            if (imgIndex==4)
            { // left
                xl=x;
                yl=z;
            }
            if (imgIndex==5)
            { // right
                xl=-x;
                yl=z;
            }

            int pixelX=int((xl+1.0)*0.5f*float(_activeVisionSensorResolutionXY)-0.5f);
            int pixelY=int((yl+1.0)*0.5f*float(_activeVisionSensorResolutionXY)-0.5f);

            _mapP[cnt]=pixelX+pixelY*_activeVisionSensorResolutionXY;
            _mapV[cnt]=imgIndex;
            _mapI[cnt]=fabs(1.0f/(cos(atan(xl))*cos(atan(yl))));

            _usedActiveVisionSensors[imgIndex]=true;
        }
    }
}
