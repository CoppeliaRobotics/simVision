#include "simVision.h"
#include <simLib/scriptFunctionData.h>
#include "vis.h"
#include <iostream>
#include "visionCont.h"
#include "visionTransfCont.h"
#include "visionRemapCont.h"
#include "visionVelodyneHDL64ECont.h"
#include "visionVelodyneVPL16Cont.h"
#include "imageProcess.h"
#include <simMath/7Vector.h>
#include <map>
#include <algorithm>

#ifdef _WIN32
    #ifdef QT_COMPIL
        #include <direct.h>
    #else
        #include <shlwapi.h>
        #pragma comment(lib, "Shlwapi.lib")
    #endif
#endif
#if defined (__linux) || defined (__APPLE__)
    #include <unistd.h>
#endif

static LIBRARY simLib;
static CVisionCont* visionContainer;
static CVisionTransfCont* visionTransfContainer;
static CVisionRemapCont* visionRemapContainer;
static CVisionVelodyneHDL64ECont* visionVelodyneHDL64EContainer;
static CVisionVelodyneVPL16Cont* visionVelodyneVPL16Container;
static std::string _pluginName;

// --------------------------------------------------------------------------------------
// simVision.Distort
// --------------------------------------------------------------------------------------
#define LUA_DISTORT_COMMAND_PLUGIN "distort"

const int inArgs_DISTORT[]={
    3,
    sim_script_arg_int32,0,
    sim_script_arg_int32|sim_script_arg_table,0,
    sim_script_arg_double|sim_script_arg_table,0,
};

void LUA_DISTORT_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    int result=-1;
    if (D.readDataFromStack(p->stackID,inArgs_DISTORT,inArgs_DISTORT[0]-2,nullptr)) // last 2 args are optional
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int visionSensorHandle=inData->at(0).int32Data[0];
        if (simGetObjectType(visionSensorHandle)==sim_object_visionsensor_type)
        {
            int r[2];
            simGetVisionSensorRes(visionSensorHandle,r);
            CVisionRemap* obj=nullptr;
            if (inData->size()>=2)
            {
                int* pixelMap=nullptr;
                if (inData->at(1).int32Data.size()==r[0]*r[1])
                    pixelMap=&inData->at(1).int32Data[0];
                double* depthScaling=nullptr;
                bool depthOk=true;
                if (inData->size()>=3)
                {
                    if (inData->at(2).doubleData.size()==r[0]*r[1])
                        depthScaling=&inData->at(2).doubleData[0];
                    else
                        depthOk=false;
                }
                if ( (pixelMap!=nullptr)&&depthOk )
                {
                    obj=visionRemapContainer->getObjectFromSensorHandle(visionSensorHandle);
                    if (obj!=nullptr)
                    {
                        if (!obj->isSame(p->scriptID,pixelMap,depthScaling))
                        {
                            visionRemapContainer->removeObjectFromSensorHandle(visionSensorHandle);
                            obj=nullptr;
                        }
                        else
                            simAddLog(_pluginName.c_str(),sim_verbosity_scriptwarnings,"Mapping was already initialized, constantly handing over mapping arguments will slow down operation.");
                    }
                    if (obj==nullptr)
                    {
                        obj=new CVisionRemap(p->scriptID,visionSensorHandle,pixelMap,depthScaling);
                        visionRemapContainer->addObject(obj);
                    }
                }
                else
                    simSetLastError(nullptr,"Invalid argument(s)."); // output an error
            }
            else
            {
                obj=visionRemapContainer->getObjectFromSensorHandle(visionSensorHandle);
                if (obj==nullptr)
                    simSetLastError(nullptr,"Mapping was not yet initialized."); // output an error
            }
            if (obj!=nullptr)
            {
                obj->handleObject();
                result=1; // success
            }
        }
        else
            simSetLastError(nullptr,"Invalid vision sensor handle."); // output an error
    }
    D.pushOutData(CScriptFunctionDataItem(result));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simVision.handleSpherical
// --------------------------------------------------------------------------------------
#define LUA_HANDLESPHERICAL_COMMAND_PLUGIN "handleSpherical"

const int inArgs_HANDLESPHERICAL[]={
    5,
    sim_script_arg_int32,0,
    sim_script_arg_int32|sim_script_arg_table,6,
    sim_script_arg_double,0,
    sim_script_arg_double,0,
    sim_script_arg_int32,0,
};

void LUA_HANDLESPHERICAL_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    int result=-1;
    if (D.readDataFromStack(p->stackID,inArgs_HANDLESPHERICAL,inArgs_HANDLESPHERICAL[0]-1,nullptr)) // last arg is optional
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int passiveVisionSensorHandle=inData->at(0).int32Data[0];
        int activeVisionSensorHandes[6];
        for (int i=0;i<6;i++)
            activeVisionSensorHandes[i]=inData->at(1).int32Data[i];
        double horizontalAngle=inData->at(2).doubleData[0];
        double verticalAngle=inData->at(3).doubleData[0];
        int passiveVisionSensor2Handle=-1;
        if (inData->size()>=5)
            passiveVisionSensor2Handle=inData->at(4).int32Data[0]; // for the depth map
        int h=passiveVisionSensorHandle;
        if (h==-1)
            h=passiveVisionSensor2Handle;
        CVisionTransf* obj=visionTransfContainer->getVisionTransfFromReferencePassiveVisionSensor(h);
        if (obj!=NULL)
        {
            if (!obj->isSame(p->scriptID,activeVisionSensorHandes,horizontalAngle,verticalAngle,passiveVisionSensorHandle,passiveVisionSensor2Handle))
            {
                visionTransfContainer->removeObjectFromPassiveSensorHandle(h);
                obj=NULL;
            }
        }
        if (obj==NULL)
        {
            obj=new CVisionTransf(p->scriptID,passiveVisionSensorHandle,activeVisionSensorHandes,horizontalAngle,verticalAngle,passiveVisionSensor2Handle);
            visionTransfContainer->addObject(obj);
        }

        if (obj->doAllObjectsExistAndAreVisionSensors())
        {
            if (obj->isActiveVisionSensorResolutionCorrect())
            {
                if (obj->areActiveVisionSensorsExplicitelyHandled())
                {
                    if (!obj->areRGBAndDepthVisionSensorResolutionsCorrect())
                        simSetLastError(nullptr,"Invalid vision sensor resolution for the RGB or depth component."); // output an error
                    else
                    {
                        obj->handleObject();
                        result=1; // success
                    }
                }
                else
                    simSetLastError(nullptr,"Active vision sensors should be explicitely handled."); // output an error
            }
            else
                simSetLastError(nullptr,"Invalid vision sensor resolutions."); // output an error
        }
        else
            simSetLastError(nullptr,"Invalid handles, or handles are not vision sensor handles."); // output an error

        if (result==-1)
            visionTransfContainer->removeObjectFromPassiveSensorHandle(h);

    }
    D.pushOutData(CScriptFunctionDataItem(result));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simVision.handleAnaglyphStereo
// --------------------------------------------------------------------------------------
#define LUA_HANDLEANAGLYPHSTEREO_COMMAND_PLUGIN "handleAnaglyphStereo"

const int inArgs_HANDLEANAGLYPHSTEREO[]={
    3,
    sim_script_arg_int32,0,
    sim_script_arg_int32|sim_script_arg_table,2,
    sim_script_arg_double|sim_script_arg_table,6,
};

void LUA_HANDLEANAGLYPHSTEREO_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    int result=-1;
    if (D.readDataFromStack(p->stackID,inArgs_HANDLEANAGLYPHSTEREO,inArgs_HANDLEANAGLYPHSTEREO[0]-1,nullptr)) // -1 because last arg is optional
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int passiveVisionSensorHande=inData->at(0).int32Data[0];
        int leftSensorHandle=inData->at(1).int32Data[0];
        int rightSensorHandle=inData->at(1).int32Data[1];
        // Check the object types:
        bool existAndAreVisionSensors=true;
        if (simGetObjectType(passiveVisionSensorHande)!=sim_object_visionsensor_type)
            existAndAreVisionSensors=false;
        if (simGetObjectType(leftSensorHandle)!=sim_object_visionsensor_type)
            existAndAreVisionSensors=false;
        if (simGetObjectType(rightSensorHandle)!=sim_object_visionsensor_type)
            existAndAreVisionSensors=false;
        if (existAndAreVisionSensors)
        { // check the sensor resolutions:
            int r[2];
            simGetVisionSensorRes(passiveVisionSensorHande,r);
            int rl[2];
            simGetVisionSensorRes(leftSensorHandle,rl);
            int rr[2];
            simGetVisionSensorRes(rightSensorHandle,rr);
            if ((r[0]==rl[0])&&(r[0]==rr[0])&&(r[1]==rl[1])&&(r[1]==rr[1]))
            { // check if the sensors are explicitely handled:
                int e=simGetExplicitHandling(passiveVisionSensorHande);
                int el=simGetExplicitHandling(leftSensorHandle);
                int er=simGetExplicitHandling(rightSensorHandle);
                if ((e&el&er&1)==1)
                {
                    double leftAndRightColors[6]={1.0f,0.0f,0.0f,0.0f,1.0f,1.0f}; // default
                    if (inData->size()>2)
                    { // we have the optional argument
                        for (int i=0;i<6;i++)
                            leftAndRightColors[i]=inData->at(2).doubleData[i];
                    }
                    simHandleVisionSensor(leftSensorHandle,NULL,NULL);
                    double* leftImage=getVisionSensorImage(leftSensorHandle);
                    simHandleVisionSensor(rightSensorHandle,NULL,NULL);
                    double* rightImage=getVisionSensorImage(rightSensorHandle);
                    for (int i=0;i<r[0]*r[1];i++)
                    {
                        double il=(leftImage[3*i+0]+leftImage[3*i+1]+leftImage[3*i+2])/3.0f;
                        double ir=(rightImage[3*i+0]+rightImage[3*i+1]+rightImage[3*i+2])/3.0f;
                        leftImage[3*i+0]=il*leftAndRightColors[0]+ir*leftAndRightColors[3];
                        leftImage[3*i+1]=il*leftAndRightColors[1]+ir*leftAndRightColors[4];
                        leftImage[3*i+2]=il*leftAndRightColors[2]+ir*leftAndRightColors[5];
                    }
                    setVisionSensorImage(passiveVisionSensorHande,leftImage);
                    simReleaseBuffer((char*)leftImage);
                    simReleaseBuffer((char*)rightImage);
                    result=1;
                }
                else
                    simSetLastError(nullptr,"Vision sensors should be explicitely handled."); // output an error
            }
            else
                simSetLastError(nullptr,"Invalid vision sensor resolutions."); // output an error
        }
        else
            simSetLastError(nullptr,"Invalid handles, or handles are not vision sensor handles."); // output an error
    }
    D.pushOutData(CScriptFunctionDataItem(result));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simVision.createVelodyneHDL64E
// --------------------------------------------------------------------------------------
#define LUA_CREATEVELODYNEHDL64E_COMMAND_PLUGIN "createVelodyneHDL64E"

const int inArgs_CREATEVELODYNEHDL64E[]={
    7,
    sim_script_arg_int32|sim_script_arg_table,4,
    sim_script_arg_double,0,
    sim_script_arg_int32,0,
    sim_script_arg_double,0,
    sim_script_arg_double|sim_script_arg_table,2,
    sim_script_arg_double,0,
    sim_script_arg_int32,0,
};

void LUA_CREATEVELODYNEHDL64E_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    int velodyneHandle=-1;
    if (D.readDataFromStack(p->stackID,inArgs_CREATEVELODYNEHDL64E,inArgs_CREATEVELODYNEHDL64E[0]-5,nullptr)) // -5 because the last 5 arguments are optional
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int options=0;
        double pointSize=2.0f;
        double scalingFactor=1.0f;
        double coloringDistances[2]={1,5};
        int visionSensorHandes[4];
        for (int i=0;i<4;i++)
            visionSensorHandes[i]=inData->at(0).int32Data[i];
        double frequency=inData->at(1).doubleData[0];
        int pointCloudHandle=-1;
        if (inData->size()>2)
        { // we have the optional 'options' argument:
            options=inData->at(2).int32Data[0];
        }
        if (inData->size()>3)
        { // we have the optional 'pointSize' argument:
            pointSize=inData->at(3).doubleData[0];
        }
        if (inData->size()>4)
        { // we have the optional 'coloringDistance' argument:
            coloringDistances[0]=inData->at(4).doubleData[0];
            coloringDistances[1]=inData->at(4).doubleData[1];
        }
        if (inData->size()>5)
        { // we have the optional 'displayScalingFactor' argument:
            scalingFactor=inData->at(5).doubleData[0];
        }
        if (inData->size()>6)
        { // we have the optional 'pointCloudHandle' argument:
            pointCloudHandle=inData->at(6).int32Data[0];
        }
        CVisionVelodyneHDL64E* obj=new CVisionVelodyneHDL64E(p->scriptID,visionSensorHandes,frequency,options,pointSize,coloringDistances,scalingFactor,pointCloudHandle);
        visionVelodyneHDL64EContainer->addObject(obj);
        if (obj->doAllObjectsExistAndAreVisionSensors())
        {
            if (obj->areVisionSensorsExplicitelyHandled())
                velodyneHandle=obj->getVelodyneHandle(); // success
            else
                simSetLastError(nullptr,"Vision sensors should be explicitely handled."); // output an error
        }
        else
            simSetLastError(nullptr,"Invalid handles, or handles are not vision sensor handles."); // output an error

        if (velodyneHandle==-1)
            visionVelodyneHDL64EContainer->removeObjectFromSensorHandle(obj->getVelodyneHandle());
    }
    D.pushOutData(CScriptFunctionDataItem(velodyneHandle));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simVision.destroyVelodyneHDL64E
// --------------------------------------------------------------------------------------
#define LUA_DESTROYVELODYNEHDL64E_COMMAND_PLUGIN "destroyVelodyneHDL64E"

const int inArgs_DESTROYVELODYNEHDL64E[]={
    1,
    sim_script_arg_int32,0,
};

void LUA_DESTROYVELODYNEHDL64E_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    int result=-1;
    if (D.readDataFromStack(p->stackID,inArgs_DESTROYVELODYNEHDL64E,inArgs_DESTROYVELODYNEHDL64E[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int handle=inData->at(0).int32Data[0];
        CVisionVelodyneHDL64E* obj=visionVelodyneHDL64EContainer->getObject(handle);
        if (obj!=NULL)
        {
            visionVelodyneHDL64EContainer->removeObjectFromSensorHandle(obj->getVelodyneHandle());
            result=1;
        }
        else
            simSetLastError(nullptr,"Invalid handle."); // output an error
    }
    D.pushOutData(CScriptFunctionDataItem(result));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simVision.handleVelodyneHDL64E
// --------------------------------------------------------------------------------------
#define LUA_HANDLEVELODYNEHDL64E_COMMAND_PLUGIN "handleVelodyneHDL64E"

const int inArgs_HANDLEVELODYNEHDL64E[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_double,0,
};

void LUA_HANDLEVELODYNEHDL64E_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    std::vector<double> pts;
    std::vector<unsigned char> cols;
    bool result=false;
    bool codedString=false;
    if (D.readDataFromStack(p->stackID,inArgs_HANDLEVELODYNEHDL64E,inArgs_HANDLEVELODYNEHDL64E[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int handle=inData->at(0).int32Data[0];
        bool absCoords=false;
        if (handle>=0)
        {
            codedString=((handle&sim_handleflag_codedstring)!=0);
            absCoords=((handle&sim_handleflag_abscoords)!=0);
            handle=handle&0x000fffff;
        }
        double dt=inData->at(1).doubleData[0];
        CVisionVelodyneHDL64E* obj=visionVelodyneHDL64EContainer->getObject(handle);
        if (obj!=NULL)
            result=obj->handle(dt,pts,absCoords,cols);
        else
            simSetLastError(nullptr,"Invalid handle."); // output an error
    }
    if (result)
    {
        if (codedString)
        {
            if (pts.size()>0)
            {
                std::vector<float> pp;
                pp.resize(pts.size());
                for (size_t i=0;i<pts.size();i++)
                    pp[i]=(float)pts[i];
                D.pushOutData(CScriptFunctionDataItem((char*)&pp[0],pts.size()*sizeof(float)));
            }
            else
                D.pushOutData(CScriptFunctionDataItem(nullptr,0));
        }
        else
            D.pushOutData(CScriptFunctionDataItem(pts));
        if (cols.size()>0)
            D.pushOutData(CScriptFunctionDataItem((char*)&cols[0],cols.size()));
    }
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simVision.createVelodyneVPL16
// --------------------------------------------------------------------------------------
#define LUA_CREATEVELODYNEVPL16_COMMAND_PLUGIN "createVelodyneVPL16"

const int inArgs_CREATEVELODYNEVPL16[]={
    7,
    sim_script_arg_int32|sim_script_arg_table,4,
    sim_script_arg_double,0,
    sim_script_arg_int32,0,
    sim_script_arg_double,0,
    sim_script_arg_double|sim_script_arg_table,2,
    sim_script_arg_double,0,
    sim_script_arg_int32,0,
};

void LUA_CREATEVELODYNEVPL16_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    int velodyneHandle=-1;
    if (D.readDataFromStack(p->stackID,inArgs_CREATEVELODYNEVPL16,inArgs_CREATEVELODYNEVPL16[0]-5,nullptr)) // -5 because the last 5 arguments are optional
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int options=0;
        double pointSize=2.0f;
        double scalingFactor=1.0f;
        double coloringDistances[2]={1,5};
        int visionSensorHandes[4];
        for (int i=0;i<4;i++)
            visionSensorHandes[i]=inData->at(0).int32Data[i];
        double frequency=inData->at(1).doubleData[0];
        int pointCloudHandle=-1;
        if (inData->size()>2)
        { // we have the optional 'options' argument:
            options=inData->at(2).int32Data[0];
        }
        if (inData->size()>3)
        { // we have the optional 'pointSize' argument:
            pointSize=inData->at(3).doubleData[0];
        }
        if (inData->size()>4)
        { // we have the optional 'coloringDistance' argument:
            coloringDistances[0]=inData->at(4).doubleData[0];
            coloringDistances[1]=inData->at(4).doubleData[1];
        }
        if (inData->size()>5)
        { // we have the optional 'displayScalingFactor' argument:
            scalingFactor=inData->at(5).doubleData[0];
        }
        if (inData->size()>6)
        { // we have the optional 'pointCloudHandle' argument:
            pointCloudHandle=inData->at(6).int32Data[0];
        }
        CVisionVelodyneVPL16* obj=new CVisionVelodyneVPL16(p->scriptID,visionSensorHandes,frequency,options,pointSize,coloringDistances,scalingFactor,pointCloudHandle);
        visionVelodyneVPL16Container->addObject(obj);
        if (obj->doAllObjectsExistAndAreVisionSensors())
        {
            if (obj->areVisionSensorsExplicitelyHandled())
                velodyneHandle=obj->getVelodyneHandle(); // success
            else
                simSetLastError(nullptr,"Vision sensors should be explicitely handled."); // output an error
        }
        else
            simSetLastError(nullptr,"Invalid handles, or handles are not vision sensor handles."); // output an error

        if (velodyneHandle==-1)
            visionVelodyneVPL16Container->removeObjectFromSensorHandle(obj->getVelodyneHandle());
    }
    D.pushOutData(CScriptFunctionDataItem(velodyneHandle));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simVision.destroyVelodyneVPL16
// --------------------------------------------------------------------------------------
#define LUA_DESTROYVELODYNEVPL16_COMMAND_PLUGIN "destroyVelodyneVPL16"

const int inArgs_DESTROYVELODYNEVPL16[]={
    1,
    sim_script_arg_int32,0,
};

void LUA_DESTROYVELODYNEVPL16_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    int result=-1;
    if (D.readDataFromStack(p->stackID,inArgs_DESTROYVELODYNEVPL16,inArgs_DESTROYVELODYNEVPL16[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int handle=inData->at(0).int32Data[0];
        CVisionVelodyneVPL16* obj=visionVelodyneVPL16Container->getObject(handle);
        if (obj!=NULL)
        {
            visionVelodyneVPL16Container->removeObjectFromSensorHandle(obj->getVelodyneHandle());
            result=1;
        }
        else
            simSetLastError(nullptr,"Invalid handle."); // output an error
    }
    D.pushOutData(CScriptFunctionDataItem(result));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simVision.handleVelodyneVPL16
// --------------------------------------------------------------------------------------
#define LUA_HANDLEVELODYNEVPL16_COMMAND_PLUGIN "handleVelodyneVPL16"

const int inArgs_HANDLEVELODYNEVPL16[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_double,0,
};

void LUA_HANDLEVELODYNEVPL16_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    std::vector<double> pts;
    std::vector<unsigned char> cols;
    bool result=false;
    bool codedString=false;
    if (D.readDataFromStack(p->stackID,inArgs_HANDLEVELODYNEVPL16,inArgs_HANDLEVELODYNEVPL16[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int handle=inData->at(0).int32Data[0];
        bool absCoords=false;
        if (handle>=0)
        {
            codedString=((handle&sim_handleflag_codedstring)!=0);
            absCoords=((handle&sim_handleflag_abscoords)!=0);
            handle=handle&0x000fffff;
        }
        double dt=inData->at(1).doubleData[0];
        CVisionVelodyneVPL16* obj=visionVelodyneVPL16Container->getObject(handle);
        if (obj!=NULL)
            result=obj->handle(dt,pts,absCoords,cols);
        else
            simSetLastError(nullptr,"Invalid handle."); // output an error
    }
    if (result)
    {
        if (codedString)
        {
            if (pts.size()>0)
            {
                std::vector<float> pp;
                pp.resize(pts.size());
                for (size_t i=0;i<pts.size();i++)
                    pp[i]=(float)pts[i];
                D.pushOutData(CScriptFunctionDataItem((char*)&pp[0],pts.size()*sizeof(float)));
            }
            else
                D.pushOutData(CScriptFunctionDataItem(nullptr,0));
        }
        else
            D.pushOutData(CScriptFunctionDataItem(pts));
        if (cols.size()>0)
            D.pushOutData(CScriptFunctionDataItem((char*)&cols[0],cols.size()));
    }
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

int getVisionSensorHandle(int handle,int attachedObj)
{
    if (handle==sim_handle_self)
        handle=attachedObj;
    return(handle);
}

// --------------------------------------------------------------------------------------
// simVision.sensorImgToWorkImg
// --------------------------------------------------------------------------------------
#define LUA_SENSORIMGTOWORKIMG_COMMAND_PLUGIN "sensorImgToWorkImg"

const int inArgs_SENSORIMGTOWORKIMG[]={
    1,
    sim_script_arg_int32,0,
};

void LUA_SENSORIMGTOWORKIMG_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_SENSORIMGTOWORKIMG,inArgs_SENSORIMGTOWORKIMG[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int handle=getVisionSensorHandle(inData->at(0).int32Data[0],p->objectID);
        double* img=getVisionSensorImage(handle);
        if (img!=nullptr)
        {
            int res[2];
            simGetVisionSensorRes(handle,res);
            CVisionSensorData* imgData=visionContainer->getImageObject(handle);
            if (imgData==nullptr)
            {
                imgData=new CVisionSensorData();
                imgData->resolution[0]=res[0];
                imgData->resolution[1]=res[1];
                visionContainer->setImageObject(handle,p->scriptID,imgData);
            }
            if ( (imgData->resolution[0]!=res[0])||(imgData->resolution[1]!=res[1]) )
            {
                if (imgData->workImg!=nullptr)
                {
                    delete[] imgData->workImg;
                    imgData->workImg=new double[res[0]*res[1]*3];
                }
                if (imgData->buff1Img!=nullptr)
                {
                    delete[] imgData->buff1Img;
                    imgData->buff1Img=new double[res[0]*res[1]*3];
                }
                if (imgData->buff2Img!=nullptr)
                {
                    delete[] imgData->buff2Img;
                    imgData->buff2Img=new double[res[0]*res[1]*3];
                }
                imgData->resolution[0]=res[0];
                imgData->resolution[1]=res[1];
            }
            if (imgData->workImg==nullptr)
                imgData->workImg=new double[res[0]*res[1]*3];
            for (size_t i=0;i<res[0]*res[1]*3;i++)
                imgData->workImg[i]=img[i];
            simReleaseBuffer((char*)img);
        }
        else
            simSetLastError(nullptr,"Invalid handle.");
    }
    D.pushOutData(CScriptFunctionDataItem(false));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simVision.sensorDepthMapToWorkImg
// --------------------------------------------------------------------------------------
#define LUA_SENSORDEPTHMAPTOWORKIMG_COMMAND_PLUGIN "sensorDepthMapToWorkImg"

const int inArgs_SENSORDEPTHMAPTOWORKIMG[]={
    1,
    sim_script_arg_int32,0,
};

void LUA_SENSORDEPTHMAPTOWORKIMG_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_SENSORDEPTHMAPTOWORKIMG,inArgs_SENSORDEPTHMAPTOWORKIMG[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int handle=getVisionSensorHandle(inData->at(0).int32Data[0],p->objectID);
        double* img=getVisionSensorDepth(handle);
        if (img!=nullptr)
        {
            int res[2];
            simGetVisionSensorRes(handle,res);
            CVisionSensorData* imgData=visionContainer->getImageObject(handle);
            if (imgData==nullptr)
            {
                imgData=new CVisionSensorData();
                imgData->resolution[0]=res[0];
                imgData->resolution[1]=res[1];
                visionContainer->setImageObject(handle,p->scriptID,imgData);
            }
            if ( (imgData->resolution[0]!=res[0])||(imgData->resolution[1]!=res[1]) )
            {
                if (imgData->workImg!=nullptr)
                {
                    delete[] imgData->workImg;
                    imgData->workImg=new double[res[0]*res[1]*3];
                }
                if (imgData->buff1Img!=nullptr)
                {
                    delete[] imgData->buff1Img;
                    imgData->buff1Img=new double[res[0]*res[1]*3];
                }
                if (imgData->buff2Img!=nullptr)
                {
                    delete[] imgData->buff2Img;
                    imgData->buff2Img=new double[res[0]*res[1]*3];
                }
                imgData->resolution[0]=res[0];
                imgData->resolution[1]=res[1];
            }
            if (imgData->workImg==nullptr)
                imgData->workImg=new double[res[0]*res[1]*3];
            for (size_t i=0;i<res[0]*res[1];i++)
            {
                imgData->workImg[3*i+0]=img[i];
                imgData->workImg[3*i+1]=img[i];
                imgData->workImg[3*i+2]=img[i];
            }
            simReleaseBuffer((char*)img);
        }
        else
            simSetLastError(nullptr,"Invalid handle.");
    }
    D.pushOutData(CScriptFunctionDataItem(false));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simVision.workImgToSensorImg
// --------------------------------------------------------------------------------------
#define LUA_WORKIMGTOSENSORIMG_COMMAND_PLUGIN "workImgToSensorImg"

const int inArgs_WORKIMGTOSENSORIMG[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_bool,0,
};

void LUA_WORKIMGTOSENSORIMG_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_WORKIMGTOSENSORIMG,inArgs_WORKIMGTOSENSORIMG[0]-1,nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int handle=getVisionSensorHandle(inData->at(0).int32Data[0],p->objectID);
        CVisionSensorData* imgData=visionContainer->getImageObject(handle);
        bool removeImg=true;
        if ( (inData->size()>1)&&(inData->at(1).boolData.size()==1) )
            removeImg=inData->at(1).boolData[0];
        if (imgData!=nullptr)
        {
            int res[2];
            simGetVisionSensorRes(handle,res);
            if ( (imgData->resolution[0]==res[0])&&(imgData->resolution[1]==res[1]) )
                setVisionSensorImage(handle,imgData->workImg);
            else
                simSetLastError(nullptr,"Resolution mismatch.");
            if ( (imgData->buff1Img==nullptr)&&(imgData->buff2Img==nullptr)&&removeImg )
                visionContainer->removeImageObject(handle); // otherwise, we have to explicitely free the image data with simVision.releaseBuffers
        }
        else
            simSetLastError(nullptr,"Invalid handle or work image not initialized.");
    }
    D.pushOutData(CScriptFunctionDataItem(false));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simVision.workImgToSensorDepthMap
// --------------------------------------------------------------------------------------
#define LUA_WORKIMGTOSENSORDEPTHMAP_COMMAND_PLUGIN "workImgToSensorDepthMap"

const int inArgs_WORKIMGTOSENSORDEPTHMAP[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_bool,0,
};

void LUA_WORKIMGTOSENSORDEPTHMAP_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_WORKIMGTOSENSORDEPTHMAP,inArgs_WORKIMGTOSENSORDEPTHMAP[0]-1,nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int handle=getVisionSensorHandle(inData->at(0).int32Data[0],p->objectID);
        CVisionSensorData* imgData=visionContainer->getImageObject(handle);
        bool removeImg=true;
        if ( (inData->size()>1)&&(inData->at(1).boolData.size()==1) )
            removeImg=inData->at(1).boolData[0];
        if (imgData!=nullptr)
        {
            int res[2];
            simGetVisionSensorRes(handle,res);
            if ( (imgData->resolution[0]==res[0])&&(imgData->resolution[1]==res[1]) )
            {
                double* tmpDepthMap=new double[res[0]*res[1]];
                for (size_t i=0;i<res[0]*res[1];i++)
                    tmpDepthMap[i]=(imgData->workImg[3*i+0]+imgData->workImg[3*i+1]+imgData->workImg[3*i+2])/3.0f;
                setVisionSensorDepth(handle,tmpDepthMap);
                delete[] tmpDepthMap;
            }
            else
                simSetLastError(nullptr,"Resolution mismatch.");
            if ( (imgData->buff1Img==nullptr)&&(imgData->buff2Img==nullptr)&&removeImg )
                visionContainer->removeImageObject(handle); // otherwise, we have to explicitely free the image data with simVision.releaseBuffers
        }
        else
            simSetLastError(nullptr,"Invalid handle or work image not initialized.");
    }
    D.pushOutData(CScriptFunctionDataItem(false));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simVision.workImgToBuffer1
// --------------------------------------------------------------------------------------
#define LUA_WORKIMGTOBUFFER1_COMMAND_PLUGIN "workImgToBuffer1"

const int inArgs_WORKIMGTOBUFFER1[]={
    1,
    sim_script_arg_int32,0,
};

void LUA_WORKIMGTOBUFFER1_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_WORKIMGTOBUFFER1,inArgs_WORKIMGTOBUFFER1[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int handle=getVisionSensorHandle(inData->at(0).int32Data[0],p->objectID);
        CVisionSensorData* imgData=visionContainer->getImageObject(handle);
        if (imgData!=nullptr)
        {
            if (imgData->buff1Img==nullptr)
                imgData->buff1Img=new double[imgData->resolution[0]*imgData->resolution[1]*3];
            for (size_t i=0;i<imgData->resolution[0]*imgData->resolution[1]*3;i++)
                imgData->buff1Img[i]=imgData->workImg[i];
        }
        else
            simSetLastError(nullptr,"Invalid handle or work image not initialized.");
    }
    D.pushOutData(CScriptFunctionDataItem(false));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simVision.workImgToBuffer2
// --------------------------------------------------------------------------------------
#define LUA_WORKIMGTOBUFFER2_COMMAND_PLUGIN "workImgToBuffer2"

const int inArgs_WORKIMGTOBUFFER2[]={
    1,
    sim_script_arg_int32,0,
};

void LUA_WORKIMGTOBUFFER2_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_WORKIMGTOBUFFER2,inArgs_WORKIMGTOBUFFER2[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int handle=getVisionSensorHandle(inData->at(0).int32Data[0],p->objectID);
        CVisionSensorData* imgData=visionContainer->getImageObject(handle);
        if (imgData!=nullptr)
        {
            if (imgData->buff2Img==nullptr)
                imgData->buff2Img=new double[imgData->resolution[0]*imgData->resolution[1]*3];
            for (size_t i=0;i<imgData->resolution[0]*imgData->resolution[1]*3;i++)
                imgData->buff2Img[i]=imgData->workImg[i];
        }
        else
            simSetLastError(nullptr,"Invalid handle or work image not initialized.");
    }
    D.pushOutData(CScriptFunctionDataItem(false));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simVision.swapBuffers
// --------------------------------------------------------------------------------------
#define LUA_SWAPBUFFERS_COMMAND_PLUGIN "swapBuffers"

const int inArgs_SWAPBUFFERS[]={
    1,
    sim_script_arg_int32,0,
};

void LUA_SWAPBUFFERS_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_SWAPBUFFERS,inArgs_SWAPBUFFERS[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int handle=getVisionSensorHandle(inData->at(0).int32Data[0],p->objectID);
        CVisionSensorData* imgData=visionContainer->getImageObject(handle);
        if (imgData!=nullptr)
        {
            if (imgData->buff1Img==nullptr)
                imgData->buff1Img=new double[imgData->resolution[0]*imgData->resolution[1]*3];
            if (imgData->buff2Img==nullptr)
                imgData->buff2Img=new double[imgData->resolution[0]*imgData->resolution[1]*3];
            double* tmp=imgData->buff1Img;
            imgData->buff1Img=imgData->buff2Img;
            imgData->buff2Img=tmp;
        }
        else
            simSetLastError(nullptr,"Invalid handle or work image not initialized.");
    }
    D.pushOutData(CScriptFunctionDataItem(false));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simVision.buffer1ToWorkImg
// --------------------------------------------------------------------------------------
#define LUA_BUFFER1TOWORKIMG_COMMAND_PLUGIN "buffer1ToWorkImg"

const int inArgs_BUFFER1TOWORKIMG[]={
    1,
    sim_script_arg_int32,0,
};

void LUA_BUFFER1TOWORKIMG_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_BUFFER1TOWORKIMG,inArgs_BUFFER1TOWORKIMG[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int handle=getVisionSensorHandle(inData->at(0).int32Data[0],p->objectID);
        CVisionSensorData* imgData=visionContainer->getImageObject(handle);
        if (imgData!=nullptr)
        {
            if (imgData->buff1Img==nullptr)
                imgData->buff1Img=new double[imgData->resolution[0]*imgData->resolution[1]*3];
            for (size_t i=0;i<imgData->resolution[0]*imgData->resolution[1]*3;i++)
                imgData->workImg[i]=imgData->buff1Img[i];
        }
        else
            simSetLastError(nullptr,"Invalid handle or work image not initialized.");
    }
    D.pushOutData(CScriptFunctionDataItem(false));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simVision.buffer2ToWorkImg
// --------------------------------------------------------------------------------------
#define LUA_BUFFER2TOWORKIMG_COMMAND_PLUGIN "buffer2ToWorkImg"

const int inArgs_BUFFER2TOWORKIMG[]={
    1,
    sim_script_arg_int32,0,
};

void LUA_BUFFER2TOWORKIMG_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_BUFFER2TOWORKIMG,inArgs_BUFFER2TOWORKIMG[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int handle=getVisionSensorHandle(inData->at(0).int32Data[0],p->objectID);
        CVisionSensorData* imgData=visionContainer->getImageObject(handle);
        if (imgData!=nullptr)
        {
            if (imgData->buff2Img==nullptr)
                imgData->buff2Img=new double[imgData->resolution[0]*imgData->resolution[1]*3];
            for (size_t i=0;i<imgData->resolution[0]*imgData->resolution[1]*3;i++)
                imgData->workImg[i]=imgData->buff2Img[i];
        }
        else
            simSetLastError(nullptr,"Invalid handle or work image not initialized.");
    }
    D.pushOutData(CScriptFunctionDataItem(false));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simVision.swapWorkImgWithBuffer1
// --------------------------------------------------------------------------------------
#define LUA_SWAPWORKIMGWITHBUFFER1_COMMAND_PLUGIN "swapWorkImgWithBuffer1"

const int inArgs_SWAPWORKIMGWITHBUFFER1[]={
    1,
    sim_script_arg_int32,0,
};

void LUA_SWAPWORKIMGWITHBUFFER1_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_SWAPWORKIMGWITHBUFFER1,inArgs_SWAPWORKIMGWITHBUFFER1[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int handle=getVisionSensorHandle(inData->at(0).int32Data[0],p->objectID);
        CVisionSensorData* imgData=visionContainer->getImageObject(handle);
        if (imgData!=nullptr)
        {
            if (imgData->buff1Img==nullptr)
                imgData->buff1Img=new double[imgData->resolution[0]*imgData->resolution[1]*3];
            for (size_t i=0;i<imgData->resolution[0]*imgData->resolution[1]*3;i++)
            {
                double tmp=imgData->workImg[i];
                imgData->workImg[i]=imgData->buff1Img[i];
                imgData->buff1Img[i]=tmp;
            }
        }
        else
            simSetLastError(nullptr,"Invalid handle or work image not initialized.");
    }
    D.pushOutData(CScriptFunctionDataItem(false));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simVision.addWorkImgToBuffer1
// --------------------------------------------------------------------------------------
#define LUA_ADDWORKIMGTOBUFFER1_COMMAND_PLUGIN "addWorkImgToBuffer1"

const int inArgs_ADDWORKIMGTOBUFFER1[]={
    1,
    sim_script_arg_int32,0,
};

void LUA_ADDWORKIMGTOBUFFER1_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_ADDWORKIMGTOBUFFER1,inArgs_ADDWORKIMGTOBUFFER1[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int handle=getVisionSensorHandle(inData->at(0).int32Data[0],p->objectID);
        CVisionSensorData* imgData=visionContainer->getImageObject(handle);
        if (imgData!=nullptr)
        {
            if (imgData->buff1Img==nullptr)
                imgData->buff1Img=new double[imgData->resolution[0]*imgData->resolution[1]*3];
            for (size_t i=0;i<imgData->resolution[0]*imgData->resolution[1]*3;i++)
            {
                imgData->buff1Img[i]+=imgData->workImg[i];
                if (imgData->buff1Img[i]>1.0f)
                    imgData->buff1Img[i]=1.0f;
            }
        }
        else
            simSetLastError(nullptr,"Invalid handle or work image not initialized.");
    }
    D.pushOutData(CScriptFunctionDataItem(false));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simVision.subtractWorkImgFromBuffer1
// --------------------------------------------------------------------------------------
#define LUA_SUBTRACTWORKIMGFROMBUFFER1_COMMAND_PLUGIN "subtractWorkImgFromBuffer1"

const int inArgs_SUBTRACTWORKIMGFROMBUFFER1[]={
    1,
    sim_script_arg_int32,0,
};

void LUA_SUBTRACTWORKIMGFROMBUFFER1_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_SUBTRACTWORKIMGFROMBUFFER1,inArgs_SUBTRACTWORKIMGFROMBUFFER1[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int handle=getVisionSensorHandle(inData->at(0).int32Data[0],p->objectID);
        CVisionSensorData* imgData=visionContainer->getImageObject(handle);
        if (imgData!=nullptr)
        {
            if (imgData->buff1Img==nullptr)
                imgData->buff1Img=new double[imgData->resolution[0]*imgData->resolution[1]*3];
            for (size_t i=0;i<imgData->resolution[0]*imgData->resolution[1]*3;i++)
            {
                imgData->buff1Img[i]-=imgData->workImg[i];
                if (imgData->buff1Img[i]<0.0f)
                    imgData->buff1Img[i]=0.0f;
            }
        }
        else
            simSetLastError(nullptr,"Invalid handle or work image not initialized.");
    }
    D.pushOutData(CScriptFunctionDataItem(false));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simVision.addBuffer1ToWorkImg
// --------------------------------------------------------------------------------------
#define LUA_ADDBUFFER1TOWORKIMG_COMMAND_PLUGIN "addBuffer1ToWorkImg"

const int inArgs_ADDBUFFER1TOWORKIMG[]={
    1,
    sim_script_arg_int32,0,
};

void LUA_ADDBUFFER1TOWORKIMG_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_ADDBUFFER1TOWORKIMG,inArgs_ADDBUFFER1TOWORKIMG[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int handle=getVisionSensorHandle(inData->at(0).int32Data[0],p->objectID);
        CVisionSensorData* imgData=visionContainer->getImageObject(handle);
        if (imgData!=nullptr)
        {
            if (imgData->buff1Img==nullptr)
                imgData->buff1Img=new double[imgData->resolution[0]*imgData->resolution[1]*3];
            for (size_t i=0;i<imgData->resolution[0]*imgData->resolution[1]*3;i++)
            {
                imgData->workImg[i]+=imgData->buff1Img[i];
                if (imgData->workImg[i]>1.0f)
                    imgData->workImg[i]=1.0f;
            }
        }
        else
            simSetLastError(nullptr,"Invalid handle or work image not initialized.");
    }
    D.pushOutData(CScriptFunctionDataItem(false));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simVision.subtractBuffer1FromWorkImg
// --------------------------------------------------------------------------------------
#define LUA_SUBTRACTBUFFER1FROMWORKIMG_COMMAND_PLUGIN "subtractBuffer1FromWorkImg"

const int inArgs_SUBTRACTBUFFER1FROMWORKIMG[]={
    1,
    sim_script_arg_int32,0,
};

void LUA_SUBTRACTBUFFER1FROMWORKIMG_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_SUBTRACTBUFFER1FROMWORKIMG,inArgs_SUBTRACTBUFFER1FROMWORKIMG[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int handle=getVisionSensorHandle(inData->at(0).int32Data[0],p->objectID);
        CVisionSensorData* imgData=visionContainer->getImageObject(handle);
        if (imgData!=nullptr)
        {
            if (imgData->buff1Img==nullptr)
                imgData->buff1Img=new double[imgData->resolution[0]*imgData->resolution[1]*3];
            for (size_t i=0;i<imgData->resolution[0]*imgData->resolution[1]*3;i++)
            {
                imgData->workImg[i]-=imgData->buff1Img[i];
                if (imgData->workImg[i]<0.0f)
                    imgData->workImg[i]=0.0f;
            }
        }
        else
            simSetLastError(nullptr,"Invalid handle or work image not initialized.");
    }
    D.pushOutData(CScriptFunctionDataItem(false));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simVision.multiplyWorkImgWithBuffer1
// --------------------------------------------------------------------------------------
#define LUA_MULTIPLYWORKIMGWITHBUFFER1_COMMAND_PLUGIN "multiplyWorkImgWithBuffer1"

const int inArgs_MULTIPLYWORKIMGWITHBUFFER1[]={
    1,
    sim_script_arg_int32,0,
};

void LUA_MULTIPLYWORKIMGWITHBUFFER1_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_MULTIPLYWORKIMGWITHBUFFER1,inArgs_MULTIPLYWORKIMGWITHBUFFER1[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int handle=getVisionSensorHandle(inData->at(0).int32Data[0],p->objectID);
        CVisionSensorData* imgData=visionContainer->getImageObject(handle);
        if (imgData!=nullptr)
        {
            if (imgData->buff1Img==nullptr)
                imgData->buff1Img=new double[imgData->resolution[0]*imgData->resolution[1]*3];
            for (size_t i=0;i<imgData->resolution[0]*imgData->resolution[1]*3;i++)
            {
                imgData->workImg[i]*=imgData->buff1Img[i];
                if (imgData->workImg[i]>1.0f)
                    imgData->workImg[i]=1.0f;
            }
        }
        else
            simSetLastError(nullptr,"Invalid handle or work image not initialized.");
    }
    D.pushOutData(CScriptFunctionDataItem(false));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simVision.horizontalFlipWorkImg
// --------------------------------------------------------------------------------------
#define LUA_HORIZONTALFLIPWORKIMG_COMMAND_PLUGIN "horizontalFlipWorkImg"

const int inArgs_HORIZONTALFLIPWORKIMG[]={
    1,
    sim_script_arg_int32,0,
};

void LUA_HORIZONTALFLIPWORKIMG_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_HORIZONTALFLIPWORKIMG,inArgs_HORIZONTALFLIPWORKIMG[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int handle=getVisionSensorHandle(inData->at(0).int32Data[0],p->objectID);
        CVisionSensorData* imgData=visionContainer->getImageObject(handle);
        if (imgData!=nullptr)
        {
            double tmp;
            int sizeX=imgData->resolution[0];
            int sizeY=imgData->resolution[1];
            for (int i=0;i<sizeX/2;i++)
            {
                for (int j=0;j<sizeY;j++)
                {
                    for (int k=0;k<3;k++)
                    {
                        tmp=imgData->workImg[3*(i+j*sizeX)+k];
                        imgData->workImg[3*(i+j*sizeX)+k]=imgData->workImg[3*((sizeX-1-i)+j*sizeX)+k];
                        imgData->workImg[3*((sizeX-1-i)+j*sizeX)+k]=tmp;
                    }
                }
            }
        }
        else
            simSetLastError(nullptr,"Invalid handle or work image not initialized.");
    }
    D.pushOutData(CScriptFunctionDataItem(false));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simVision.verticalFlipWorkImg
// --------------------------------------------------------------------------------------
#define LUA_VERTICALFLIPWORKIMG_COMMAND_PLUGIN "verticalFlipWorkImg"

const int inArgs_VERTICALFLIPWORKIMG[]={
    1,
    sim_script_arg_int32,0,
};

void LUA_VERTICALFLIPWORKIMG_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_VERTICALFLIPWORKIMG,inArgs_VERTICALFLIPWORKIMG[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int handle=getVisionSensorHandle(inData->at(0).int32Data[0],p->objectID);
        CVisionSensorData* imgData=visionContainer->getImageObject(handle);
        if (imgData!=nullptr)
        {
            double tmp;
            int sizeX=imgData->resolution[0];
            int sizeY=imgData->resolution[1];
            for (int i=0;i<sizeX;i++)
            {
                for (int j=0;j<sizeY/2;j++)
                {
                    for (int k=0;k<3;k++)
                    {
                        tmp=imgData->workImg[3*(i+j*sizeX)+k];
                        imgData->workImg[3*(i+j*sizeX)+k]=imgData->workImg[3*(i+(sizeY-1-j)*sizeX)+k];
                        imgData->workImg[3*(i+(sizeY-1-j)*sizeX)+k]=tmp;
                    }
                }
            }
        }
        else
            simSetLastError(nullptr,"Invalid handle or work image not initialized.");
    }
    D.pushOutData(CScriptFunctionDataItem(false));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simVision.uniformImgToWorkImg
// --------------------------------------------------------------------------------------
#define LUA_UNIFORMIMGTOWORKIMG_COMMAND_PLUGIN "uniformImgToWorkImg"

const int inArgs_UNIFORMIMGTOWORKIMG[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_double|sim_lua_arg_table,3,
};

void LUA_UNIFORMIMGTOWORKIMG_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_UNIFORMIMGTOWORKIMG,inArgs_UNIFORMIMGTOWORKIMG[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int handle=getVisionSensorHandle(inData->at(0).int32Data[0],p->objectID);
        double* p=&(inData->at(1).doubleData[0]);
        CVisionSensorData* imgData=visionContainer->getImageObject(handle);
        if (imgData!=nullptr)
        {
            for (int i=0;i<imgData->resolution[0]*imgData->resolution[1];i++)
            {
                imgData->workImg[3*i+0]=p[0];
                imgData->workImg[3*i+1]=p[1];
                imgData->workImg[3*i+2]=p[2];
            }
        }
        else
            simSetLastError(nullptr,"Invalid handle or work image not initialized.");
    }
    D.pushOutData(CScriptFunctionDataItem(false));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simVision.normalizeWorkImg
// --------------------------------------------------------------------------------------
#define LUA_NORMALIZEWORKIMG_COMMAND_PLUGIN "normalizeWorkImg"

const int inArgs_NORMALIZEWORKIMG[]={
    1,
    sim_script_arg_int32,0,
};

void LUA_NORMALIZEWORKIMG_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_NORMALIZEWORKIMG,inArgs_NORMALIZEWORKIMG[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int handle=getVisionSensorHandle(inData->at(0).int32Data[0],p->objectID);
        CVisionSensorData* imgData=visionContainer->getImageObject(handle);
        if (imgData!=nullptr)
        {
            int s=imgData->resolution[0]*imgData->resolution[1]*3;
            double maxCol=0.0f;
            double minCol=1.0f;
            for (int i=0;i<s;i++)
            {
                if (imgData->workImg[i]>maxCol)
                    maxCol=imgData->workImg[i];
                if (imgData->workImg[i]<minCol)
                    minCol=imgData->workImg[i];
            }
            if (maxCol-minCol!=0.0f)
            {
                double mul=1.0f/(maxCol-minCol);
                for (int i=0;i<s;i++)
                    imgData->workImg[i]=(imgData->workImg[i]-minCol)*mul;
            }
        }
        else
            simSetLastError(nullptr,"Invalid handle or work image not initialized.");
    }
    D.pushOutData(CScriptFunctionDataItem(false));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simVision.colorSegmentationOnWorkImg
// --------------------------------------------------------------------------------------
#define LUA_COLORSEGMENTATIONONWORKIMG_COMMAND_PLUGIN "colorSegmentationOnWorkImg"

const int inArgs_COLORSEGMENTATIONONWORKIMG[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_double,0,
};

void LUA_COLORSEGMENTATIONONWORKIMG_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_COLORSEGMENTATIONONWORKIMG,inArgs_COLORSEGMENTATIONONWORKIMG[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int handle=getVisionSensorHandle(inData->at(0).int32Data[0],p->objectID);
        double p=inData->at(1).doubleData[0];
        CVisionSensorData* imgData=visionContainer->getImageObject(handle);
        if (imgData!=nullptr)
        {
            int s=imgData->resolution[0]*imgData->resolution[1];
            std::vector<double> goodColors;
            double squaredDistance=p*p;
            for (int i=0;i<s;i++)
            {
                bool found=false;
                for (size_t j=0;j<goodColors.size()/3;j++)
                {
                    double r=imgData->workImg[3*i+0]-goodColors[3*j+0];
                    double g=imgData->workImg[3*i+1]-goodColors[3*j+1];
                    double b=imgData->workImg[3*i+2]-goodColors[3*j+2];
                    double d=r*r+g*g+b*b;
                    if (d<squaredDistance)
                    {
                        found=true;
                        imgData->workImg[3*i+0]=goodColors[3*j+0];
                        imgData->workImg[3*i+1]=goodColors[3*j+1];
                        imgData->workImg[3*i+2]=goodColors[3*j+2];
                        break;
                    }
                }
                if (!found)
                {
                    goodColors.push_back(imgData->workImg[3*i+0]);
                    goodColors.push_back(imgData->workImg[3*i+1]);
                    goodColors.push_back(imgData->workImg[3*i+2]);
                }
            }
        }
        else
            simSetLastError(nullptr,"Invalid handle or work image not initialized.");
    }
    D.pushOutData(CScriptFunctionDataItem(false));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

void colorFromIntensity(double intensity,int colorTable,double col[3])
{
    if (intensity>1.0f)
        intensity=1.0f;
    if (intensity<0.0f)
        intensity=0.0f;
    const double c[12]={0.0f,0.0f,0.0f,0.0f,0.0f,1.0f,1.0f,0.0f,0.0f,1.0f,1.0f,0.0f};
    int d=int(intensity*3);
    if (d>2)
        d=2;
    double r=(intensity-double(d)/3.0f)*3.0f;
    col[0]=c[3*d+0]*(1.0f-r)+c[3*(d+1)+0]*r;
    col[1]=c[3*d+1]*(1.0f-r)+c[3*(d+1)+1]*r;
    col[2]=c[3*d+2]*(1.0f-r)+c[3*(d+1)+2]*r;
}

// --------------------------------------------------------------------------------------
// simVision.intensityScaleOnWorkImg
// --------------------------------------------------------------------------------------
#define LUA_INTENSITYSCALEONWORKIMG_COMMAND_PLUGIN "intensityScaleOnWorkImg"

const int inArgs_INTENSITYSCALEONWORKIMG[]={
    4,
    sim_script_arg_int32,0,
    sim_script_arg_double,0,
    sim_script_arg_double,0,
    sim_script_arg_bool,0,
};

void LUA_INTENSITYSCALEONWORKIMG_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_INTENSITYSCALEONWORKIMG,inArgs_INTENSITYSCALEONWORKIMG[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int handle=getVisionSensorHandle(inData->at(0).int32Data[0],p->objectID);
        double start=inData->at(1).doubleData[0];
        double end=inData->at(2).doubleData[0];
        bool greyScale=inData->at(3).boolData[0];
        CVisionSensorData* imgData=visionContainer->getImageObject(handle);
        if (imgData!=nullptr)
        {
            double b=start;
            double a=end-b;
            // intensity first transformed like: intensity=a*intensity+b
            int s=imgData->resolution[0]*imgData->resolution[1];
            double intensity;
            double col[3];
            for (int i=0;i<s;i++)
            {
                intensity=(imgData->workImg[3*i+0]+imgData->workImg[3*i+1]+imgData->workImg[3*i+2])/3.0f;
                intensity=a*intensity+b;
                if (greyScale)
                { // grey scale
                    imgData->workImg[3*i+0]=intensity;
                    imgData->workImg[3*i+1]=intensity;
                    imgData->workImg[3*i+2]=intensity;
                }
                else
                { // intensity scale
                    colorFromIntensity(intensity,0,col);
                    imgData->workImg[3*i+0]=col[0];
                    imgData->workImg[3*i+1]=col[1];
                    imgData->workImg[3*i+2]=col[2];
                }
            }
        }
        else
            simSetLastError(nullptr,"Invalid handle or work image not initialized.");
    }
    D.pushOutData(CScriptFunctionDataItem(false));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

void rgbToHsl(double rgb[3],double hsl[3])
{
    double r=rgb[0];
    double g=rgb[1];
    double b=rgb[2];
    double h,s,l,delta;
    double cmax=std::max<double>(r,std::max<double>(g,b));
    double cmin=std::min<double>(r,std::min<double>(g,b));
    l=(cmax+cmin)/2.0f;
    if (cmax==cmin)
    {
        s=0.0f;
        h=0.0f;
    }
    else
    {
        if(l<0.5f)
            s=(cmax-cmin)/(cmax+cmin);
        else
            s=(cmax-cmin)/(2.0f-cmax-cmin);
        delta=cmax-cmin;
        if (r==cmax)
            h=(g-b)/delta;
        else
            if (g==cmax)
                h=2.0f+(b-r)/delta;
            else
                h=4.0f+(r-g)/delta;
        h=h/6.0f;
        if (h<0.0f)
            h=h+1.0f;
    }
    hsl[0]=h;
    hsl[1]=s;
    hsl[2]=l;
}

double hueToRgb(double m1,double m2,double h)
{
    if (h<0.0f)
        h=h+1.0f;
    if (h>1.0f)
        h=h-1.0f;
    if (6.0f*h<1.0f)
        return(m1+(m2-m1)*h*6.0f);
    if (2.0f*h<1.0f)
        return(m2);
    if (3.0f*h<2.0f)
        return(m1+(m2-m1)*((2.0f/3.0f)-h)*6.0f);
    return(m1);
}

void hslToRgb(double hsl[3],double rgb[3])
{
    double h=hsl[0];
    double s=hsl[1];
    double l=hsl[2];
    double m1,m2;

    if (s==0.0f)
    {
        rgb[0]=l;
        rgb[1]=l;
        rgb[2]=l;
    }
    else
    {
        if (l<=0.5f)
            m2=l*(1.0f+s);
        else
            m2=l+s-l*s;
        m1=2.0f*l-m2;
        rgb[0]=hueToRgb(m1,m2,h+1.0f/3.0f);
        rgb[1]=hueToRgb(m1,m2,h);
        rgb[2]=hueToRgb(m1,m2,h-1.0f/3.0f);
    }
}


// --------------------------------------------------------------------------------------
// simVision.selectiveColorOnWorkImg
// --------------------------------------------------------------------------------------
#define LUA_SELECTIVECOLORONONWORKIMG_COMMAND_PLUGIN "selectiveColorOnWorkImg"

const int inArgs_SELECTIVECOLORONONWORKIMG[]={
    6,
    sim_script_arg_int32,0,
    sim_script_arg_double|sim_lua_arg_table,3,
    sim_script_arg_double|sim_lua_arg_table,3,
    sim_script_arg_bool,0,
    sim_script_arg_bool,0,
    sim_script_arg_bool,0,
};

void LUA_SELECTIVECOLORONONWORKIMG_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_SELECTIVECOLORONONWORKIMG,inArgs_SELECTIVECOLORONONWORKIMG[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int handle=getVisionSensorHandle(inData->at(0).int32Data[0],p->objectID);
        double* pCol=&(inData->at(1).doubleData)[0];
        double* pTol=&(inData->at(2).doubleData)[0];
        bool inRgbDim=inData->at(3).boolData[0];
        bool keep=inData->at(4).boolData[0];
        bool toBuffer1=inData->at(5).boolData[0];
        CVisionSensorData* imgData=visionContainer->getImageObject(handle);
        if (imgData!=nullptr)
        {
            if (toBuffer1)
            {
                if (imgData->buff1Img==nullptr)
                    imgData->buff1Img=new double[imgData->resolution[0]*imgData->resolution[1]*3];
            }
            int s=imgData->resolution[0]*imgData->resolution[1];
            double col[3];
            double rgb[3];
            double lowTol[3]={pCol[0]-pTol[0],pCol[1]-pTol[1],pCol[2]-pTol[2]};
            double upTol[3]={pCol[0]+pTol[0],pCol[1]+pTol[1],pCol[2]+pTol[2]};
            double lowTolUpHue=1.0f+lowTol[0];
            double upTolLowHue=upTol[0]-1.0f;
            for (int i=0;i<s;i++)
            {
                if (inRgbDim)
                { // rgb dimension
                    col[0]=imgData->workImg[3*i+0];
                    col[1]=imgData->workImg[3*i+1];
                    col[2]=imgData->workImg[3*i+2];
                }
                else
                { // hsl dimension
                    rgb[0]=imgData->workImg[3*i+0];
                    rgb[1]=imgData->workImg[3*i+1];
                    rgb[2]=imgData->workImg[3*i+2];
                    rgbToHsl(rgb,col);
                }
                bool outOfTol;
                if (inRgbDim)
                { // rgb dimension
                    outOfTol=((col[0]>upTol[0])||
                        (col[0]<lowTol[0])||
                        (col[1]>upTol[1])||
                        (col[1]<lowTol[1])||
                        (col[2]>upTol[2])||
                        (col[2]<lowTol[2]));
                }
                else
                { // hsl dimension
                    outOfTol=((col[1]>upTol[1])||
                        (col[1]<lowTol[1])||
                        (col[2]>upTol[2])||
                        (col[2]<lowTol[2]));
                    if ( (!outOfTol)&&((upTol[0]-lowTol[0])<1.0f) )
                    { // Check the Hue value (special handling):
                        outOfTol=( (col[0]>upTol[0])&&(col[0]<lowTolUpHue) )||( (col[0]<lowTol[0])&&(col[0]>upTolLowHue) );
                    }
                }

                if (outOfTol)
                {
                    if (keep)
                    { // color not within tolerance, we remove it
                        if (toBuffer1)
                        { // we copy the removed part to buffer 1
                            imgData->buff1Img[3*i+0]=imgData->workImg[3*i+0];
                            imgData->buff1Img[3*i+1]=imgData->workImg[3*i+1];
                            imgData->buff1Img[3*i+2]=imgData->workImg[3*i+2];
                        }
                        imgData->workImg[3*i+0]=0.0f;
                        imgData->workImg[3*i+1]=0.0f;
                        imgData->workImg[3*i+2]=0.0f;
                    }
                    else
                    { // color within tolerance
                        if (toBuffer1)
                        { // we mark as black in buffer 1 the parts not removed
                            imgData->buff1Img[3*i+0]=0.0f;
                            imgData->buff1Img[3*i+1]=0.0f;
                            imgData->buff1Img[3*i+2]=0.0f;
                        }
                    }
                }
                else
                {
                    if (!keep)
                    { // color not within tolerance, we remove it
                        if (toBuffer1)
                        { // we copy the removed part to buffer 1
                            imgData->buff1Img[3*i+0]=imgData->workImg[3*i+0];
                            imgData->buff1Img[3*i+1]=imgData->workImg[3*i+1];
                            imgData->buff1Img[3*i+2]=imgData->workImg[3*i+2];
                        }
                        imgData->workImg[3*i+0]=0.0f;
                        imgData->workImg[3*i+1]=0.0f;
                        imgData->workImg[3*i+2]=0.0f;
                    }
                    else
                    { // color within tolerance
                        if (toBuffer1)
                        { // we mark as black in buffer 1 the parts not removed
                            imgData->buff1Img[3*i+0]=0.0f;
                            imgData->buff1Img[3*i+1]=0.0f;
                            imgData->buff1Img[3*i+2]=0.0f;
                        }
                    }
                }
            }
        }
        else
            simSetLastError(nullptr,"Invalid handle or work image not initialized.");
    }
    D.pushOutData(CScriptFunctionDataItem(false));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simVision.scaleAndOffsetWorkImg
// --------------------------------------------------------------------------------------
#define LUA_SCALEANDOFFSETWORKIMG_COMMAND_PLUGIN "scaleAndOffsetWorkImg"

const int inArgs_SCALEANDOFFSETWORKIMG[]={
    5,
    sim_script_arg_int32,0,
    sim_script_arg_double|sim_lua_arg_table,3,
    sim_script_arg_double|sim_lua_arg_table,3,
    sim_script_arg_double|sim_lua_arg_table,3,
    sim_script_arg_bool,0,
};

void LUA_SCALEANDOFFSETWORKIMG_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_SCALEANDOFFSETWORKIMG,inArgs_SCALEANDOFFSETWORKIMG[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int handle=getVisionSensorHandle(inData->at(0).int32Data[0],p->objectID);
        double* pCol=&(inData->at(1).doubleData)[0];
        double* pScale=&(inData->at(2).doubleData)[0];
        double* pOff=&(inData->at(3).doubleData)[0];
        bool inRgbDim=inData->at(4).boolData[0];
        CVisionSensorData* imgData=visionContainer->getImageObject(handle);
        if (imgData!=nullptr)
        {
            int s=imgData->resolution[0]*imgData->resolution[1];
            double col[3];
            double rgb[3];
            for (int i=0;i<s;i++)
            {
                if (inRgbDim)
                { // rgb dimension
                    col[0]=imgData->workImg[3*i+0];
                    col[1]=imgData->workImg[3*i+1];
                    col[2]=imgData->workImg[3*i+2];
                }
                else
                { // hsl dimension
                    rgb[0]=imgData->workImg[3*i+0];
                    rgb[1]=imgData->workImg[3*i+1];
                    rgb[2]=imgData->workImg[3*i+2];
                    rgbToHsl(rgb,col);
                }
                col[0]=pOff[0]+(col[0]+pCol[0])*pScale[0];
                col[1]=pOff[1]+(col[1]+pCol[1])*pScale[1];
                col[2]=pOff[2]+(col[2]+pCol[2])*pScale[2];
                if (col[0]<0.0f)
                    col[0]=0.0f;
                if (col[0]>1.0f)
                    col[0]=1.0f;
                if (col[1]<0.0f)
                    col[1]=0.0f;
                if (col[1]>1.0f)
                    col[1]=1.0f;
                if (col[2]<0.0f)
                    col[2]=0.0f;
                if (col[2]>1.0f)
                    col[2]=1.0f;
                if (inRgbDim)
                { // rgb dimension
                    imgData->workImg[3*i+0]=col[0];
                    imgData->workImg[3*i+1]=col[1];
                    imgData->workImg[3*i+2]=col[2];
                }
                else
                { // hsl dimension
                    rgb[0]=imgData->workImg[3*i+0];
                    rgb[1]=imgData->workImg[3*i+1];
                    rgb[2]=imgData->workImg[3*i+2];
                    hslToRgb(col,imgData->workImg+3*i+0);
                }
            }
        }
        else
            simSetLastError(nullptr,"Invalid handle or work image not initialized.");
    }
    D.pushOutData(CScriptFunctionDataItem(false));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

double angleMinusAlpha(double angle,double alpha)
{
    double sinAngle0=sin(double(angle));
    double sinAngle1=sin(double(alpha));
    double cosAngle0=cos(double(angle));
    double cosAngle1=cos(double(alpha));
    double sin_da=sinAngle0*cosAngle1-cosAngle0*sinAngle1;
    double cos_da=cosAngle0*cosAngle1+sinAngle0*sinAngle1;
    double angle_da=atan2(sin_da,cos_da);
    return(double(angle_da));
}

void drawLine(CVisionSensorData* imgData,const double col[3],int x0,int y0,int x1,int y1)
{
    int sizeX=imgData->resolution[0];
    int sizeY=imgData->resolution[1];
    int x=x0;
    int y=y0;
    int dx=x1-x0;
    int dy=y1-y0;
    int incX=1;
    int incY=1;
    if (dx<0)
        incX=-1;
    if (dy<0)
        incY=-1;
    if ( abs(dx)>=abs(dy) )
    {
        int p=abs(dx);
        while(x!=x1)
        {
            p-=abs(dy);
            if ( (x>=0)&&(x<sizeX)&&(y>=0)&&(y<sizeY) )
            {
                imgData->workImg[3*(x+y*sizeX)+0]=col[0];
                imgData->workImg[3*(x+y*sizeX)+1]=col[1];
                imgData->workImg[3*(x+y*sizeX)+2]=col[2];
            }
            if (p<0)
            {
                y+=incY;
                p+=abs(dx);
            }
            x+=incX;
        }
    }
    else
    {
        int p=abs(dy);
        while(y!=y1)
        {
            p-=abs(dx);
            if ( (x>=0)&&(x<sizeX)&&(y>=0)&&(y<sizeY) )
            {
                imgData->workImg[3*(x+y*sizeX)+0]=col[0];
                imgData->workImg[3*(x+y*sizeX)+1]=col[1];
                imgData->workImg[3*(x+y*sizeX)+2]=col[2];
            }
            if (p<0)
            {
                x+=incX;
                p+=abs(dy);
            }
            y+=incY;
        }
    }
}

void drawLines(CVisionSensorData* imgData,const std::vector<int>& overlayLines,const double col[3])
{
    for (size_t i=0;i<overlayLines.size()/4;i++)
        drawLine(imgData,col,overlayLines[4*i+0],overlayLines[4*i+1],overlayLines[4*i+2],overlayLines[4*i+3]);
}

// --------------------------------------------------------------------------------------
// simVision.binaryWorkImg
// --------------------------------------------------------------------------------------
#define LUA_BINARYWORKIMG_COMMAND_PLUGIN "binaryWorkImg"

const int inArgs_BINARYWORKIMG[]={
    13,
    sim_script_arg_int32,0,
    sim_script_arg_double,0,
    sim_script_arg_double,0,
    sim_script_arg_double,0,
    sim_script_arg_double,0,
    sim_script_arg_double,0,
    sim_script_arg_double,0,
    sim_script_arg_double,0,
    sim_script_arg_double,0,
    sim_script_arg_double,0,
    sim_script_arg_double,0,
    sim_script_arg_bool,0,
    sim_script_arg_double|sim_script_arg_table,3,
};

void LUA_BINARYWORKIMG_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    bool retVal=false;
    std::vector<float> returnData;
    if (D.readDataFromStack(p->stackID,inArgs_BINARYWORKIMG,inArgs_BINARYWORKIMG[0]-1,nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int handle=getVisionSensorHandle(inData->at(0).int32Data[0],p->objectID);
        double threshold=inData->at(1).doubleData[0];
        double oneProp=inData->at(2).doubleData[0];
        double oneTol=inData->at(3).doubleData[0];
        double xCenter=inData->at(4).doubleData[0];
        double xCenterTol=inData->at(5).doubleData[0];
        double yCenter=inData->at(6).doubleData[0];
        double yCenterTol=inData->at(7).doubleData[0];
        double orient=inData->at(8).doubleData[0];
        double orientTol=inData->at(9).doubleData[0];
        double roundV=inData->at(10).doubleData[0];
        bool triggerEnabled=inData->at(11).boolData[0];
        CVisionSensorData* imgData=visionContainer->getImageObject(handle);
        if (imgData!=nullptr)
        {
            double col[3]={1.0f,0.0f,1.0f};
            bool overlay=false;
            if ( (inData->size()>=13)&&(inData->at(12).doubleData.size()==3) )
            {
                col[0]=inData->at(12).doubleData[0];
                col[1]=inData->at(12).doubleData[1];
                col[2]=inData->at(12).doubleData[2];
                overlay=true;
            }
            std::vector<int> overlayLines;
            int sizeX=imgData->resolution[0];
            int sizeY=imgData->resolution[1];
            double area=0.0f;
            double proportion=0.0f;
            double cmx=0.0f;
            double cmy=0.0f;
            double angle=0.0f;
            double roundness=1.0f;
            for (int i=0;i<sizeX;i++)
            {
                for (int j=0;j<sizeY;j++)
                {
                    double intensity=(imgData->workImg[3*(i+j*sizeX)+0]+imgData->workImg[3*(i+j*sizeX)+1]+imgData->workImg[3*(i+j*sizeX)+2])/3.0f;
                    if (intensity>=threshold)
                    { // Binary 1
                        imgData->workImg[3*(i+j*sizeX)+0]=1.0f;
                        imgData->workImg[3*(i+j*sizeX)+1]=1.0f;
                        imgData->workImg[3*(i+j*sizeX)+2]=1.0f;
                        area+=1.0f;
                        cmx+=double(i);
                        cmy+=double(j);
                    }
                    else
                    { // Binary 0
                        imgData->workImg[3*(i+j*sizeX)+0]=0.0f;
                        imgData->workImg[3*(i+j*sizeX)+1]=0.0f;
                        imgData->workImg[3*(i+j*sizeX)+2]=0.0f;
                    }
                }
            }
            proportion=area/double(sizeX*sizeY);
            if (area!=0.0f)
            {
                cmx/=area;
                cmy/=area;

                double a=0.0f;
                double b=0.0f;
                double c=0.0f;
                double tmpX,tmpY;
                for (int i=0;i<sizeX;i++)
                {
                    for (int j=0;j<sizeY;j++)
                    {
                        if (imgData->workImg[3*(i+j*sizeX)+0]!=0.0f)
                        { // Binary 1
                            tmpX=double(i)-cmx;
                            tmpY=double(j)-cmy;
                            a+=tmpX*tmpX;
                            b+=tmpX*tmpY;
                            c+=tmpY*tmpY;
                        }
                    }
                }
                b*=2.0f;
                if ((b!=0.0f)||(a!=c))
                {
                    double denom=sqrt(b*b+(a-c)*(a-c));
                    double sin2ThetaMax=-b/denom;
                    double sin2ThetaMin=b/denom;
                    double cos2ThetaMax=-(a-c)/denom;
                    double cos2ThetaMin=(a-c)/denom;
                    double iMax=0.5f*(c+a)-0.5f*(a-c)*cos2ThetaMax-0.5f*b*sin2ThetaMax;
                    double iMin=0.5f*(c+a)-0.5f*(a-c)*cos2ThetaMin-0.5f*b*sin2ThetaMin;
                    roundness=iMin/iMax;
                    double theta=cos2ThetaMin;
                    if (theta>=1.0f)
                        theta=0.0f;
                    else if (theta<=-1.0f)
                        theta=3.14159265f;
                    else
                        theta=acosf(theta);
                    theta*=0.5f;
                    if (sin2ThetaMin<0.0f)
                        theta*=-1.0f;
                    angle=theta;
                    if (overlay)
                    {
                        double rcm[2]={cmx/double(sizeX),cmy/double(sizeY)};
                        double l=0.3f-roundness*0.25f;
                        overlayLines.push_back(int(double(sizeX)*(rcm[0]-cos(theta)*l)));
                        overlayLines.push_back(int(double(sizeY)*(rcm[1]-sin(theta)*l)));
                        overlayLines.push_back(int(double(sizeX)*(rcm[0]+cos(theta)*l)));
                        overlayLines.push_back(int(double(sizeY)*(rcm[1]+sin(theta)*l)));
                    }
                }
                if (overlay)
                { // visualize the CM
                    int rcm[2]={int(cmx+0.5f),int(cmy+0.5f)};
                    overlayLines.push_back(rcm[0]);
                    overlayLines.push_back(rcm[1]-4);
                    overlayLines.push_back(rcm[0]);
                    overlayLines.push_back(rcm[1]+4);
                    overlayLines.push_back(rcm[0]-4);
                    overlayLines.push_back(rcm[1]);
                    overlayLines.push_back(rcm[0]+4);
                    overlayLines.push_back(rcm[1]);
                }
            }
            else
            {
                cmx=0.5f;
                cmy=0.5f;
            }
            returnData.push_back((float)proportion);
            returnData.push_back(float(cmx/double(sizeX)));
            returnData.push_back(float(cmy/double(sizeY)));
            returnData.push_back((float)angle);
            returnData.push_back((float)roundness);
            // Now check if we have to trigger:
            if (triggerEnabled)
            { // we might have to trigger!
                if (fabs(oneProp-proportion)<oneTol)
                { // within proportions
                    if (fabs(xCenter-cmx/double(sizeX))<xCenterTol)
                    { // within cm x-pos
                        if (fabs(yCenter-cmy/double(sizeY))<yCenterTol)
                        { // within cm y-pos
                            double d=fabs(angleMinusAlpha(orient,angle));
                            if (d<orientTol)
                            { // within angular tolerance
                                retVal=(roundness<=roundV);
                            }
                        }
                    }
                }
            }
            if (overlayLines.size()>0)
                drawLines(imgData,overlayLines,col);
        }
        else
            simSetLastError(nullptr,"Invalid handle or work image not initialized.");
    }
    D.pushOutData(CScriptFunctionDataItem(retVal));
    if (returnData.size()>0)
        D.pushOutData(CScriptFunctionDataItem((char*)(&returnData[0]),returnData.size()*sizeof(float))); // packed data is much faster in Lua
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simVision.blobDetectionOnWorkImg
// --------------------------------------------------------------------------------------
#define LUA_BLOBDETECTIONONWORKIMG_COMMAND_PLUGIN "blobDetectionOnWorkImg"

const int inArgs_BLOBDETECTIONONWORKIMG[]={
    5,
    sim_script_arg_int32,0,
    sim_script_arg_double,0,
    sim_script_arg_double,0,
    sim_script_arg_bool,0,
    sim_script_arg_double|sim_script_arg_table,3,
};

void LUA_BLOBDETECTIONONWORKIMG_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    bool retVal=false;
    std::vector<float> returnData;
    if (D.readDataFromStack(p->stackID,inArgs_BLOBDETECTIONONWORKIMG,inArgs_BLOBDETECTIONONWORKIMG[0]-1,nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int handle=getVisionSensorHandle(inData->at(0).int32Data[0],p->objectID);
        double threshold=inData->at(1).doubleData[0];
        double minBlobSize=inData->at(2).doubleData[0];
        bool diffColor=inData->at(3).boolData[0];
        CVisionSensorData* imgData=visionContainer->getImageObject(handle);
        if (imgData!=nullptr)
        {
            double col[3]={1.0f,0.0f,1.0f};
            bool overlay=false;
            if ( (inData->size()>=5)&&(inData->at(4).doubleData.size()==3) )
            {
                col[0]=inData->at(4).doubleData[0];
                col[1]=inData->at(4).doubleData[1];
                col[2]=inData->at(4).doubleData[2];
                overlay=true;
            }
            int sizeX=imgData->resolution[0];
            int sizeY=imgData->resolution[1];
            int s=sizeX*sizeY;
            const double colorsR[12]={1.0f,0.0f,1.0f,0.0f,1.0f,0.0f,1.0f};
            const double colorsG[12]={0.0f,1.0f,1.0f,0.0f,0.0f,1.0f,1.0f};
            const double colorsB[12]={0.0f,0.0f,0.0f,1.0f,1.0f,1.0f,1.0f};

            int* cellAppart=new int[s];
            int currentCellID=0;
            std::vector<std::vector<int>*> cellEquivalencies;
            for (int j=0;j<sizeY;j++)
            {
                for (int i=0;i<sizeX;i++)
                {
                    double intensity=(imgData->workImg[3*(i+j*sizeX)+0]+imgData->workImg[3*(i+j*sizeX)+1]+imgData->workImg[3*(i+j*sizeX)+2])/3.0f;
                    if (intensity>=threshold)
                    { // Binary 1
                        // Check the 4 neighbours:
                        int neighbourCellIDs[4]={99999,99999,99999,99999};
                        if (i>0)
                        {
                            neighbourCellIDs[0]=cellAppart[(i-1)+j*sizeX];
                            if (j>0)
                                neighbourCellIDs[1]=cellAppart[(i-1)+(j-1)*sizeX];
                        }
                        if (j>0)
                            neighbourCellIDs[2]=cellAppart[i+(j-1)*sizeX];
                        if ((i<sizeX-1)&&(j>0))
                            neighbourCellIDs[3]=cellAppart[(i+1)+(j-1)*sizeX];
                        int cellID=neighbourCellIDs[0];
                        if (neighbourCellIDs[1]<cellID)
                            cellID=neighbourCellIDs[1];
                        if (neighbourCellIDs[2]<cellID)
                            cellID=neighbourCellIDs[2];
                        if (neighbourCellIDs[3]<cellID)
                            cellID=neighbourCellIDs[3];
                        if (cellID==99999)
                        {
                            cellID=currentCellID++;
                            cellEquivalencies.push_back(new std::vector<int>);
                            cellEquivalencies[cellEquivalencies.size()-1]->push_back(cellID);
                        }
                        else
                        {
                            for (int k=0;k<4;k++)
                            {
                                if ( (neighbourCellIDs[k]!=99999)&&(neighbourCellIDs[k]!=cellID) )
                                { // Cell is equivalent!
                                    int classCellID=-1;
                                    for (int l=0;l<int(cellEquivalencies.size());l++)
                                    {
                                        for (int m=0;m<int(cellEquivalencies[l]->size());m++)
                                        {
                                            if (cellEquivalencies[l]->at(m)==cellID)
                                            {
                                                classCellID=l;
                                                break;
                                            }
                                        }
                                        if (classCellID!=-1)
                                            break;
                                    }


                                    int classNeighbourCellID=-1;
                                    for (int l=0;l<int(cellEquivalencies.size());l++)
                                    {
                                        for (int m=0;m<int(cellEquivalencies[l]->size());m++)
                                        {
                                            if (cellEquivalencies[l]->at(m)==neighbourCellIDs[k])
                                            {
                                                classNeighbourCellID=l;
                                                break;
                                            }
                                        }
                                        if (classNeighbourCellID!=-1)
                                            break;
                                    }
                                    if (classCellID!=classNeighbourCellID)
                                    { // We have to merge the two classes:
                                        cellEquivalencies[classCellID]->insert(cellEquivalencies[classCellID]->end(),cellEquivalencies[classNeighbourCellID]->begin(),cellEquivalencies[classNeighbourCellID]->end());
                                        delete cellEquivalencies[classNeighbourCellID];
                                        cellEquivalencies.erase(cellEquivalencies.begin()+classNeighbourCellID);
                                    }
                                }
                            }
                        }
                        cellAppart[i+j*sizeX]=cellID;
                    }
                    else
                    { // Binary 0
                        if (diffColor)
                        {
                            imgData->workImg[3*(i+j*sizeX)+0]=0.0f;
                            imgData->workImg[3*(i+j*sizeX)+1]=0.0f;
                            imgData->workImg[3*(i+j*sizeX)+2]=0.0f;
                        }
                        cellAppart[i+j*sizeX]=99999;
                    }
                }
            }
            int* classIDs=new int[currentCellID];
            for (int i=0;i<int(cellEquivalencies.size());i++)
            {
                for (int j=0;j<int(cellEquivalencies[i]->size());j++)
                    classIDs[cellEquivalencies[i]->at(j)]=i;
            }
            std::vector<std::vector<double>*> vertices;
            const int BLOBDATSIZE=6;
            double* blobData=new double[BLOBDATSIZE*currentCellID];
            for (int i=0;i<currentCellID;i++)
            {
                vertices.push_back(new std::vector<double>);
                blobData[BLOBDATSIZE*i+0]=0.0f; // the number of pixels
            }


            for (int j=0;j<sizeY;j++)
            {
                for (int i=0;i<sizeX;i++)
                {
                    int b=cellAppart[i+j*sizeX];
                    if (b!=99999)
                    {
                        double v=0.8f-double(classIDs[b]/7)*0.2f;
                        while (v<0.19f)
                            v+=0.7f;
                        if (diffColor)
                        {
                            imgData->workImg[3*(i+j*sizeX)+0]=colorsR[classIDs[b]%7]*v;
                            imgData->workImg[3*(i+j*sizeX)+1]=colorsG[classIDs[b]%7]*v;
                            imgData->workImg[3*(i+j*sizeX)+2]=colorsB[classIDs[b]%7]*v;
                        }

                        if (    (i==0)||(i==sizeX-1)||(j==0)||(j==sizeY-1)||
                                (cellAppart[(i-1)+j*sizeX]==99999)||(cellAppart[(i+1)+j*sizeX]==99999)||
                                (cellAppart[i+(j-1)*sizeX]==99999)||(cellAppart[i+(j+1)*sizeX]==99999)||
                                (cellAppart[(i-1)+(j-1)*sizeX]==99999)||(cellAppart[(i-1)+(j+1)*sizeX]==99999)||
                                (cellAppart[(i+1)+(j-1)*sizeX]==99999)||(cellAppart[(i+1)+(j+1)*sizeX]==99999) )
                        {
                            vertices[classIDs[b]]->push_back(double(i));
                            vertices[classIDs[b]]->push_back(double(j));
                        }
                        blobData[BLOBDATSIZE*classIDs[b]+0]++;
                    }
                }
            }

            for (int j=0;j<sizeY;j++)
            {
                for (int i=0;i<sizeX;i++)
                {
                    int b=cellAppart[i+j*sizeX];
                    if (b!=99999)
                    {
                        double relSize=blobData[BLOBDATSIZE*classIDs[b]+0]/double(sizeX*sizeY); // relative size of the blob
                        if (relSize<minBlobSize)
                        { // the blob is too small
                            imgData->workImg[3*(i+j*sizeX)+0]=0.0;
                            imgData->workImg[3*(i+j*sizeX)+1]=0.0;
                            imgData->workImg[3*(i+j*sizeX)+2]=0.0;
                        }
                    }
                }
            }

            int blobCount=0;
            returnData.clear();
            returnData.push_back(float(blobCount)); // We have to actualize this at the end!!!
            returnData.push_back(6.0f);
            std::vector<int> overlayLines;
            for (int i=0;i<int(vertices.size());i++)
            { // For each blob...

                if (vertices[i]->size()!=0)
                {
                    double relSize=blobData[BLOBDATSIZE*i+0]/double(sizeX*sizeY); // relative size of the blob
                    if (relSize>=minBlobSize)
                    { // the blob is large enough
                        blobCount++;
                        double bestOrientation[6]={0.0f,99999999999999999999999.0f,0.0f,0.0f,0.0f,0.0f};
                        double previousDa=0.392699081f; // 22.5 degrees
                        for (int j=0;j<4;j++)
                        { // Try 4 orientations..
                            double a=previousDa*double(j);
                            double cosa=cos(a);
                            double sina=sin(a);
                            double minV[2]={99999.0f,99999.0f};
                            double maxV[2]={-99999.0f,-99999.0f};
                            for (int j=0;j<int(vertices[i]->size()/2);j++)
                            {
                                double ox=(*vertices[i])[2*j+0];
                                double oy=(*vertices[i])[2*j+1];
                                double x=ox*cosa-oy*sina;
                                double y=ox*sina+oy*cosa;
                                if (x<minV[0])
                                    minV[0]=x;
                                if (x>maxV[0])
                                    maxV[0]=x;
                                if (y<minV[1])
                                    minV[1]=y;
                                if (y>maxV[1])
                                    maxV[1]=y;
                            }
                            double s=(maxV[0]-minV[0])*(maxV[1]-minV[1]);
                            if (s<bestOrientation[1])
                            {
                                bestOrientation[0]=a;
                                bestOrientation[1]=s;
                                bestOrientation[2]=maxV[0]-minV[0];
                                bestOrientation[3]=maxV[1]-minV[1];
                                double c[2]={(maxV[0]+minV[0])*0.5f,(maxV[1]+minV[1])*0.5f};
                                bestOrientation[4]=c[0]*cos(-a)-c[1]*sin(-a);
                                bestOrientation[5]=c[0]*sin(-a)+c[1]*cos(-a);
                            }
                        }

                        for (int k=0;k<3;k++) // the desired precision here
                        {
                            previousDa/=3.0f;
                            double bestOrientationFromPreviousStep=bestOrientation[0];
                            for (int j=-2;j<=2;j++)
                            { // Try 4 orientations..
                                if (j!=0)
                                {
                                    double a=bestOrientationFromPreviousStep+previousDa*double(j);
                                    double cosa=cos(a);
                                    double sina=sin(a);
                                    double minV[2]={99999.0f,99999.0f};
                                    double maxV[2]={-99999.0f,-99999.0f};
                                    for (int j=0;j<int(vertices[i]->size()/2);j++)
                                    {
                                        double ox=(*vertices[i])[2*j+0];
                                        double oy=(*vertices[i])[2*j+1];
                                        double x=ox*cosa-oy*sina;
                                        double y=ox*sina+oy*cosa;
                                        if (x<minV[0])
                                            minV[0]=x;
                                        if (x>maxV[0])
                                            maxV[0]=x;
                                        if (y<minV[1])
                                            minV[1]=y;
                                        if (y>maxV[1])
                                            maxV[1]=y;
                                    }
                                    double s=(maxV[0]-minV[0])*(maxV[1]-minV[1]);
                                    if (s<bestOrientation[1])
                                    {
                                        bestOrientation[0]=a;
                                        bestOrientation[1]=s;
                                        bestOrientation[2]=maxV[0]-minV[0];
                                        bestOrientation[3]=maxV[1]-minV[1];
                                        double c[2]={(maxV[0]+minV[0])*0.5f,(maxV[1]+minV[1])*0.5f};
                                        bestOrientation[4]=c[0]*cos(-a)-c[1]*sin(-a);
                                        bestOrientation[5]=c[0]*sin(-a)+c[1]*cos(-a);
                                    }
                                }
                            }

                        }

                        bestOrientation[0]=-bestOrientation[0]; // it is inversed!
                        bestOrientation[4]+=0.5f; // b/c of pixel precision
                        bestOrientation[5]+=0.5f; // b/c of pixel precision

                        double c[2]={bestOrientation[4]/sizeX,bestOrientation[5]/sizeY};
                        double v2[2]={bestOrientation[2]*0.5f/sizeX,bestOrientation[3]*0.5f/sizeY};
                        if (overlay)
                        {
                            double cosa=cos(bestOrientation[0]);
                            double sina=sin(bestOrientation[0]);
                            overlayLines.push_back(int(double(sizeX)*(c[0]+v2[0]*cosa-v2[1]*sina)));
                            overlayLines.push_back(int(double(sizeY)*(c[1]+v2[0]*sina+v2[1]*cosa)));
                            overlayLines.push_back(int(double(sizeX)*(c[0]-v2[0]*cosa-v2[1]*sina)));
                            overlayLines.push_back(int(double(sizeY)*(c[1]-v2[0]*sina+v2[1]*cosa)));
                            overlayLines.push_back(int(double(sizeX)*(c[0]-v2[0]*cosa-v2[1]*sina)));
                            overlayLines.push_back(int(double(sizeY)*(c[1]-v2[0]*sina+v2[1]*cosa)));
                            overlayLines.push_back(int(double(sizeX)*(c[0]-v2[0]*cosa+v2[1]*sina)));
                            overlayLines.push_back(int(double(sizeY)*(c[1]-v2[0]*sina-v2[1]*cosa)));
                            overlayLines.push_back(int(double(sizeX)*(c[0]-v2[0]*cosa+v2[1]*sina)));
                            overlayLines.push_back(int(double(sizeY)*(c[1]-v2[0]*sina-v2[1]*cosa)));
                            overlayLines.push_back(int(double(sizeX)*(c[0]+v2[0]*cosa+v2[1]*sina)));
                            overlayLines.push_back(int(double(sizeY)*(c[1]+v2[0]*sina-v2[1]*cosa)));
                            overlayLines.push_back(int(double(sizeX)*(c[0]+v2[0]*cosa+v2[1]*sina)));
                            overlayLines.push_back(int(double(sizeY)*(c[1]+v2[0]*sina-v2[1]*cosa)));
                            overlayLines.push_back(int(double(sizeX)*(c[0]+v2[0]*cosa-v2[1]*sina)));
                            overlayLines.push_back(int(double(sizeY)*(c[1]+v2[0]*sina+v2[1]*cosa)));
                        }
                        returnData.push_back((float)relSize);
                        returnData.push_back((float)bestOrientation[0]);
                        returnData.push_back((float)c[0]);
                        returnData.push_back((float)c[1]);
                        returnData.push_back((float)v2[0]*2.0f);
                        returnData.push_back((float)v2[1]*2.0f);
                    }
                }
            }

            delete[] blobData;
            for (int i=0;i<int(vertices.size());i++)
                delete vertices[i];

            delete[] classIDs;
            for (int i=0;i<int(cellEquivalencies.size());i++)
                delete cellEquivalencies[i];

            delete[] cellAppart;

            if (overlayLines.size()>0)
                drawLines(imgData,overlayLines,col);

            returnData[0]=float(blobCount);
            // the other return data is filled-in here above!
        }
        else
            simSetLastError(nullptr,"Invalid handle or work image not initialized.");
    }
    D.pushOutData(CScriptFunctionDataItem(retVal));
    if (returnData.size()>0)
        D.pushOutData(CScriptFunctionDataItem((char*)(&returnData[0]),returnData.size()*sizeof(float))); // packed data is much faster in Lua
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simVision.sharpenWorkImg
// --------------------------------------------------------------------------------------
#define LUA_SHARPENWORKIMG_COMMAND_PLUGIN "sharpenWorkImg"

const int inArgs_SHARPENWORKIMG[]={
    1,
    sim_script_arg_int32,0,
};

void LUA_SHARPENWORKIMG_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_SHARPENWORKIMG,inArgs_SHARPENWORKIMG[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int handle=getVisionSensorHandle(inData->at(0).int32Data[0],p->objectID);
        CVisionSensorData* imgData=visionContainer->getImageObject(handle);
        if (imgData!=nullptr)
        {
            int sizeX=imgData->resolution[0];
            int sizeY=imgData->resolution[1];
            double m[9]={-0.1111f,-0.1111f,-0.1111f,  -0.1111f,+1.8888f,-0.1111f,  -0.1111f,-0.1111f,-0.1111f};
            double* im2=CImageProcess::createRGBImage(sizeX,sizeY);
            CImageProcess::filter3x3RgbImage(sizeX,sizeY,imgData->workImg,im2,m);
            CImageProcess::copyRGBImage(sizeX,sizeY,im2,imgData->workImg);
            CImageProcess::clampRgbImage(sizeX,sizeY,imgData->workImg,0.0f,1.0f);
            CImageProcess::deleteImage(im2);
        }
        else
            simSetLastError(nullptr,"Invalid handle or work image not initialized.");
    }
    D.pushOutData(CScriptFunctionDataItem(false));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simVision.edgeDetectionOnWorkImg
// --------------------------------------------------------------------------------------
#define LUA_EDGEDETECTIONONWORKIMG_COMMAND_PLUGIN "edgeDetectionOnWorkImg"

const int inArgs_EDGEDETECTIONONWORKIMG[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_double,0,
};

void LUA_EDGEDETECTIONONWORKIMG_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_EDGEDETECTIONONWORKIMG,inArgs_EDGEDETECTIONONWORKIMG[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int handle=getVisionSensorHandle(inData->at(0).int32Data[0],p->objectID);
        double threshold=inData->at(1).doubleData[0];
        CVisionSensorData* imgData=visionContainer->getImageObject(handle);
        if (imgData!=nullptr)
        {
            int sizeX=imgData->resolution[0];
            int sizeY=imgData->resolution[1];
            double m[9]={-3.0f,-3.0f,-3.0f,  -3.0f,24.0f,-3.0f,  -3.0f,-3.0f,-3.0f};
            double* im2=CImageProcess::createRGBImage(sizeX,sizeY);
            CImageProcess::filter3x3RgbImage(sizeX,sizeY,imgData->workImg,im2,m);
            CImageProcess::copyRGBImage(sizeX,sizeY,im2,imgData->workImg);
            int s=sizeX*sizeY;
            for (int i=0;i<s;i++)
            {
                double intens=(imgData->workImg[3*i+0]+imgData->workImg[3*i+1]+imgData->workImg[3*i+2])/3.0f;
                if (intens>threshold)
                {
                    imgData->workImg[3*i+0]=1.0f;
                    imgData->workImg[3*i+1]=1.0f;
                    imgData->workImg[3*i+2]=1.0f;
                }
                else
                {
                    imgData->workImg[3*i+0]=0.0f;
                    imgData->workImg[3*i+1]=0.0f;
                    imgData->workImg[3*i+2]=0.0f;
                }
            }
            CImageProcess::deleteImage(im2);
        }
        else
            simSetLastError(nullptr,"Invalid handle or work image not initialized.");
    }
    D.pushOutData(CScriptFunctionDataItem(false));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simVision.shiftWorkImg
// --------------------------------------------------------------------------------------
#define LUA_SHIFTWORKIMG_COMMAND_PLUGIN "shiftWorkImg"

const int inArgs_SHIFTWORKIMG[]={
    3,
    sim_script_arg_int32,0,
    sim_script_arg_double|sim_lua_arg_table,2,
    sim_script_arg_bool,0,
};

void LUA_SHIFTWORKIMG_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_SHIFTWORKIMG,inArgs_SHIFTWORKIMG[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int handle=getVisionSensorHandle(inData->at(0).int32Data[0],p->objectID);
        double xShift=inData->at(1).doubleData[0];
        double yShift=inData->at(1).doubleData[1];
        double wrap=inData->at(2).boolData[0];
        CVisionSensorData* imgData=visionContainer->getImageObject(handle);
        if (imgData!=nullptr)
        {
            int sizeX=imgData->resolution[0];
            int sizeY=imgData->resolution[1];
            double* im2=CImageProcess::createRGBImage(sizeX,sizeY);
            CImageProcess::copyRGBImage(sizeX,sizeY,imgData->workImg,im2);
            double xShiftT=xShift*double(sizeX);
            double yShiftT=yShift*double(sizeY);
            if (xShiftT>=0.0f)
                xShiftT+=0.5f;
            else
                xShiftT-=0.5f;
            if (yShiftT>=0.0f)
                yShiftT+=0.5f;
            else
                yShiftT-=0.5f;
            int xShift=int(xShiftT);
            int yShift=int(yShiftT);
            if (xShift>sizeX)
                xShift=sizeX;
            if (xShift<-sizeX)
                xShift=-sizeX;
            if (yShift>sizeY)
                yShift=sizeY;
            if (yShift<-sizeY)
                yShift=-sizeY;
            int cpx,cpy;
            int ppx,ppy;
            for (int i=0;i<sizeX;i++)
            {
                cpx=i+xShift;
                ppx=cpx;
                if (ppx>=sizeX)
                    ppx-=sizeX;
                if (ppx<0)
                    ppx+=sizeX;
                for (int j=0;j<sizeY;j++)
                {
                    cpy=j+yShift;
                    ppy=cpy;
                    if (ppy>=sizeY)
                        ppy-=sizeY;
                    if (ppy<0)
                        ppy+=sizeY;
                    if (((cpx<0)||(cpx>=sizeX)||(cpy<0)||(cpy>=sizeY))&&(!wrap))
                    { // we remove that area (black)
                        imgData->workImg[3*(ppx+ppy*sizeX)+0]=0.0f;
                        imgData->workImg[3*(ppx+ppy*sizeX)+1]=0.0f;
                        imgData->workImg[3*(ppx+ppy*sizeX)+2]=0.0f;
                    }
                    else
                    {
                        imgData->workImg[3*(ppx+ppy*sizeX)+0]=im2[3*(i+j*sizeX)+0];
                        imgData->workImg[3*(ppx+ppy*sizeX)+1]=im2[3*(i+j*sizeX)+1];
                        imgData->workImg[3*(ppx+ppy*sizeX)+2]=im2[3*(i+j*sizeX)+2];
                    }
                }
            }
            CImageProcess::deleteImage(im2);
        }
        else
            simSetLastError(nullptr,"Invalid handle or work image not initialized.");
    }
    D.pushOutData(CScriptFunctionDataItem(false));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simVision.circularCutWorkImg
// --------------------------------------------------------------------------------------
#define LUA_CIRCULARCUTWORKIMG_COMMAND_PLUGIN "circularCutWorkImg"

const int inArgs_CIRCULARCUTWORKIMG[]={
    3,
    sim_script_arg_int32,0,
    sim_script_arg_double,0,
    sim_script_arg_bool,0,
};

void LUA_CIRCULARCUTWORKIMG_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_CIRCULARCUTWORKIMG,inArgs_CIRCULARCUTWORKIMG[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int handle=getVisionSensorHandle(inData->at(0).int32Data[0],p->objectID);
        double radius=inData->at(1).doubleData[0];
        double copyToBuffer1=inData->at(2).boolData[0];
        CVisionSensorData* imgData=visionContainer->getImageObject(handle);
        if (imgData!=nullptr)
        {
            if (copyToBuffer1)
            {
                if (imgData->buff1Img==nullptr)
                    imgData->buff1Img=new double[imgData->resolution[0]*imgData->resolution[1]*3];
            }
            int sizeX=imgData->resolution[0];
            int sizeY=imgData->resolution[1];
            int smallestSize=std::min<int>(sizeX,sizeY);
            double centerX=double(sizeX)/2.0f;
            double centerY=double(sizeY)/2.0f;
            double radSquared=radius*double(smallestSize)*0.5f;
            radSquared*=radSquared;
            double dx,dy;
            for (int i=0;i<sizeX;i++)
            {
                dx=double(i)+0.5f-centerX;
                dx*=dx;
                for (int j=0;j<sizeY;j++)
                {
                    dy=double(j)+0.5f-centerY;
                    dy*=dy;
                    if (dy+dx>radSquared)
                    {
                        if (copyToBuffer1)
                        {
                            imgData->buff1Img[3*(i+j*sizeX)+0]=imgData->workImg[3*(i+j*sizeX)+0];
                            imgData->buff1Img[3*(i+j*sizeX)+1]=imgData->workImg[3*(i+j*sizeX)+1];
                            imgData->buff1Img[3*(i+j*sizeX)+2]=imgData->workImg[3*(i+j*sizeX)+2];
                        }
                        imgData->workImg[3*(i+j*sizeX)+0]=0.0f;
                        imgData->workImg[3*(i+j*sizeX)+1]=0.0f;
                        imgData->workImg[3*(i+j*sizeX)+2]=0.0f;
                    }
                    else
                    {
                        if (copyToBuffer1)
                        {
                            imgData->buff1Img[3*(i+j*sizeX)+0]=0.0f;
                            imgData->buff1Img[3*(i+j*sizeX)+1]=0.0f;
                            imgData->buff1Img[3*(i+j*sizeX)+2]=0.0f;
                        }
                    }
                }
            }
        }
        else
            simSetLastError(nullptr,"Invalid handle or work image not initialized.");
    }
    D.pushOutData(CScriptFunctionDataItem(false));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simVision.resizeWorkImg
// --------------------------------------------------------------------------------------
#define LUA_RESIZEWORKIMG_COMMAND_PLUGIN "resizeWorkImg"

const int inArgs_RESIZEWORKIMG[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_double|sim_lua_arg_table,2,
};

void LUA_RESIZEWORKIMG_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_RESIZEWORKIMG,inArgs_RESIZEWORKIMG[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int handle=getVisionSensorHandle(inData->at(0).int32Data[0],p->objectID);
        double xScale=inData->at(1).doubleData[0];
        double yScale=inData->at(1).doubleData[1];
        CVisionSensorData* imgData=visionContainer->getImageObject(handle);
        if (imgData!=nullptr)
        {
            int sizeX=imgData->resolution[0];
            int sizeY=imgData->resolution[1];
            double* im=CImageProcess::createRGBImage(sizeX,sizeY);
            CImageProcess::copyRGBImage(sizeX,sizeY,imgData->workImg,im);
            double* cntIm=CImageProcess::createIntensityImage(sizeX,sizeY);
            int s=sizeX*sizeY;
            for (int i=0;i<s;i++)
            {
                cntIm[i]=0.0f;
                imgData->workImg[3*i+0]=0.0f;
                imgData->workImg[3*i+1]=0.0f;
                imgData->workImg[3*i+2]=0.0f;
            }
            double centerX=double(sizeX)/2.0f;
            double centerY=double(sizeY)/2.0f;
            double dx,dy;
            int npx,npy;
            for (int i=0;i<sizeX;i++)
            {
                dx=(double(i)+0.5f-centerX)/xScale;
                npx=int(centerX+dx);
                if ((npx>=0)&&(npx<sizeX))
                {
                    for (int j=0;j<sizeY;j++)
                    {
                        dy=(double(j)+0.5f-centerY)/yScale;
                        npy=int(centerY+dy);
                        if ((npy>=0)&&(npy<sizeY))
                        {
                            imgData->workImg[3*(i+j*sizeX)+0]+=im[3*(npx+npy*sizeX)+0];
                            imgData->workImg[3*(i+j*sizeX)+1]+=im[3*(npx+npy*sizeX)+1];
                            imgData->workImg[3*(i+j*sizeX)+2]+=im[3*(npx+npy*sizeX)+2];
                            cntIm[i+j*sizeX]+=1.0f;
                        }
                    }
                }
            }
            for (int i=0;i<s;i++)
            {
                if (cntIm[i]>0.1f)
                {
                    imgData->workImg[3*i+0]/=cntIm[i];
                    imgData->workImg[3*i+1]/=cntIm[i];
                    imgData->workImg[3*i+2]/=cntIm[i];
                }
            }
            CImageProcess::deleteImage(cntIm);
            CImageProcess::deleteImage(im);
        }
        else
            simSetLastError(nullptr,"Invalid handle or work image not initialized.");
    }
    D.pushOutData(CScriptFunctionDataItem(false));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simVision.rotateWorkImg
// --------------------------------------------------------------------------------------
#define LUA_ROTATEWORKIMG_COMMAND_PLUGIN "rotateWorkImg"

const int inArgs_ROTATEWORKIMG[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_double,0,
};

void LUA_ROTATEWORKIMG_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_ROTATEWORKIMG,inArgs_ROTATEWORKIMG[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int handle=getVisionSensorHandle(inData->at(0).int32Data[0],p->objectID);
        double rotAngle=inData->at(1).doubleData[0];
        CVisionSensorData* imgData=visionContainer->getImageObject(handle);
        if (imgData!=nullptr)
        {
            int sizeX=imgData->resolution[0];
            int sizeY=imgData->resolution[1];
            double* im=CImageProcess::createRGBImage(sizeX,sizeY);
            CImageProcess::copyRGBImage(sizeX,sizeY,imgData->workImg,im);
            int ss=sizeX*sizeY*3;
            for (int i=0;i<ss;i++)
                imgData->workImg[i]=0.0f;
            double centerX=double(sizeX)/2.0f;
            double centerY=double(sizeY)/2.0f;
            double dx,dy;
            int npx,npy;
            double dxp0;
            double dxp1;
            double dyp0;
            double dyp1;
            double c=cos(-rotAngle);
            double s=sin(-rotAngle);
            for (int i=0;i<sizeX;i++)
            {
                dx=double(i)+0.5f-centerX;
                dxp0=dx*c;
                dyp0=dx*s;
                for (int j=0;j<sizeY;j++)
                {
                    dy=double(j)+0.5f-centerY;
                    dxp1=-dy*s;
                    dyp1=dy*c;
                    npx=int(centerX+dxp0+dxp1);
                    npy=int(centerY+dyp0+dyp1);
                    if ((npy>=0)&&(npy<sizeY)&&(npx>=0)&&(npx<sizeX))
                    {
                        imgData->workImg[3*(i+j*sizeX)+0]+=im[3*(npx+npy*sizeX)+0];
                        imgData->workImg[3*(i+j*sizeX)+1]+=im[3*(npx+npy*sizeX)+1];
                        imgData->workImg[3*(i+j*sizeX)+2]+=im[3*(npx+npy*sizeX)+2];
                    }
                }
            }
            CImageProcess::deleteImage(im);
        }
        else
            simSetLastError(nullptr,"Invalid handle or work image not initialized.");
    }
    D.pushOutData(CScriptFunctionDataItem(false));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simVision.matrix3x3OnWorkImg
// --------------------------------------------------------------------------------------
#define LUA_MATRIX3X3ONWORKIMG_COMMAND_PLUGIN "matrix3x3OnWorkImg"

const int inArgs_MATRIX3X3ONWORKIMG[]={
    4,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_double,0,
    sim_script_arg_double|sim_lua_arg_table,9,
};

void LUA_MATRIX3X3ONWORKIMG_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_MATRIX3X3ONWORKIMG,inArgs_MATRIX3X3ONWORKIMG[0]-1,nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int handle=getVisionSensorHandle(inData->at(0).int32Data[0],p->objectID);
        int passes=inData->at(1).int32Data[0];
        double multiplier=inData->at(2).doubleData[0];
        double m[9];
        if ( (inData->size()<4)||(inData->at(3).doubleData.size()<9) )
        {
            for (size_t i=0;i<9;i++)
                m[i]=1.0f;
            const double sigma=2.0f;
            double tot=0.0f;
            for (int i=-1;i<2;i++)
            {
                for (int j=-1;j<2;j++)
                {
                    double v=pow(2.7182818f,-(i*i+j*j)/(2.0f*sigma*sigma))/(2.0f*3.14159265f*sigma*sigma);
                    m[1+i+(j+1)*3]=v;
                    tot+=v;
                }
            }
            for (size_t i=0;i<9;i++)
                m[i]*=multiplier/tot;
        }
        else
        {
            for (size_t i=0;i<9;i++)
                m[i]=inData->at(3).doubleData[i]*multiplier;
        }
        CVisionSensorData* imgData=visionContainer->getImageObject(handle);
        if (imgData!=nullptr)
        {
            int sizeX=imgData->resolution[0];
            int sizeY=imgData->resolution[1];
            double* im2=CImageProcess::createRGBImage(sizeX,sizeY);
            for (int i=0;i<passes/2;i++)
            {
                CImageProcess::filter3x3RgbImage(sizeX,sizeY,imgData->workImg,im2,m);
                CImageProcess::filter3x3RgbImage(sizeX,sizeY,im2,imgData->workImg,m);
            }
            if (passes&1)
            {
                CImageProcess::filter3x3RgbImage(sizeX,sizeY,imgData->workImg,im2,m);
                CImageProcess::copyRGBImage(sizeX,sizeY,im2,imgData->workImg);
            }
            CImageProcess::clampRgbImage(sizeX,sizeY,imgData->workImg,0.0f,1.0f);
            CImageProcess::deleteImage(im2);
        }
        else
            simSetLastError(nullptr,"Invalid handle or work image not initialized.");
    }
    D.pushOutData(CScriptFunctionDataItem(false));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simVision.matrix5x5OnWorkImg
// --------------------------------------------------------------------------------------
#define LUA_MATRIX5X5ONWORKIMG_COMMAND_PLUGIN "matrix5x5OnWorkImg"

const int inArgs_MATRIX5X5ONWORKIMG[]={
    4,
    sim_script_arg_int32,0,
    sim_script_arg_int32,0,
    sim_script_arg_double,0,
    sim_script_arg_double|sim_lua_arg_table,25,
};

void LUA_MATRIX5X5ONWORKIMG_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_MATRIX5X5ONWORKIMG,inArgs_MATRIX5X5ONWORKIMG[0]-1,nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int handle=getVisionSensorHandle(inData->at(0).int32Data[0],p->objectID);
        int passes=inData->at(1).int32Data[0];
        double multiplier=inData->at(2).doubleData[0];
        double m[25];
        if ( (inData->size()<4)||(inData->at(3).doubleData.size()<25) )
        {
            for (size_t i=0;i<25;i++)
                m[i]=1.0f;
            const double sigma=2.0f;
            double tot=0.0f;
            for (int i=-2;i<3;i++)
            {
                for (int j=-2;j<3;j++)
                {
                    double v=pow(2.7182818f,-(i*i+j*j)/(2.0f*sigma*sigma))/(2.0f*3.14159265f*sigma*sigma);
                    m[i+2+(j+2)*5]=v;
                    tot+=v;
                }
            }
            for (size_t i=0;i<25;i++)
                m[i]*=multiplier/tot;
        }
        else
        {
            for (size_t i=0;i<25;i++)
                m[i]=inData->at(3).doubleData[i]*multiplier;
        }
        CVisionSensorData* imgData=visionContainer->getImageObject(handle);
        if (imgData!=nullptr)
        {
            int sizeX=imgData->resolution[0];
            int sizeY=imgData->resolution[1];
            double* im2=CImageProcess::createRGBImage(sizeX,sizeY);
            for (int i=0;i<passes/2;i++)
            {
                CImageProcess::filter5x5RgbImage(sizeX,sizeY,imgData->workImg,im2,m);
                CImageProcess::filter5x5RgbImage(sizeX,sizeY,im2,imgData->workImg,m);
            }
            if (passes&1)
            {
                CImageProcess::filter5x5RgbImage(sizeX,sizeY,imgData->workImg,im2,m);
                CImageProcess::copyRGBImage(sizeX,sizeY,im2,imgData->workImg);
            }
            CImageProcess::clampRgbImage(sizeX,sizeY,imgData->workImg,0.0f,1.0f);
            CImageProcess::deleteImage(im2);
        }
        else
            simSetLastError(nullptr,"Invalid handle or work image not initialized.");
    }
    D.pushOutData(CScriptFunctionDataItem(false));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simVision.rectangularCutWorkImg
// --------------------------------------------------------------------------------------
#define LUA_RECTANGULARCUTWORKIMG_COMMAND_PLUGIN "rectangularCutWorkImg"

const int inArgs_RECTANGULARCUTWORKIMG[]={
    3,
    sim_script_arg_int32,0,
    sim_script_arg_double|sim_lua_arg_table,2,
    sim_script_arg_bool,0,
};

void LUA_RECTANGULARCUTWORKIMG_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_RECTANGULARCUTWORKIMG,inArgs_RECTANGULARCUTWORKIMG[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int handle=getVisionSensorHandle(inData->at(0).int32Data[0],p->objectID);
        double nsizeX=inData->at(1).doubleData[0];
        double nsizeY=inData->at(1).doubleData[1];
        double copyToBuffer1=inData->at(2).boolData[0];
        CVisionSensorData* imgData=visionContainer->getImageObject(handle);
        if (imgData!=nullptr)
        {
            if (copyToBuffer1)
            {
                if (imgData->buff1Img==nullptr)
                    imgData->buff1Img=new double[imgData->resolution[0]*imgData->resolution[1]*3];
            }
            int sizeX=imgData->resolution[0];
            int sizeY=imgData->resolution[1];
            double centerX=double(sizeX)/2.0f;
            double centerY=double(sizeY)/2.0f;
            double dx,dy;
            double hdx=nsizeX*0.5f;
            double hdy=nsizeY*0.5f;
            for (int i=0;i<sizeX;i++)
            {
                dx=double(i)+0.5f-centerX;
                for (int j=0;j<sizeY;j++)
                {
                    dy=double(j)+0.5f-centerY;
                    if ((fabs(dx)>hdx*double(sizeX))||(fabs(dy)>hdy*double(sizeY)))
                    {
                        if (copyToBuffer1)
                        {
                            imgData->buff1Img[3*(i+j*sizeX)+0]=imgData->workImg[3*(i+j*sizeX)+0];
                            imgData->buff1Img[3*(i+j*sizeX)+1]=imgData->workImg[3*(i+j*sizeX)+1];
                            imgData->buff1Img[3*(i+j*sizeX)+2]=imgData->workImg[3*(i+j*sizeX)+2];
                        }
                        imgData->workImg[3*(i+j*sizeX)+0]=0.0f;
                        imgData->workImg[3*(i+j*sizeX)+1]=0.0f;
                        imgData->workImg[3*(i+j*sizeX)+2]=0.0f;
                    }
                    else
                    {
                        if (copyToBuffer1)
                        {
                            imgData->buff1Img[3*(i+j*sizeX)+0]=0.0f;
                            imgData->buff1Img[3*(i+j*sizeX)+1]=0.0f;
                            imgData->buff1Img[3*(i+j*sizeX)+2]=0.0f;
                        }
                    }
                }
            }
        }
        else
            simSetLastError(nullptr,"Invalid handle or work image not initialized.");
    }
    D.pushOutData(CScriptFunctionDataItem(false));
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simVision.coordinatesFromWorkImg
// --------------------------------------------------------------------------------------
#define LUA_COORDINATESFROMWORKIMG_COMMAND_PLUGIN "coordinatesFromWorkImg"

const int inArgs_COORDINATESFROMWORKIMG[]={
    4,
    sim_script_arg_int32,0,
    sim_script_arg_int32|sim_lua_arg_table,2,
    sim_script_arg_bool,0,
    sim_script_arg_bool,0,
};

void LUA_COORDINATESFROMWORKIMG_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    std::vector<float> returnData;
    std::vector<unsigned char> returnCol;
    if (D.readDataFromStack(p->stackID,inArgs_COORDINATESFROMWORKIMG,inArgs_COORDINATESFROMWORKIMG[0]-1,nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int hhandle=inData->at(0).int32Data[0];
        bool absCoords=false;
        if (hhandle>=0)
        {
            absCoords=((hhandle&sim_handleflag_abscoords)!=0);
            hhandle=hhandle&0x000fffff;
        }

        int handle=getVisionSensorHandle(hhandle,p->objectID);
        int ptCntX=inData->at(1).int32Data[0];
        int ptCntY=inData->at(1).int32Data[1];
        bool angularSpace=inData->at(2).boolData[0];
        bool returnRgb=false;
        if (inData->size()>=4)
            returnRgb=inData->at(3).boolData[0];
        CVisionSensorData* imgData=visionContainer->getImageObject(handle);
        if (imgData!=nullptr)
        {
            if (imgData->buff1Img==nullptr)
                returnRgb=false;
            C7Vector sensorTr;
            simGetObjectPosition(handle,-1,sensorTr.X.data);
            double q[4];
            simGetObjectQuaternion(handle,-1,q);
            sensorTr.Q=C4Vector(q[3],q[0],q[1],q[2]); // CoppeliaSim quaternion, internally: w x y z, at interfaces: x y z w
            int sizeX=imgData->resolution[0];
            int sizeY=imgData->resolution[1];

            int perspectiveOperation;
            simGetObjectInt32Param(handle,sim_visionintparam_perspective_operation,&perspectiveOperation);
            double np,fp;
            simGetObjectFloatParam(handle,sim_visionfloatparam_near_clipping,&np);
            simGetObjectFloatParam(handle,sim_visionfloatparam_far_clipping,&fp);
            double depthThresh=np;
            double depthRange=fp-depthThresh;
            double farthestValue=fp;
            double xAngle;
            simGetObjectFloatParam(handle,sim_visionfloatparam_perspective_angle,&xAngle);
            double yAngle=xAngle;
            double ratio=double(sizeX)/double(sizeY);
            if (sizeX>sizeY)
                yAngle=2.0*atan(tan(xAngle/2.0)/ratio);
            else
                xAngle=2.0*atan(tan(xAngle/2.0)*ratio);
            double xS;
            simGetObjectFloatParam(handle,sim_visionfloatparam_ortho_size,&xS);
            double yS=xS;
            if (sizeX>sizeY)
                yS=xS/ratio;
            else
                xS=xS*ratio;


            int xPtCnt=ptCntX;
            int yPtCnt=ptCntY;

            returnData.clear();
            returnData.push_back(float(xPtCnt));
            returnData.push_back(float(yPtCnt));


            if (perspectiveOperation!=0)
            {
                double yDist=0.0f;
                double dy=0.0f;
                if (yPtCnt>1)
                {
                    dy=yAngle/double(yPtCnt-1);
                    yDist=-yAngle*0.5f;
                }
                double dx=0.0f;
                if (xPtCnt>1)
                    dx=xAngle/double(xPtCnt-1);

                double xAlpha=0.5f/(tan(xAngle*0.5f));
                double yAlpha=0.5f/(tan(yAngle*0.5f));

                double xBeta=2.0f*tan(xAngle*0.5f);
                double yBeta=2.0f*tan(yAngle*0.5f);

                for (int j=0;j<yPtCnt;j++)
                {
                    double tanYDistTyAlpha;
                    int yRow;
                    if (angularSpace)
                    {
                        tanYDistTyAlpha=tan(yDist)*yAlpha;
                        yRow=int((tanYDistTyAlpha+0.5f)*(sizeY-0.5f));
                    }
                    else
                        yRow=int((0.5f+yDist/yAngle)*(sizeY-0.5f));

                    double xDist=0.0f;
                    if (xPtCnt>1)
                        xDist=-xAngle*0.5f;
                    for (int i=0;i<xPtCnt;i++)
                    {
                        C3Vector v;
                        if (angularSpace)
                        {
                            double tanXDistTxAlpha=tan(xDist)*xAlpha;
                            int xRow=int((0.5f-tanXDistTxAlpha)*(sizeX-0.5f));
                            int indexP=3*(xRow+yRow*sizeX);
                            double intensity=(imgData->workImg[indexP+0]+imgData->workImg[indexP+1]+imgData->workImg[indexP+2])/3.0f;
                            double zDist=depthThresh+intensity*depthRange;
                            v.setData(tanXDistTxAlpha*xBeta*zDist,tanYDistTyAlpha*yBeta*zDist,zDist);
                            if (returnRgb)
                            {
                                returnCol.push_back((unsigned char)(imgData->buff1Img[indexP+0]*255.1f));
                                returnCol.push_back((unsigned char)(imgData->buff1Img[indexP+1]*255.1f));
                                returnCol.push_back((unsigned char)(imgData->buff1Img[indexP+2]*255.1f));
                            }
                        }
                        else
                        {
                            int xRow=int((0.5f-xDist/xAngle)*(sizeX-0.5f));
                            int indexP=3*(xRow+yRow*sizeX);
                            double intensity=(imgData->workImg[indexP+0]+imgData->workImg[indexP+1]+imgData->workImg[indexP+2])/3.0f;
                            double zDist=depthThresh+intensity*depthRange;
                            v.setData(tan(xAngle*0.5f)*xDist/(xAngle*0.5f)*zDist,tan(yAngle*0.5f)*yDist/(yAngle*0.5f)*zDist,zDist);
                            if (returnRgb)
                            {
                                returnCol.push_back((unsigned char)(imgData->buff1Img[indexP+0]*255.1f));
                                returnCol.push_back((unsigned char)(imgData->buff1Img[indexP+1]*255.1f));
                                returnCol.push_back((unsigned char)(imgData->buff1Img[indexP+2]*255.1f));
                            }
                        }

                        double l=v.getLength();
                        if (l>farthestValue)
                        {
                            v=(v/l)*farthestValue;
                            if (absCoords)
                                v=sensorTr*v;
                            returnData.push_back((float)v(0));
                            returnData.push_back((float)v(1));
                            returnData.push_back((float)v(2));
                            returnData.push_back((float)farthestValue);
                        }
                        else
                        {
                            if (absCoords)
                                v=sensorTr*v;
                            returnData.push_back((float)v(0));
                            returnData.push_back((float)v(1));
                            returnData.push_back((float)v(2));
                            returnData.push_back((float)l);
                        }
                        xDist+=dx;
                    }
                    yDist+=dy;
                }
            }
            else
            {
                double yDist=0.0f;
                double dy=0.0f;
                if (yPtCnt>1)
                {
                    dy=yS/double(yPtCnt-1);
                    yDist=-yS*0.5f;
                }
                double dx=0.0f;
                if (xPtCnt>1)
                    dx=xS/double(xPtCnt-1);

                for (int j=0;j<yPtCnt;j++)
                {
                    int yRow=int(((yDist+yS*0.5f)/yS)*(sizeY-0.5f));

                    double xDist=0.0f;
                    if (xPtCnt>1)
                        xDist=-xS*0.5f;
                    for (int i=0;i<xPtCnt;i++)
                    {
                        int xRow=int((1.0f-((xDist+xS*0.5f)/xS))*(sizeX-0.5f));
                        int indexP=3*(xRow+yRow*sizeX);
                        double intensity=(imgData->workImg[indexP+0]+imgData->workImg[indexP+1]+imgData->workImg[indexP+2])/3.0f;
                        double zDist=depthThresh+intensity*depthRange;
                        if (returnRgb)
                        {
                            returnCol.push_back((unsigned char)(imgData->buff1Img[indexP+0]*255.1f));
                            returnCol.push_back((unsigned char)(imgData->buff1Img[indexP+1]*255.1f));
                            returnCol.push_back((unsigned char)(imgData->buff1Img[indexP+2]*255.1f));
                        }
                        if (absCoords)
                        {
                            C3Vector absv(sensorTr*C3Vector(xDist,yDist,zDist));
                            returnData.push_back((float)absv(0));
                            returnData.push_back((float)absv(1));
                            returnData.push_back((float)absv(2));
                        }
                        else
                        {
                            returnData.push_back((float)xDist);
                            returnData.push_back((float)yDist);
                            returnData.push_back((float)zDist);
                        }
                        returnData.push_back((float)zDist);
                        xDist+=dx;
                    }
                    yDist+=dy;
                }
            }
        }
        else
            simSetLastError(nullptr,"Invalid handle or work image not initialized.");
    }
    D.pushOutData(CScriptFunctionDataItem(false));
    if (returnData.size()>0)
    {
        D.pushOutData(CScriptFunctionDataItem((char*)(&returnData[0]),returnData.size()*sizeof(float))); // packed data is much faster in Lua
        if (returnCol.size()>0)
        {
            returnCol.push_back(0); // return one more color, b/c CoppeliaSim assumes the data to be n*4 bytes in length, i.e. floats
            returnCol.push_back(0);
            returnCol.push_back(0);
            D.pushOutData(CScriptFunctionDataItem((char*)(&returnCol[0]),returnCol.size()));
        }
    }
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simVision.changedPixelsOnWorkImg
// --------------------------------------------------------------------------------------
#define LUA_CHANGEDPIXELSONWORKIMG_COMMAND_PLUGIN "changedPixelsOnWorkImg"

const int inArgs_CHANGEDPIXELSONWORKIMG[]={
    2,
    sim_script_arg_int32,0,
    sim_script_arg_double,0,
};

void LUA_CHANGEDPIXELSONWORKIMG_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    std::vector<float> returnData;
    if (D.readDataFromStack(p->stackID,inArgs_CHANGEDPIXELSONWORKIMG,inArgs_CHANGEDPIXELSONWORKIMG[0],nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int handle=getVisionSensorHandle(inData->at(0).int32Data[0],p->objectID);
        double thresh=inData->at(1).doubleData[0];
        CVisionSensorData* imgData=visionContainer->getImageObject(handle);
        if (imgData!=nullptr)
        {
            int sizeX=imgData->resolution[0];
            int sizeY=imgData->resolution[1];
            if (imgData->buff2Img==nullptr)
                imgData->buff2Img=new double[imgData->resolution[0]*imgData->resolution[1]*3];
            for (int j=0;j<sizeY;j++)
            {
                for (int i=0;i<sizeX;i++)
                {
                    double buffIntens=(imgData->buff2Img[3*(i+j*sizeX)+0]+imgData->buff2Img[3*(i+j*sizeX)+1]+imgData->buff2Img[3*(i+j*sizeX)+2])/3.0f;
                    double imgIntens=(imgData->workImg[3*(i+j*sizeX)+0]+imgData->workImg[3*(i+j*sizeX)+1]+imgData->workImg[3*(i+j*sizeX)+2])/3.0f;
                    double diff=imgIntens-buffIntens;
                    if (buffIntens==0.0f)
                        buffIntens=0.001f;
                    if (fabs(diff)/buffIntens>=thresh)
                    {
                        imgData->buff2Img[3*(i+j*sizeX)+0]=imgIntens;
                        imgData->buff2Img[3*(i+j*sizeX)+1]=imgIntens;
                        imgData->buff2Img[3*(i+j*sizeX)+2]=imgIntens;
                        if (diff>0)
                        {
                            imgData->workImg[3*(i+j*sizeX)+0]=1.0f;
                            imgData->workImg[3*(i+j*sizeX)+1]=1.0f;
                            imgData->workImg[3*(i+j*sizeX)+2]=1.0f;
                            returnData.push_back(1.0f);
                        }
                        else
                        {
                            imgData->workImg[3*(i+j*sizeX)+0]=0.0f;
                            imgData->workImg[3*(i+j*sizeX)+1]=0.0f;
                            imgData->workImg[3*(i+j*sizeX)+2]=0.0f;
                            returnData.push_back(-1.0f);
                        }
                        returnData.push_back(float(i));
                        returnData.push_back(float(j));
                    }
                    else
                    {
                        imgData->workImg[3*(i+j*sizeX)+0]=0.5f;
                        imgData->workImg[3*(i+j*sizeX)+1]=0.5f;
                        imgData->workImg[3*(i+j*sizeX)+2]=0.5f;
                    }
                }
            }
        }
        else
            simSetLastError(nullptr,"Invalid handle or work image not initialized.");
    }
    D.pushOutData(CScriptFunctionDataItem(false));
    if (returnData.size()>0)
        D.pushOutData(CScriptFunctionDataItem((char*)(&returnData[0]),returnData.size()*sizeof(float))); // packed data is much faster in Lua
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simVision.velodyneDataFromWorkImg
// --------------------------------------------------------------------------------------
#define LUA_VELODYNEDATAFROMWORKIMG_COMMAND_PLUGIN "velodyneDataFromWorkImg"

const int inArgs_VELODYNEDATAFROMWORKIMG[]={
    4,
    sim_script_arg_int32,0,
    sim_script_arg_int32|sim_lua_arg_table,2,
    sim_script_arg_double,0,
    sim_script_arg_bool,0,
};

void LUA_VELODYNEDATAFROMWORKIMG_CALLBACK(SScriptCallBack* p)
{
    CScriptFunctionData D;
    std::vector<float> returnData;
    std::vector<unsigned char> returnCol;
    if (D.readDataFromStack(p->stackID,inArgs_VELODYNEDATAFROMWORKIMG,inArgs_VELODYNEDATAFROMWORKIMG[0]-1,nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int hhandle=inData->at(0).int32Data[0];
        bool absCoords=false;
        if (hhandle>=0)
        {
            absCoords=((hhandle&sim_handleflag_abscoords)!=0);
            hhandle=hhandle&0x000fffff;
        }
        int handle=getVisionSensorHandle(hhandle,p->objectID);
        int xPtCnt=inData->at(1).int32Data[0];
        int yPtCnt=inData->at(1).int32Data[1];
        double vAngle=inData->at(2).doubleData[0];
        bool returnRgb=false;
        if (inData->size()>=4)
            returnRgb=inData->at(3).boolData[0];
        CVisionSensorData* imgData=visionContainer->getImageObject(handle);
        if (imgData!=nullptr)
        {
            if (imgData->buff1Img==nullptr)
                returnRgb=false;
            C7Vector sensorTr;
            simGetObjectPosition(handle,-1,sensorTr.X.data);
            double q[4];
            simGetObjectQuaternion(handle,-1,q);
            sensorTr.Q=C4Vector(q[3],q[0],q[1],q[2]); // CoppeliaSim quaternion, internally: w x y z, at interfaces: x y z w

            int sizeX=imgData->resolution[0];
            int sizeY=imgData->resolution[1];

            double np,fp;
            simGetObjectFloatParam(handle,sim_visionfloatparam_near_clipping,&np);
            simGetObjectFloatParam(handle,sim_visionfloatparam_far_clipping,&fp);
            double xAngle;
            simGetObjectFloatParam(handle,sim_visionfloatparam_perspective_angle,&xAngle);

            double depthThresh=np;
            double depthRange=fp-depthThresh;
            double farthestValue=fp;
            double yAngle=xAngle;
            double ratio=double(sizeX)/double(sizeY);
            if (sizeX>sizeY)
                yAngle=2.0f*(double)atan(tan(xAngle/2.0f)/ratio);
            else
                xAngle=2.0f*(double)atan(tan(xAngle/2.0f)*ratio);
            returnData.clear();
            returnData.push_back(float(xPtCnt));
            returnData.push_back(float(yPtCnt));

            //if (sensor->getPerspectiveOperation())
            {
                double dx=0.0f;
                if (xPtCnt>1)
                    dx=xAngle/double(xPtCnt-1);
                double xDist=0.0f;
                if (xPtCnt>1)
                    xDist=-xAngle*0.5f;

                double xAlpha=0.5f/(tan(xAngle*0.5f));

                double xBeta=2.0f*tan(xAngle*0.5f);
                double yBeta=2.0f*tan(yAngle*0.5f);

                for (int j=0;j<xPtCnt;j++)
                {
                    double h=1.0f/cos(xDist);

                    double yDist=0.0f;
                    double dy=0.0f;
                    if (yPtCnt>1)
                    {
                        dy=vAngle/double(yPtCnt-1);
                        yDist=-vAngle*0.5f;
                    }

                    double tanXDistTxAlpha=tan(xDist)*xAlpha;
                    int xRow=int((0.5f-tanXDistTxAlpha)*(sizeX-0.5f));

                    if (xRow<0)
                        xRow=0;
                    if (xRow>=sizeX)
                        xRow=sizeX-1;

                    double yAlpha=0.5f/(tan(yAngle*0.5f));

                    for (int i=0;i<yPtCnt;i++)
                    {
                        double tanYDistTyAlpha=tan(yDist)*h*yAlpha;
                        int yRow=int((tanYDistTyAlpha+0.5f)*(sizeY-0.5f));
                        if (yRow<0)
                            yRow=0;
                        if (yRow>=sizeY)
                            yRow=sizeY-1;
                        int indexP=3*(xRow+yRow*sizeX);
                        double intensity=(imgData->workImg[indexP+0]+imgData->workImg[indexP+1]+imgData->workImg[indexP+2])/3.0f;
                        if (returnRgb)
                        {
                            returnCol.push_back((unsigned char)(imgData->buff1Img[indexP+0]*255.1f));
                            returnCol.push_back((unsigned char)(imgData->buff1Img[indexP+1]*255.1f));
                            returnCol.push_back((unsigned char)(imgData->buff1Img[indexP+2]*255.1f));
                        }
                        double zDist=depthThresh+intensity*depthRange;
                        C3Vector v(tanXDistTxAlpha*xBeta*zDist,tanYDistTyAlpha*yBeta*zDist,zDist);
                        double l=v.getLength();
                        if (l>farthestValue)
                        {
                            v=(v/l)*farthestValue;
                            if (absCoords)
                                v=sensorTr*v;
                            returnData.push_back((float)v(0));
                            returnData.push_back((float)v(1));
                            returnData.push_back((float)v(2));
                            returnData.push_back((float)farthestValue);
                        }
                        else
                        {
                            if (absCoords)
                                v=sensorTr*v;
                            returnData.push_back((float)v(0));
                            returnData.push_back((float)v(1));
                            returnData.push_back((float)v(2));
                            returnData.push_back((float)l);
                        }
                        yDist+=dy;
                    }
                    xDist+=dx;
                }
            }
        }
        else
            simSetLastError(nullptr,"Invalid handle or work image not initialized.");
    }
    D.pushOutData(CScriptFunctionDataItem(false));
    if (returnData.size()>0)
    {
        D.pushOutData(CScriptFunctionDataItem((char*)(&returnData[0]),returnData.size()*sizeof(float))); // packed data is much faster in Lua
        if (returnCol.size()>0)
        {
            returnCol.push_back(0); // return one more color, b/c CoppeliaSim assumes the data to be n*4 bytes in length, i.e. floats
            returnCol.push_back(0);
            returnCol.push_back(0);
            D.pushOutData(CScriptFunctionDataItem((char*)(&returnCol[0]),returnCol.size()));
        }
    }
    D.writeDataToStack(p->stackID);
}
// --------------------------------------------------------------------------------------

SIM_DLLEXPORT int simInit(const char* pluginName)
{
    _pluginName=pluginName;
    char curDirAndFile[1024];
#ifdef _WIN32
    #ifdef QT_COMPIL
        _getcwd(curDirAndFile, sizeof(curDirAndFile));
    #else
        GetModuleFileName(NULL,curDirAndFile,1023);
        PathRemoveFileSpec(curDirAndFile);
    #endif
#elif defined (__linux) || defined (__APPLE__)
    getcwd(curDirAndFile, sizeof(curDirAndFile));
#endif

    std::string currentDirAndPath(curDirAndFile);
    std::string temp(currentDirAndPath);

#ifdef _WIN32
    temp+="\\coppeliaSim.dll";
#elif defined (__linux)
    temp+="/libcoppeliaSim.so";
#elif defined (__APPLE__)
    temp+="/libcoppeliaSim.dylib";
#endif

    simLib=loadSimLibrary(temp.c_str());
    if (simLib==NULL)
    {
        simAddLog(pluginName,sim_verbosity_errors,"could not find or correctly load the CoppeliaSim library. Cannot start the plugin.");
        return(0);
    }
    if (getSimProcAddresses(simLib)==0)
    {
        simAddLog(pluginName,sim_verbosity_errors,"could not find all required functions in the CoppeliaSim library. Cannot start the plugin.");
        unloadSimLibrary(simLib);
        return(0);
    }

    // Register the new Lua commands:

    // Spherical vision sensor:
    simRegisterScriptCallbackFunction(LUA_HANDLESPHERICAL_COMMAND_PLUGIN,nullptr,LUA_HANDLESPHERICAL_CALLBACK);

    // Anaglyph sensor:
    simRegisterScriptCallbackFunction(LUA_HANDLEANAGLYPHSTEREO_COMMAND_PLUGIN,nullptr,LUA_HANDLEANAGLYPHSTEREO_CALLBACK);


    // HDL-64E:
    simRegisterScriptCallbackFunction(LUA_CREATEVELODYNEHDL64E_COMMAND_PLUGIN,nullptr,LUA_CREATEVELODYNEHDL64E_CALLBACK);
    simRegisterScriptCallbackFunction(LUA_DESTROYVELODYNEHDL64E_COMMAND_PLUGIN,nullptr,LUA_DESTROYVELODYNEHDL64E_CALLBACK);
    simRegisterScriptCallbackFunction(LUA_HANDLEVELODYNEHDL64E_COMMAND_PLUGIN,nullptr,LUA_HANDLEVELODYNEHDL64E_CALLBACK);

    simRegisterScriptCallbackFunction(LUA_CREATEVELODYNEVPL16_COMMAND_PLUGIN,nullptr,LUA_CREATEVELODYNEVPL16_CALLBACK);
    simRegisterScriptCallbackFunction(LUA_DESTROYVELODYNEVPL16_COMMAND_PLUGIN,nullptr,LUA_DESTROYVELODYNEVPL16_CALLBACK);
    simRegisterScriptCallbackFunction(LUA_HANDLEVELODYNEVPL16_COMMAND_PLUGIN,nullptr,LUA_HANDLEVELODYNEVPL16_CALLBACK);

    // basic vision sensor processing:
    simRegisterScriptCallbackFunction(LUA_SENSORIMGTOWORKIMG_COMMAND_PLUGIN,nullptr,LUA_SENSORIMGTOWORKIMG_CALLBACK);
    simRegisterScriptCallbackFunction(LUA_SENSORDEPTHMAPTOWORKIMG_COMMAND_PLUGIN,nullptr,LUA_SENSORDEPTHMAPTOWORKIMG_CALLBACK);
    simRegisterScriptCallbackFunction(LUA_WORKIMGTOSENSORIMG_COMMAND_PLUGIN,nullptr,LUA_WORKIMGTOSENSORIMG_CALLBACK);
    simRegisterScriptCallbackFunction(LUA_WORKIMGTOSENSORDEPTHMAP_COMMAND_PLUGIN,nullptr,LUA_WORKIMGTOSENSORDEPTHMAP_CALLBACK);
    simRegisterScriptCallbackFunction(LUA_WORKIMGTOBUFFER1_COMMAND_PLUGIN,nullptr,LUA_WORKIMGTOBUFFER1_CALLBACK);
    simRegisterScriptCallbackFunction(LUA_WORKIMGTOBUFFER2_COMMAND_PLUGIN,nullptr,LUA_WORKIMGTOBUFFER2_CALLBACK);
    simRegisterScriptCallbackFunction(LUA_SWAPBUFFERS_COMMAND_PLUGIN,nullptr,LUA_SWAPBUFFERS_CALLBACK);
    simRegisterScriptCallbackFunction(LUA_BUFFER1TOWORKIMG_COMMAND_PLUGIN,nullptr,LUA_BUFFER1TOWORKIMG_CALLBACK);
    simRegisterScriptCallbackFunction(LUA_BUFFER2TOWORKIMG_COMMAND_PLUGIN,nullptr,LUA_BUFFER2TOWORKIMG_CALLBACK);
    simRegisterScriptCallbackFunction(LUA_SWAPWORKIMGWITHBUFFER1_COMMAND_PLUGIN,nullptr,LUA_SWAPWORKIMGWITHBUFFER1_CALLBACK);
    simRegisterScriptCallbackFunction(LUA_ADDWORKIMGTOBUFFER1_COMMAND_PLUGIN,nullptr,LUA_ADDWORKIMGTOBUFFER1_CALLBACK);
    simRegisterScriptCallbackFunction(LUA_SUBTRACTWORKIMGFROMBUFFER1_COMMAND_PLUGIN,nullptr,LUA_SUBTRACTWORKIMGFROMBUFFER1_CALLBACK);
    simRegisterScriptCallbackFunction(LUA_ADDBUFFER1TOWORKIMG_COMMAND_PLUGIN,nullptr,LUA_ADDBUFFER1TOWORKIMG_CALLBACK);
    simRegisterScriptCallbackFunction(LUA_SUBTRACTBUFFER1FROMWORKIMG_COMMAND_PLUGIN,nullptr,LUA_SUBTRACTBUFFER1FROMWORKIMG_CALLBACK);
    simRegisterScriptCallbackFunction(LUA_MULTIPLYWORKIMGWITHBUFFER1_COMMAND_PLUGIN,nullptr,LUA_MULTIPLYWORKIMGWITHBUFFER1_CALLBACK);
    simRegisterScriptCallbackFunction(LUA_HORIZONTALFLIPWORKIMG_COMMAND_PLUGIN,nullptr,LUA_HORIZONTALFLIPWORKIMG_CALLBACK);
    simRegisterScriptCallbackFunction(LUA_VERTICALFLIPWORKIMG_COMMAND_PLUGIN,nullptr,LUA_VERTICALFLIPWORKIMG_CALLBACK);
    simRegisterScriptCallbackFunction(LUA_UNIFORMIMGTOWORKIMG_COMMAND_PLUGIN,nullptr,LUA_UNIFORMIMGTOWORKIMG_CALLBACK);
    simRegisterScriptCallbackFunction(LUA_NORMALIZEWORKIMG_COMMAND_PLUGIN,nullptr,LUA_NORMALIZEWORKIMG_CALLBACK);
    simRegisterScriptCallbackFunction(LUA_COLORSEGMENTATIONONWORKIMG_COMMAND_PLUGIN,nullptr,LUA_COLORSEGMENTATIONONWORKIMG_CALLBACK);
    simRegisterScriptCallbackFunction(LUA_INTENSITYSCALEONWORKIMG_COMMAND_PLUGIN,nullptr,LUA_INTENSITYSCALEONWORKIMG_CALLBACK);
    simRegisterScriptCallbackFunction(LUA_SELECTIVECOLORONONWORKIMG_COMMAND_PLUGIN,nullptr,LUA_SELECTIVECOLORONONWORKIMG_CALLBACK);
    simRegisterScriptCallbackFunction(LUA_SCALEANDOFFSETWORKIMG_COMMAND_PLUGIN,nullptr,LUA_SCALEANDOFFSETWORKIMG_CALLBACK);
    simRegisterScriptCallbackFunction(LUA_BINARYWORKIMG_COMMAND_PLUGIN,nullptr,LUA_BINARYWORKIMG_CALLBACK);
    simRegisterScriptCallbackFunction(LUA_BLOBDETECTIONONWORKIMG_COMMAND_PLUGIN,nullptr,LUA_BLOBDETECTIONONWORKIMG_CALLBACK);
    simRegisterScriptCallbackFunction(LUA_SHARPENWORKIMG_COMMAND_PLUGIN,nullptr,LUA_SHARPENWORKIMG_CALLBACK);
    simRegisterScriptCallbackFunction(LUA_EDGEDETECTIONONWORKIMG_COMMAND_PLUGIN,nullptr,LUA_EDGEDETECTIONONWORKIMG_CALLBACK);
    simRegisterScriptCallbackFunction(LUA_SHIFTWORKIMG_COMMAND_PLUGIN,nullptr,LUA_SHIFTWORKIMG_CALLBACK);
    simRegisterScriptCallbackFunction(LUA_CIRCULARCUTWORKIMG_COMMAND_PLUGIN,nullptr,LUA_CIRCULARCUTWORKIMG_CALLBACK);
    simRegisterScriptCallbackFunction(LUA_RESIZEWORKIMG_COMMAND_PLUGIN,nullptr,LUA_RESIZEWORKIMG_CALLBACK);
    simRegisterScriptCallbackFunction(LUA_ROTATEWORKIMG_COMMAND_PLUGIN,nullptr,LUA_ROTATEWORKIMG_CALLBACK);
    simRegisterScriptCallbackFunction(LUA_MATRIX3X3ONWORKIMG_COMMAND_PLUGIN,nullptr,LUA_MATRIX3X3ONWORKIMG_CALLBACK);
    simRegisterScriptCallbackFunction(LUA_MATRIX5X5ONWORKIMG_COMMAND_PLUGIN,nullptr,LUA_MATRIX5X5ONWORKIMG_CALLBACK);
    simRegisterScriptCallbackFunction(LUA_RECTANGULARCUTWORKIMG_COMMAND_PLUGIN,nullptr,LUA_RECTANGULARCUTWORKIMG_CALLBACK);
    simRegisterScriptCallbackFunction(LUA_DISTORT_COMMAND_PLUGIN,nullptr,LUA_DISTORT_CALLBACK);
    simRegisterScriptCallbackFunction(LUA_COORDINATESFROMWORKIMG_COMMAND_PLUGIN,nullptr,LUA_COORDINATESFROMWORKIMG_CALLBACK);
    simRegisterScriptCallbackFunction(LUA_CHANGEDPIXELSONWORKIMG_COMMAND_PLUGIN,nullptr,LUA_CHANGEDPIXELSONWORKIMG_CALLBACK);
    simRegisterScriptCallbackFunction(LUA_VELODYNEDATAFROMWORKIMG_COMMAND_PLUGIN,nullptr,LUA_VELODYNEDATAFROMWORKIMG_CALLBACK);

    visionContainer = new CVisionCont();
    visionTransfContainer = new CVisionTransfCont();
    visionRemapContainer = new CVisionRemapCont();
    visionVelodyneHDL64EContainer = new CVisionVelodyneHDL64ECont();
    visionVelodyneVPL16Container = new CVisionVelodyneVPL16Cont();

    return(6);  // initialization went fine, we return the version number of this extension module (can be queried with simGetModuleName)
                // Version 2 since 3.2.1
                // Version 3 since 3.3.1
                // Version 4 since 3.4.1
                // Version 5 since 4.3.0
                // Version 6 since 4.6.0
}

SIM_DLLEXPORT void simCleanup()
{
    delete visionVelodyneVPL16Container;
    delete visionVelodyneHDL64EContainer;
    delete visionRemapContainer;
    delete visionTransfContainer;
    delete visionContainer;

    unloadSimLibrary(simLib); // release the library
}

SIM_DLLEXPORT void simMsg(int message,int* auxData,void*)
{
    if (message==sim_message_eventcallback_instancepass)
    {
        /* removed on 19.08.2021, not needed
        if (auxData[0]&1)
            visionTransfContainer->removeInvalidObjects();

        for (std::map<int,CVisionSensorData*>::iterator it=_imgData.begin();it!=_imgData.end();it++)
        {
            if (simIsHandleValid(it->first,-1)==0)
                removeImgData(it->first);
        }
        */
    }

    if (message==sim_message_eventcallback_scriptstatedestroyed)
    {
        visionContainer->removeImageObjectFromScriptHandle(auxData[0]);
        visionTransfContainer->removeObjectFromScriptHandle(auxData[0]);
        visionRemapContainer->removeObjectFromScriptHandle(auxData[0]);
        visionVelodyneHDL64EContainer->removeObjectFromScriptHandle(auxData[0]);
        visionVelodyneVPL16Container->removeObjectFromScriptHandle(auxData[0]);
    }
}

