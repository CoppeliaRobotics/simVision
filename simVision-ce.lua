local codeEditorInfos=[[
simVision.addBuffer1ToWorkImg(int visionSensorHandle)
simVision.addWorkImgToBuffer1(int visionSensorHandle)
bool trigger,buffer packedDataPacket=simVision.binaryWorkImg(int visionSensorHandle,float threshold,float oneProportion,float oneTol,float xCenter,float xCenterTol,float yCenter,float yCenterTol,float orient,float orientTol,float roundness,bool enableTrigger,float[3] overlayColor={1.0,0.0,1.0})
bool trigger,buffer packedDataPacket=simVision.blobDetectionOnWorkImg(int visionSensorHandle,float threshold,float minBlobSize,bool modifyWorkImage,float[3] overlayColor={1.0,0.0,1.0})
simVision.buffer1ToWorkImg(int visionSensorHandle)
simVision.buffer2ToWorkImg(int visionSensorHandle)
bool trigger,buffer packedDataPacket=simVision.changedPixelsOnWorkImg(int visionSensorHandle,float threshold)
simVision.circularCutWorkImg(int visionSensorHandle,float radius,bool copyToBuffer1)
simVision.colorSegmentationOnWorkImg(int visionSensorHandle,float maxColorColorDistance)
bool trigger,buffer packedDataPacket,buffer colorData=simVision.coordinatesFromWorkImg(int visionSensorHandle,int[2] xyPointCount,bool evenlySpacedInAngularSpace,bool returnColorData=false)
int velodyneHandle=simVision.createVelodyneHDL64E(int[4] visionSensorHandles,float frequency,int options=0,int pointSize=2,float[2] coloring_closeFarDist={1,5},float displayScalingFactor=1)
int velodyneHandle=simVision.createVelodyneVPL16(int[4] visionSensorHandles,float frequency,int options=0,int pointSize=2,float[2] coloring_closeFarDist={1,5},float displayScalingFactor=1)
int result=simVision.destroyVelodyneHDL64E(int velodyneHandle)
int result=simVision.destroyVelodyneVPL16(int velodyneHandle)
simVision.distort(int visionSensorHandle,int[] pixelMap=nil,float[] depthScalings=nil)
simVision.edgeDetectionOnWorkImg(int visionSensorHandle,float threshold)
int result=simVision.handleAnaglyphStereo(int passiveVisionSensorHandle,int[2] activeVisionSensorHandles,float[6] leftAndRightColors=nil)
int result=simVision.handleSpherical(int passiveVisionSensorHandleForRGB,int[6] activeVisionSensorHandles,float horizontalAngle,float verticalAngle,int passiveVisionSensorHandleForDepth=-1)
buffer points,buffer colorData=simVision.handleVelodyneHDL64E(int velodyneHandle,float dt)
buffer points,buffer colorData=simVision.handleVelodyneVPL16(int velodyneHandle,float dt)
simVision.horizontalFlipWorkImg(int visionSensorHandle)
simVision.intensityScaleOnWorkImg(int visionSensorHandle,float start,float end,bool greyScale)
simVision.matrix3x3OnWorkImg(int visionSensorHandle,int passes,float multiplier,float[9] matrix=nil)
simVision.matrix5x5OnWorkImg(int visionSensorHandle,int passes,float multiplier,float[25] matrix=nil)
simVision.multiplyWorkImgWithBuffer1(int visionSensorHandle)
simVision.normalizeWorkImg(int visionSensorHandle)
simVision.rectangularCutWorkImg(int visionSensorHandle,float[2] sizes,bool copyToBuffer1)
simVision.resizeWorkImg(int visionSensorHandle,float[2] scaling)
simVision.rotateWorkImg(int visionSensorHandle,float rotationAngle)
simVision.scaleAndOffsetWorkImg(int visionSensorHandle,float[3] preOffset,float[3] scaling,float[3] postOffset,bool rgb)
simVision.selectiveColorOnWorkImg(int visionSensorHandle,float[3] color,float[3] colorTolerance,bool rgb,bool keep,bool removedPartToBuffer1)
simVision.sensorDepthMapToWorkImg(int visionSensorHandle)
simVision.sensorImgToWorkImg(int visionSensorHandle)
simVision.sharpenWorkImg(int visionSensorHandle)
simVision.shiftWorkImg(int visionSensorHandle,float[2] shift,bool wrapAround)
simVision.subtractBuffer1FromWorkImg(int visionSensorHandle)
simVision.subtractWorkImgFromBuffer1(int visionSensorHandle)
simVision.swapBuffers(int visionSensorHandle)
simVision.swapWorkImgWithBuffer1(int visionSensorHandle)
simVision.uniformImgToWorkImg(int visionSensorHandle,float[3] color)
bool trigger,buffer packedDataPacket,buffer colorData=simVision.velodyneDataFromWorkImg(int visionSensorHandle,int[2] xyPointCount,float vAngle,bool returnColorData=false)
simVision.verticalFlipWorkImg(int visionSensorHandle)
simVision.workImgToBuffer1(int visionSensorHandle)
simVision.workImgToBuffer2(int visionSensorHandle)
simVision.workImgToSensorDepthMap(int visionSensorHandle,bool removeBuffer=true)
simVision.workImgToSensorImg(int visionSensorHandle,bool removeBuffer=true)
]]

registerCodeEditorInfos("simVision",codeEditorInfos)