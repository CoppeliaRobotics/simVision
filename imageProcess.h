
#pragma once

class CImageProcess  
{
public:
    CImageProcess();
    virtual ~CImageProcess();
    static double* createRGBImage(int resX,int resY);
    static void copyRGBImage(int resX,int resY,double* imageSource,double* imageDest);
    static void clearRGBImage(int resX,int resY,double* image,double clearRed,double clearGreen,double clearBlue);

    static double* createIntensityImage(int resX,int resY);
    static void copyIntensityImage(int resX,int resY,double* intensImageSource,double* intensImageDest);
    static void clearIntensityImage(int resX,int resY,double* intensImage,double clearIntens);

    static void filter3x3RgbImage(int resX,int resY,double* rgbIn,double* rgbOut,double m[9]);
    static void filter5x5RgbImage(int resX,int resY,double* rgbIn,double* rgbOut,double m[25]);
    static void clampRgbImage(int resX,int resY,double* rgbImage,double lowClamp,double highClamp);

    static void deleteImage(double* image);

    static void rgbImageToIntensityImage(int resX,int resY,double* rgbImage,double* intensImage);
    static void intensityImageToRGBImage(int resX,int resY,double* rgbImage,double* intensImage);

    static void getEdges(int resX,int resY,double* imageSource,double* imageDest);
    static void scaleIntensity(int resX,int resY,double* image,double scaleFactor,bool clampToMax);
    static void scaleRGB(int resX,int resY,double* image,double redFactor,double greenFactor,double blueFactor,bool clampToMax);
    static void boxBlurIntensity(int resX,int resY,double* imageSource,double* imageDest);
    static void boxBlurRGB(int resX,int resY,double* imageSource,double* imageDest);
    static void blurIntensity(int resX,int resY,double* image);
    static void blurRGB(int resX,int resY,double* image);
    static void horizontalFlipRGB(int resX,int resY,double* image);
    static void verticalFlipRGB(int resX,int resY,double* image);

    static void scaleRgbImageWithIntensityImage(int resX,int resY,double* rgbImage,double* intensImage);

    static void keepThresholdIntensity(int resX,int resY,double* image,double thresholdVal,bool keepAbove);
    static void keepThresholdRGB(int resX,int resY,double* image,double thresholdVal,bool keepAbove);
    static void nonZeroToOneIntensity(int resX,int resY,double* image);
    static void nonZeroToOneRGB(int resX,int resY,double* image);
    static void addImagesRGB(int resX,int resY,double* imageInOut,double* imageOverlay);
    static void clampToOneRGB(int resX,int resY,double* image);

    static void predef_lightBlurRGB(int resX,int resY,double* image);
    static void predef_heavyBlurRGB(int resX,int resY,double* image);
    static void predef_getThinEdgesRGB(int resX,int resY,double* image);
    static void predef_getThickEdgesRGB(int resX,int resY,double* image);
    static void predef_getThinEdgeOverlayRGB(int resX,int resY,double* image);
    static void predef_invertRGB(int resX,int resY,double* image);
};
