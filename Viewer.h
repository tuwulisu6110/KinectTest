/*******************************************************************************
*                                                                              *
*   PrimeSense NiTE 2.0 - User Viewer Sample                                   *
*   Copyright (C) 2012 PrimeSense Ltd.                                         *
*                                                                              *
*******************************************************************************/

#ifndef _NITE_USER_VIEWER_H_
#define _NITE_USER_VIEWER_H_

#include "NiTE.h"
#include <math.h>
#define MAX_DEPTH 10000
#define MAX_DEVICE 3
#include"pointf.h"
#include"voxel.h"
#include"mcolor.h"
#include"deviceSwitcher.h"
class SampleViewer
{
public:
	SampleViewer(const char* strSampleName);
	virtual ~SampleViewer();

	virtual openni::Status Init(int argc, char **argv);
	virtual openni::Status Run();	//Does not return

protected:
	virtual void Display();
	virtual void snapDisplay();
	virtual void DisplayPostDraw(){};	// Overload to draw over the screen image

	virtual void OnKey(unsigned char key, int x, int y);
	virtual void MouseEvent(int button,int state,int x,int y);
	virtual void SelectingMouseEvent(int button,int state,int x,int y);
	virtual void MouseMotion(int x,int y);
	virtual void snapShot();
	virtual void findNextSpot();
	virtual void getTheRealSpot();
	virtual void humanDisplay();
	virtual void getTranslateT(int);
	virtual void getRotateR(int);
	virtual void getCalibrationCenter();
	virtual void scaleCalibration();//only for calibration obj
	virtual void getCrossVector();
	void debugDisplay();
	void debugKey(unsigned char key, int x, int y);
	void rotate(float degree,pointf &test,pointf c);


	virtual openni::Status InitOpenGL(int argc, char **argv);
	void InitOpenGLHooks();

	void Finalize();
	openni::VideoStream			m_colorStream[MAX_DEVICE];

private:
	SampleViewer(const SampleViewer&);
	SampleViewer& operator=(SampleViewer&);

	static SampleViewer* ms_self;
	static void glutIdle();
	static void glutDisplay();
	static void glutsnapDisplay();
	static void glutKeyboard(unsigned char key, int x, int y);
	static void glutMouse(int button,int state,int x,int y);
	static void glutMouseMotion(int x,int y);
	bool selectingMode;
	pointf **calibrationRef;
	pointf *calibrationCenter;

	float				m_pDepthHist[MAX_DEPTH];
	char			m_strSampleName[ONI_MAX_STR];
	openni::RGB888Pixel*		m_pTexMap;
	unsigned int		m_nTexMapX;
	unsigned int		m_nTexMapY;

	openni::Device		m_device[MAX_DEVICE];
	nite::UserTracker* m_pUserTracker[MAX_DEVICE];

	nite::UserId m_poseUser;
	uint64_t m_poseTime;

	int deviceNum;
	bool takePicture;
	bool newChange;
	bool buttonState;
	int buttonPos;
	int clickX;
	float* meanZ;
	float* meanX;
	float* meanY;
	float* theta;
	float* thetaKeep;
	float* translateX;
	float* translateY;
	bool soraMode;
	bool viewMode;
	bool shiftMode;
	int viewingID;
	int *trackingID;//for skeleton
	int userDeviceSwitcher;
	int *debugDriftNormal;
	bool debugFullNormalMode;

	openni::VideoFrameRef snapDepthFrame[MAX_DEVICE],snapColorFrame[MAX_DEVICE];
	float nowX,nowY,nowZ;
	int nowDevice;
	int nowColor;
	bool humanDisplayMode;
	bool denseMode;
	bool skeletonCaptured;
	int frameCounter;
	bool rotationMode;
	bool traditionMode;
	bool debugMode;
	bool preDebugMode;


	pointf *translateT;
	vector *realTranslate;
	pointf *rotateR;
	vector *vectorRY,*vectorBG;
	pointf *torso;
	pointf **pointCloud;
	pointf *basePointCloud;
	pointf *humanCenter;
	voxel *volume;
	voxel **volumeZ;
	deviceSwitcher *debugSwitcher;
};


#endif // _NITE_USER_VIEWER_H_
