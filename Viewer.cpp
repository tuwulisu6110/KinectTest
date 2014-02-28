/*******************************************************************************
*                                                                              *
*   PrimeSense NiTE 2.0 - User Viewer Sample                                   *
*   Copyright (C) 2012 PrimeSense Ltd.                                         *
*                                                                              *
*******************************************************************************/
#include <fstream> 
#include <math.h>
#if (defined _WIN32)
#define PRIu64 "llu"
#else
#define __STDC_FORMAT_MACROS
#include <inttypes.h>
#endif

#include "Viewer.h"

#if (ONI_PLATFORM == ONI_PLATFORM_MACOSX)
        #include <GLUT/glut.h>
#else
        #include <GL/glut.h>
#endif

#include <NiteSampleUtilities.h>


#define BASE 0
#define VOLUME_X 500
#define VOLUME_Y 500
#define GL_WIN_SIZE_X	1024
#define GL_WIN_SIZE_Y	768
#define TEXTURE_SIZE	512
#define DEPTH_WIDTH 640
#define DEPTH_HEIGHT 480
#define SHOW_DATA_PER_FRAME 30

#define DEFAULT_DISPLAY_MODE	DISPLAY_MODE_DEPTH

#define MIN_NUM_CHUNKS(data_size, chunk_size)	((((data_size)-1) / (chunk_size) + 1))
#define MIN_CHUNKS_SIZE(data_size, chunk_size)	(MIN_NUM_CHUNKS(data_size, chunk_size) * (chunk_size))
#define View_Distance 60
SampleViewer* SampleViewer::ms_self = NULL;

bool g_drawSkeleton = true;
bool g_drawCenterOfMass = false;
bool g_drawStatusLabel = true;
bool g_drawBoundingBox = false;
bool g_drawBackground = true;
bool g_drawDepth = true;
bool g_drawFrameId = false;

int g_nXRes = 0, g_nYRes = 0;

// time to hold in pose to exit program. In milliseconds.
const int g_poseTimeoutToExit = 2000;
const float trueFactor = 20;

void SampleViewer::glutIdle()
{
	
	if(ms_self->takePicture)
	{
		ms_self->snapShot();
	}
	if(!ms_self->selectingMode||ms_self->newChange)
	{
		glutPostRedisplay();
		ms_self->newChange = false;
	}
}

void SampleViewer::glutDisplay()
{
	if(ms_self->humanDisplayMode)
	{
		SampleViewer::ms_self->humanDisplay();
	}
	else
	{
		if(ms_self->selectingMode)
			SampleViewer::ms_self->snapDisplay();
		else
			SampleViewer::ms_self->Display();
	}
}
void SampleViewer::glutKeyboard(unsigned char key, int x, int y)
{
	SampleViewer::ms_self->OnKey(key, x, y);
}
void SampleViewer::glutMouse(int button,int state,int x,int y)
{
	if(!ms_self->selectingMode)
		SampleViewer::ms_self->MouseEvent(button,state,x,y);
	else
		SampleViewer::ms_self->SelectingMouseEvent(button,state,x,y);

}
void SampleViewer::glutMouseMotion(int x,int y)
{
	SampleViewer::ms_self->MouseMotion(x,y);
}

SampleViewer::SampleViewer(const char* strSampleName) : m_poseUser(0)
{
	ms_self = this;
	strncpy(m_strSampleName, strSampleName, ONI_MAX_STR);
	meanZ = new float[MAX_DEVICE];
	meanX = new float[MAX_DEVICE];
	meanY = new float[MAX_DEVICE];
	theta = new float[MAX_DEVICE];
	thetaKeep = new float[MAX_DEVICE];
	translateX = new float[MAX_DEVICE];
	translateY = new float[MAX_DEVICE];
	translateT = new pointf[MAX_DEVICE];
	realTranslate = new vector[MAX_DEVICE];
	rotateR = new pointf[MAX_DEVICE];
	calibrationCenter = new pointf[MAX_DEVICE];
	for(int i=0;i<MAX_DEVICE;i++)
	{
		m_pUserTracker[i] = new nite::UserTracker;
		theta[i]=0;
		thetaKeep[i]=0;
		translateX[i] = 0;
		translateY[i] = 0;
		calibrationCenter[i].setToZero();
		rotateR[i].setToZero();
		translateT[i].setToZero();
	}
	soraMode = false;
	viewMode = true;
	takePicture = false;
	selectingMode = false;
	traditionMode = true;
	newChange = false;
	viewingID = 0;
	humanDisplayMode = false;
	scaleMode = true;
	denseMode = true;
	rotationMode = false;
	nowX=0;
	nowY=0;
	nowDevice = 0;
	nowColor = 0;
	trackingID = new int[MAX_DEVICE];
	userDeviceSwitcher = 0;

	torso = new pointf[MAX_DEVICE];
	skeletonCaptured = false;
	vectorRY = new vector[MAX_DEVICE];
	vectorBG = new vector[MAX_DEVICE];
	calibrationRef = new pointf*[MAX_DEVICE];
	pointCloud = new pointf*[MAX_DEVICE];
	basePointCloud = new pointf[DEPTH_WIDTH*DEPTH_HEIGHT];
	for(int i=0;i<MAX_DEVICE;i++)
	{
		calibrationRef[i] = new pointf[4];
		trackingID[i]=0;
		pointCloud[i] = new pointf[DEPTH_WIDTH*DEPTH_HEIGHT];
	}
	humanCenter = new pointf[MAX_DEVICE];
	volume = new voxel[VOLUME_X*VOLUME_Y];

}
SampleViewer::~SampleViewer()
{
	Finalize();

	delete[] m_pTexMap;
	delete[] meanZ;
	delete[] meanX;
	delete[] meanY;
	delete[] theta;
	delete[] thetaKeep;
	delete[] translateX;
	delete[] translateY;
	delete[] translateT;
	delete[] rotateR;
	delete[] calibrationCenter;
	delete[] vectorRY;
	delete[] vectorBG;
	for(int i=0;i<deviceNum;i++)
		delete calibrationRef[i];
	delete calibrationRef;
	delete torso;
	ms_self = NULL;
	delete pointCloud;
	delete realTranslate;
	delete humanCenter;
	delete [] volume;
}

void SampleViewer::Finalize()
{
	delete m_pUserTracker;
	nite::NiTE::shutdown();
	openni::OpenNI::shutdown();
}

openni::Status SampleViewer::Init(int argc, char **argv)
{
	m_pTexMap = NULL;

	openni::Status rc = openni::OpenNI::initialize();
	if (rc != openni::STATUS_OK)
	{
		printf("Failed to initialize OpenNI\n%s\n", openni::OpenNI::getExtendedError());
		return rc;
	}

	openni::Array<openni::DeviceInfo> deviceList;
	openni::OpenNI::enumerateDevices(&deviceList);
	const char* deviceUri[MAX_DEVICE];

	nite::NiTE::initialize();

	printf("device number : %d\n",deviceList.getSize());
	deviceNum = deviceList.getSize();
	for(int i=0;i<deviceList.getSize();i++)
	{
		deviceUri[i] = deviceList[i].getUri();
		rc = m_device[i].open(deviceUri[i]);
		if (rc != openni::STATUS_OK)
		{
			printf("Failed to open device\n%s\n", openni::OpenNI::getExtendedError());
			return rc;
		}	
		rc = m_colorStream[i].create(m_device[i], openni::SENSOR_COLOR);
		if (rc == openni::STATUS_OK)
		{
			rc = m_colorStream[i].start();
			if (rc != openni::STATUS_OK)
			{
				printf("SimpleViewer: Couldn't start color stream%d:\n%s\n", i,openni::OpenNI::getExtendedError());
				m_colorStream[i].destroy();
			}
		}
		else
		{
			printf("SimpleViewer: Couldn't find color stream:\n%s\n", openni::OpenNI::getExtendedError());
		}
		if (m_pUserTracker[i]->create(&m_device[i]) != nite::STATUS_OK)
		{
			return openni::STATUS_ERROR;
		}
		if(m_device[i].isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR))
		{
			printf("its support\n");
			m_device[i].setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
		}
	}


	/*const char* deviceUri = openni::ANY_DEVICE;
	for (int i = 1; i < argc-1; ++i)
	{
		if (strcmp(argv[i], "-device") == 0)
		{
			deviceUri = argv[i+1];
			break;
		}
	}*/

	






	



	return InitOpenGL(argc, argv);

}
openni::Status SampleViewer::Run()	//Does not return
{
	glutMainLoop();

	return openni::STATUS_OK;
}

float Colors[][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 1, 1}};
int colorCount = 3;
static float zFactor = 20 , xFactor = 10 , yFactor = 10;

#define MAX_USERS 10
bool g_visibleUsers[MAX_USERS] = {false};
nite::SkeletonState g_skeletonStates[MAX_USERS] = {nite::SKELETON_NONE};
char g_userStatusLabels[MAX_USERS][100] = {{0}};

char g_generalMessage[100] = {0};

#define USER_MESSAGE(msg) {\
	sprintf(g_userStatusLabels[user.getId()], "%s", msg);\
	printf("[%08" PRIu64 "] User #%d:\t%s\n", ts, user.getId(), msg);}

void updateUserState(const nite::UserData& user, uint64_t ts)
{
	if (user.isNew())
	{
		USER_MESSAGE("New");
	}
	else if (user.isVisible() && !g_visibleUsers[user.getId()])
		printf("[%08" PRIu64 "] User #%d:\tVisible\n", ts, user.getId());
	else if (!user.isVisible() && g_visibleUsers[user.getId()])
		printf("[%08" PRIu64 "] User #%d:\tOut of Scene\n", ts, user.getId());
	else if (user.isLost())
	{
		USER_MESSAGE("Lost");
	}
	g_visibleUsers[user.getId()] = user.isVisible();


	if(g_skeletonStates[user.getId()] != user.getSkeleton().getState())
	{
		switch(g_skeletonStates[user.getId()] = user.getSkeleton().getState())
		{
		case nite::SKELETON_NONE:
			USER_MESSAGE("Stopped tracking.")
			break;
		case nite::SKELETON_CALIBRATING:
			USER_MESSAGE("Calibrating...")
			break;
		case nite::SKELETON_TRACKED:
			USER_MESSAGE("Tracking!")
			break;
		case nite::SKELETON_CALIBRATION_ERROR_NOT_IN_POSE:
		case nite::SKELETON_CALIBRATION_ERROR_HANDS:
		case nite::SKELETON_CALIBRATION_ERROR_LEGS:
		case nite::SKELETON_CALIBRATION_ERROR_HEAD:
		case nite::SKELETON_CALIBRATION_ERROR_TORSO:
			USER_MESSAGE("Calibration Failed... :-|")
			break;
		}
	}
}

#ifndef USE_GLES
void glPrintString(void *font, const char *str)
{
	int i,l = (int)strlen(str);

	for(i=0; i<l; i++)
	{   
		glutBitmapCharacter(font,*str++);
	}   
}
#endif
void DrawStatusLabel(nite::UserTracker* pUserTracker, const nite::UserData& user)
{
	int color = user.getId() % colorCount;
	glColor3f(1.0f - Colors[color][0], 1.0f - Colors[color][1], 1.0f - Colors[color][2]);

	float x,y;
	pUserTracker->convertJointCoordinatesToDepth(user.getCenterOfMass().x, user.getCenterOfMass().y, user.getCenterOfMass().z, &x, &y);
	x *= GL_WIN_SIZE_X/(float)g_nXRes;
	y *= GL_WIN_SIZE_Y/(float)g_nYRes;
	char *msg = g_userStatusLabels[user.getId()];
	glRasterPos2i(x-((strlen(msg)/2)*8),y);
	glPrintString(GLUT_BITMAP_HELVETICA_18, msg);
}
void DrawFrameId(int frameId)
{
	char buffer[80] = "";
	sprintf(buffer, "%d", frameId);
	glColor3f(1.0f, 0.0f, 0.0f);
	glRasterPos2i(20, 20);
	glPrintString(GLUT_BITMAP_HELVETICA_18, buffer);
}
void DrawCenterOfMass(nite::UserTracker* pUserTracker, const nite::UserData& user)
{
	glColor3f(1.0f, 1.0f, 1.0f);

	float coordinates[3] = {0};

	pUserTracker->convertJointCoordinatesToDepth(user.getCenterOfMass().x, user.getCenterOfMass().y, user.getCenterOfMass().z, &coordinates[0], &coordinates[1]);

	coordinates[0] *= GL_WIN_SIZE_X/(float)g_nXRes;
	coordinates[1] *= GL_WIN_SIZE_Y/(float)g_nYRes;
	glPointSize(8);
	glVertexPointer(3, GL_FLOAT, 0, coordinates);
	glDrawArrays(GL_POINTS, 0, 1);

}
void DrawBoundingBox(const nite::UserData& user)
{
	glColor3f(1.0f, 1.0f, 1.0f);

	float coordinates[] =
	{
		user.getBoundingBox().max.x, user.getBoundingBox().max.y, 0,
		user.getBoundingBox().max.x, user.getBoundingBox().min.y, 0,
		user.getBoundingBox().min.x, user.getBoundingBox().min.y, 0,
		user.getBoundingBox().min.x, user.getBoundingBox().max.y, 0,
	};
	coordinates[0]  *= GL_WIN_SIZE_X/(float)g_nXRes;
	coordinates[1]  *= GL_WIN_SIZE_Y/(float)g_nYRes;
	coordinates[3]  *= GL_WIN_SIZE_X/(float)g_nXRes;
	coordinates[4]  *= GL_WIN_SIZE_Y/(float)g_nYRes;
	coordinates[6]  *= GL_WIN_SIZE_X/(float)g_nXRes;
	coordinates[7]  *= GL_WIN_SIZE_Y/(float)g_nYRes;
	coordinates[9]  *= GL_WIN_SIZE_X/(float)g_nXRes;
	coordinates[10] *= GL_WIN_SIZE_Y/(float)g_nYRes;

	glPointSize(2);
	glVertexPointer(3, GL_FLOAT, 0, coordinates);
	glDrawArrays(GL_LINE_LOOP, 0, 4);

}






void SampleViewer::snapShot()
{
	nite::UserTrackerFrameRef userTrackerFrame[MAX_DEVICE];
	//openni::VideoFrameRef depthFrame[MAX_DEVICE],colorFrame[MAX_DEVICE];


	for(int i=0;i<deviceNum;i++)
	{
		if(nowDevice == i)
		{
			nite::Status rc = m_pUserTracker[i]->readFrame(&userTrackerFrame[i]);
			//printf("read %d\n",i);
			if (rc != nite::STATUS_OK)
			{
				printf("GetNextData failed\n");
				return;
			}
			m_colorStream[i].readFrame(&snapColorFrame[i]);
			snapDepthFrame[i] = userTrackerFrame[i].getDepthFrame();
		}
		/*if (m_pTexMap == NULL)
		{
			// Texture map init
			m_nTexMapX = MIN_CHUNKS_SIZE(depthFrame[i].getVideoMode().getResolutionX(), TEXTURE_SIZE);
			m_nTexMapY = MIN_CHUNKS_SIZE(depthFrame[i].getVideoMode().getResolutionY(), TEXTURE_SIZE);
			m_pTexMap = new openni::RGB888Pixel[m_nTexMapX * m_nTexMapY];
		}*/
	}
	

	takePicture = false;
	selectingMode = true;
	findNextSpot();
	printf("width = %d , height = %d\n",snapDepthFrame[nowDevice].getWidth(),snapDepthFrame[nowDevice].getHeight());
}
static float Brb = 0.353,Brg = 0.693,Grb=0.40122,Grg=0.40871,Yrb = 34.2727 , Yrg = 1.1013 , Rrb = 2.67 , Rrg = 5.271;
static float ub = 1.1, lb = 0.9;

void SampleViewer::findNextSpot()
{
	if (snapDepthFrame[nowDevice].isValid() && g_drawDepth && snapColorFrame[nowDevice].isValid())
	{
		//printf("ha i ri ta\n");
		const openni::DepthPixel* pDepth = (const openni::DepthPixel*)snapDepthFrame[nowDevice].getData();
		const openni::RGB888Pixel* pImage = (const openni::RGB888Pixel*)snapColorFrame[nowDevice].getData();
		int rowSize = snapDepthFrame[nowDevice].getStrideInBytes() / sizeof(openni::DepthPixel);

		bool find = false;
		for (int y = nowY; y < snapDepthFrame[nowDevice].getHeight(); ++y)
		{
			int xx;
			if(y==nowY)
				xx=nowX;
			else
				xx=0;
			for (int x = xx; x < snapDepthFrame[nowDevice].getWidth(); ++x)
			{
				if(*(pDepth+y*640+x)==0)
					continue;
				float rb,rg;
				rg=((float)((pImage+y*640+x)->r))/(float)((pImage+y*640+x)->g);
				rb=((float)((pImage+y*640+x)->r))/(float)((pImage+y*640+x)->b);
				switch (nowColor)
				{
				case 0://Y
					if(rg/Yrg<ub&&rg/Yrg>lb&&rb/Yrb<ub&&rb/Yrb>lb)
					//if((pImage+y*640+x)->r>130&&(pImage+y*640+x)->g>130&&(pImage+y*640+x)->b<30)
					{
						if(abs(nowX-x)>5||abs(nowY-y)>10)
						{
							//printf("u fund r = %d,g = %d, b = %d\n",(pImage+y*640+x)->r,(pImage+y*640+x)->g,(pImage+y*640+x)->b);
							nowX=x;
							nowY=y;
							nowZ=*(pDepth+y*640+x);
							find = true;
						}
					}
					break;
				case 1://R
					if(rg/Rrg<ub+0.05&&rg/Rrg>lb-0.05&&rb/Rrb<ub+0.05&&rb/Rrb>lb-0.05)
					//if((pImage+y*640+x)->r>140&&(pImage+y*640+x)->g<70&&(pImage+y*640+x)->b<70)
					{
						if(abs(nowX-x)>5||abs(nowY-y)>10)
						{
							//printf("u fund r = %d,g = %d, b = %d\n",(pImage+y*640+x)->r,(pImage+y*640+x)->g,(pImage+y*640+x)->b);
							nowX=x;
							nowY=y;
							nowZ=*(pDepth+y*640+x);
							find = true;
						}
					}
					break;
				case 2://G
					if(rg/Grg<ub&&rg/Grg>lb&&rb/Grb<ub&&rb/Grb>lb)
					//if((pImage+y*640+x)->r>=31.6&&(pImage+y*640+x)->r<=51.6&&(pImage+y*640+x)->g>=90.7&&(pImage+y*640+x)->g<=102.1&&(pImage+y*640+x)->b>=92.2&&(pImage+y*640+x)->b<=104.2)
					//if((pImage+y*640+x)->r<60&&(pImage+y*640+x)->g>100&&(pImage+y*640+x)->b<60)
					//if((pImage+y*640+x)->g>150)
					{
						if(abs(nowX-x)>5||abs(nowY-y)>10)
						{
							
							nowX=x;
							nowY=y;
							nowZ=*(pDepth+y*640+x);
							find = true;
						}
					}
					break;
				case 3://B
					if(rg/Brg<ub&&rg/Brg>lb&&rb/Brb<ub&&rb/Brb>lb)
					//if((pImage+y*640+x)->r>=31.6&&(pImage+y*640+x)->r<=51.6&&(pImage+y*640+x)->g>=90.7&&(pImage+y*640+x)->g<=102.1&&(pImage+y*640+x)->b>=92.2&&(pImage+y*640+x)->b<=104.2)
					{
						if(abs(nowX-x)>5||abs(nowY-y)>10)
						{
							//printf("u fund r = %d,g = %d, b = %d\n",(pImage+y*640+x)->r,(pImage+y*640+x)->g,(pImage+y*640+x)->b);
							nowX=x;
							nowY=y;
							nowZ=*(pDepth+y*640+x);
							find = true;
						}
					}
					break;
				}
				if(find == true)
					break;
			}
			if(find == true)
				break;
		}
		if(find == false)
		{
			nowX = 0;
			nowY = 0;
			nowZ = 0;
			//findNextSpot();
		}
		
		newChange = true;
	}
	printf("nowX = %f , nowY = %f\n",nowX,nowY);
}

bool onajiColor(const openni::RGB888Pixel* core,const openni::RGB888Pixel* test)
{
	if(abs(core->r-test->r)<40&&abs(core->g-test->g)<40&&abs(core->b-test->b)<40)
		return true;
	else
		return false;
}
bool onajiDepth(const openni::DepthPixel* core,const openni::DepthPixel* test)
{
	if(abs(*core-*test)<50)
		return true;
	else
		return false;
}

void SampleViewer::getTheRealSpot()
{
	
	{
		for(int i=0;i<deviceNum;i++)
		{
			const openni::DepthPixel* pDepth = (const openni::DepthPixel*)snapDepthFrame[i].getData();
			const openni::RGB888Pixel* pImage = (const openni::RGB888Pixel*)snapColorFrame[i].getData();
			if (snapDepthFrame[i].isValid() && snapColorFrame[i].isValid())
			{
				for(int j=0;j<4;j++)
				{
					int originX=(int)calibrationRef[i][j].x;
					int originY=(int)calibrationRef[i][j].y;
					float sumX=0,sumY=0,sumZ=0,sumNum=0;
					//printf("%d origin : r = %d , g = %d , b = %d , x = %d, y = %d , z = %d\n",j,pImage[640*originY+originX].r,pImage[640*originY+originX].g,pImage[640*originY+originX].b,originX,originY,*(pDepth+640*originY+originX));
					for(int y=originY-15,id=0;y<originY+15;y++)
						for(int x=originX-15;x<originX+15;x++)
						{
							
							if(x<0||y<0||y>=480||x>=640)
								continue;
							//printf("id %d : r = %d , g = %d , b = %d , z = %d\n",id++,pImage[640*y+x].r,pImage[640*y+x].g,pImage[640*y+x].b,*(pDepth+640*y+x));
							if(/*onajiColor(pImage+640*originY+originX,pImage+640*y+x)&&*/pDepth[640*y+x]!=0&&onajiDepth(pDepth+640*originY+originX,pDepth+640*y+x))
							{
								sumX+=x;
								sumY+=y;
								sumZ+=pDepth[640*y+x];
								sumNum++;
							}
						}
					float x = sumX/sumNum,y=sumY/sumNum,z=sumZ/sumNum,realx,realy;
					m_pUserTracker[i]->convertDepthCoordinatesToJoint(x,y,z,&realx,&realy);
					//glVertex3f(realx/60+xShifter[i],realy/60,pDepth[0]/60);

					calibrationRef[i][j].x = realx/trueFactor;
					calibrationRef[i][j].y = realy/trueFactor;
					calibrationRef[i][j].z = z/trueFactor;
					printf("device %d color %d : x=%f , y=%f , z=%f , sum=%f\n",i,j,calibrationRef[i][j].x,calibrationRef[i][j].y,calibrationRef[i][j].z,sumNum);
					//system("pause");
				}
			}
			else
				printf("ERROR in getTheRealSpot\n");
		}
	}
}

void DrawCircle(float cx, float cy,float cz, float r, int num_segments) 
{ 
	glBegin(GL_LINE_LOOP); 
	for(int ii = 0; ii < num_segments; ii++) 
	{ 
		float theta = 2.0f * 3.1415926f * float(ii) / float(num_segments);//get the current angle 

		float x = r * cosf(theta);//calculate the x component 
		float y = r * sinf(theta);//calculate the y component 

		glVertex3f(x + cx, y + cy, cz);//output vertex 

	} 
	glEnd(); 
}

void SampleViewer::snapDisplay()
{
	//printf("draw");
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	if(viewMode)
		gluPerspective( /* field of view in degree */ View_Distance,
		/* aspect ratio */ 1.0,
		/* Z near */ 1, /* Z far */ 10000);
	else
		glOrtho(-80,80,-60,60,0,10000);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	//gluLookAt(0 ,0 ,0,  /* eye is at () */
	//	0.0,0.0 , View_Distance,      /* center is at (0,0,0) */
	//	0.0, -1.0,0.0); 
	gluLookAt(0 ,0 ,0,  /* eye is at () */
		0.0,0 , 1,      /* center is at (0,0,0) */
		0.0, -1.0,0.0); 
	float factor[3] = {1, 1, 1};
	float xShifter[3] = {-30,30,40};
	// check if we need to draw depth frame to texture
	for(int i=0;i<deviceNum;i++)
	{
		int sumZ = 0,sumX = 0,sumY = 0;
		int num = 0;
		if (snapColorFrame[i].isValid() && g_drawDepth && snapColorFrame[i].isValid())
		{
			//printf("Drawing : %d\n",i);
			//const nite::UserMap& userLabels = userTrackerFrame[i].getUserMap();
			//const nite::UserId* pLabels = userLabels.getPixels();

			const openni::DepthPixel* pDepthRow = (const openni::DepthPixel*)snapDepthFrame[i].getData();
			const openni::RGB888Pixel* pImageRow = (const openni::RGB888Pixel*)snapColorFrame[i].getData();
			//openni::RGB888Pixel* pTexRow = m_pTexMap + depthFrame.getCropOriginY() * m_nTexMapX;
			int rowSize = snapDepthFrame[i].getStrideInBytes() / sizeof(openni::DepthPixel);

			for (int y = 0; y < snapDepthFrame[i].getHeight(); ++y)
			{
				const openni::DepthPixel* pDepth = pDepthRow;
				const openni::RGB888Pixel* pImage = pImageRow;
				//openni::RGB888Pixel* pTex = pTexRow + depthFrame.getCropOriginX();

				for (int x = 0; x < snapDepthFrame[i].getWidth(); ++x, ++pDepth, ++pImage)
				{
					if (*pDepth != 0)
					{
						//if (*pLabels != 0)
						{
							//glColor3f(pImage->r/225.0,pImage->g/225.0,pImage->b/225.0);
							//int nHistValue = m_pDepthHist[*pDepth];
						
							//glVertex3f(x/10.0-24+xShifter[i]-10,y/10.0-32,pDepth[0]/10);
							sumZ+=pDepth[0]/zFactor;
							sumX+=x;
							sumY+=y;
							num++;
						}
					}
				}
				pImageRow += rowSize;
				pDepthRow += rowSize;
				//pTexRow += m_nTexMapX;
			}
		}
		if(num!=0)
		{
			//meanZ[i] = ((float)sumZ)/num;
			meanX[i] = ((float)sumX)/num;
			meanY[i] = ((float)sumY)/num;
			const openni::DepthPixel* pDepthRow = (const openni::DepthPixel*)snapDepthFrame[i].getData();
			meanZ[i] = pDepthRow[snapDepthFrame[i].getWidth()*((int)meanY[i])+(int)meanX[i]]/10;
		}
		else
		{
			meanZ[i]=0;
		}
		
		//printf("meanZ : %f\n",meanZ);

		if (snapDepthFrame[i].isValid() && g_drawDepth && snapColorFrame[i].isValid())
		{
			//printf("Drawing : %d\n",i);
			//const nite::UserMap& userLabels = userTrackerFrame[i].getUserMap();
			//const nite::UserId* pLabels = userLabels.getPixels();

			const openni::DepthPixel* pDepthRow = (const openni::DepthPixel*)snapDepthFrame[i].getData();
			const openni::RGB888Pixel* pImageRow = (const openni::RGB888Pixel*)snapColorFrame[i].getData();
			//openni::RGB888Pixel* pTexRow = m_pTexMap + depthFrame.getCropOriginY() * m_nTexMapX;
			int rowSize = snapDepthFrame[i].getStrideInBytes() / sizeof(openni::DepthPixel);

			glPushMatrix();
			glTranslatef(translateX[i],translateY[i],0);
			glTranslatef(xShifter[i],0,meanZ[i]);
			if(soraMode)
				glRotatef(90,1,0,0);
			glRotatef(theta[i],0,1,0);
			glTranslatef(-(xShifter[i]),0,-meanZ[i]);

			glBegin(GL_POINTS);  
			for (int y = 0; y < snapDepthFrame[i].getHeight(); ++y)
			{
				const openni::DepthPixel* pDepth = pDepthRow;
				const openni::RGB888Pixel* pImage = pImageRow;
				//openni::RGB888Pixel* pTex = pTexRow + depthFrame.getCropOriginX();

				for (int x = 0; x < snapDepthFrame[i].getWidth(); ++x, ++pDepth,++pImage)
				{
					if (*pDepth != 0)
					{

							glColor3f(pImage->r/225.0,pImage->g/225.0,pImage->b/225.0);
							//int nHistValue = m_pDepthHist[*pDepth];
							
							glVertex3f(x/10.0-24+xShifter[i]-10,y/10.0-32,pDepth[0]/zFactor);
							//printf("trues");
							//DrawCircle();
					}
				}
				pImageRow += rowSize;
				pDepthRow += rowSize;
				//pTexRow += m_nTexMapX;
			}
			glEnd();
			glPopMatrix();
		}
		
	}
	switch(nowColor)
	{
	case 0:
		glColor3f(1.0,1.0,0.0);
		break;
	case 1:
		glColor3f(1.0,0.0,0.0);
		break;
	case 2:
		glColor3f(0.0,1.0,0.0);
		break;
	case 3:
		glColor3f(0.0,0.0,1.0);
		break;

	}
	//glColor3f(1.0,1.0,0.0);
	//glLineWidth(10);
	//DrawCircle(nowX/10.0-24+xShifter[nowDevice]-10,nowY/10.0-32,2,2,300);
	//DrawCircle(50,50,1,10,300);
	glPointSize(7);
	glBegin(GL_POINTS);
		glVertex3f(nowX/10.0-24+xShifter[nowDevice]-10,nowY/10.0-32,2);
		//glVertex3f(10,10,1);
	glEnd();
	glPointSize(1);

	int y = nowY , x = nowX;
	const openni::RGB888Pixel* pImage = (const openni::RGB888Pixel*)snapColorFrame[nowDevice].getData();
	const openni::DepthPixel* pDepth = (const openni::DepthPixel*)snapDepthFrame[nowDevice].getData();
	printf("u fund r = %d,g = %d, b = %d , d = %d\n",(pImage+y*640+x)->r,(pImage+y*640+x)->g,(pImage+y*640+x)->b,pDepth[640*y+x]);

	glutSwapBuffers();
}



void SampleViewer::Display()
{
	//system("pause");
	nite::UserTrackerFrameRef userTrackerFrame[MAX_DEVICE];
	openni::VideoFrameRef depthFrame[MAX_DEVICE],colorFrame[MAX_DEVICE];
	for(int i=0;i<deviceNum;i++)
	{
		nite::Status rc = m_pUserTracker[i]->readFrame(&userTrackerFrame[i]);
		//printf("read %d\n",i);
		if (rc != nite::STATUS_OK)
		{
			printf("GetNextData failed\n");
			return;
		}
		m_colorStream[i].readFrame(&colorFrame[i]);
		depthFrame[i] = userTrackerFrame[i].getDepthFrame();
		
		/*if (m_pTexMap == NULL)
		{
			// Texture map init
			m_nTexMapX = MIN_CHUNKS_SIZE(depthFrame[i].getVideoMode().getResolutionX(), TEXTURE_SIZE);
			m_nTexMapY = MIN_CHUNKS_SIZE(depthFrame[i].getVideoMode().getResolutionY(), TEXTURE_SIZE);
			m_pTexMap = new openni::RGB888Pixel[m_nTexMapX * m_nTexMapY];
		}*/
	}


	


	//const nite::UserMap& userLabels = userTrackerFrame.getUserMap();

	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	/*glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0, GL_WIN_SIZE_X, GL_WIN_SIZE_Y, 0, -1.0, 1.0);*/

	/*if (depthFrame.isValid() && g_drawDepth)
	{
		calculateHistogram(m_pDepthHist, MAX_DEPTH, depthFrame);
	}*/


	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	if(viewMode)
		gluPerspective( /* field of view in degree */ View_Distance,
		/* aspect ratio */ 1.0,
		/* Z near */ 1, /* Z far */ 10000);
	else
		glOrtho(-80,80,-60,60,0,10000);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	//gluLookAt(0 ,0 ,0,  /* eye is at () */
	//	0.0,0.0 , View_Distance,      /* center is at (0,0,0) */
	//	0.0, -1.0,0.0); 
	gluLookAt(0 ,0 ,0,  /* eye is at () */
		0.0,0 , 1,      /* center is at (0,0,0) */
		0.0, -1.0,0.0); 


	float factor[3] = {1, 1, 1};
	float xShifter[3] = {-30,30,40};
	// check if we need to draw depth frame to texture
	for(int i=0;i<deviceNum;i++)
	{
		int sumZ = 0,sumX = 0,sumY = 0;
		int num = 0;
		if (depthFrame[i].isValid() && g_drawDepth && colorFrame[i].isValid())
		{
			//printf("Drawing : %d\n",i);
			const nite::UserMap& userLabels = userTrackerFrame[i].getUserMap();
			const nite::UserId* pLabels = userLabels.getPixels();

			const openni::DepthPixel* pDepthRow = (const openni::DepthPixel*)depthFrame[i].getData();
			const openni::RGB888Pixel* pImageRow = (const openni::RGB888Pixel*)colorFrame[i].getData();
			//openni::RGB888Pixel* pTexRow = m_pTexMap + depthFrame.getCropOriginY() * m_nTexMapX;
			int rowSize = depthFrame[i].getStrideInBytes() / sizeof(openni::DepthPixel);

			for (int y = 0; y < depthFrame[i].getHeight(); ++y)
			{
				const openni::DepthPixel* pDepth = pDepthRow;
				const openni::RGB888Pixel* pImage = pImageRow;
				//openni::RGB888Pixel* pTex = pTexRow + depthFrame.getCropOriginX();

				for (int x = 0; x < depthFrame[i].getWidth(); ++x, ++pDepth, ++pLabels,++pImage)
				{
					if (*pDepth != 0)
					{
						if (*pLabels != 0)
						{
							//glColor3f(pImage->r/225.0,pImage->g/225.0,pImage->b/225.0);
							//int nHistValue = m_pDepthHist[*pDepth];
						
							//glVertex3f(x/10.0-24+xShifter[i]-10,y/10.0-32,pDepth[0]/10);
							sumZ+=pDepth[0]/zFactor;
							sumX+=x;
							sumY+=y;
							num++;
						}
					}
				}
				pImageRow += rowSize;
				pDepthRow += rowSize;
				//pTexRow += m_nTexMapX;
			}
		}
		if(num!=0)
		{
			//meanZ[i] = ((float)sumZ)/num;
			meanX[i] = ((float)sumX)/num;
			meanY[i] = ((float)sumY)/num;
			const openni::DepthPixel* pDepthRow = (const openni::DepthPixel*)depthFrame[i].getData();
			meanZ[i] = pDepthRow[depthFrame[i].getWidth()*((int)meanY[i])+(int)meanX[i]]/10;
		}
		else
		{
			meanZ[i]=0;
		}
		
		//printf("meanZ : %f\n",meanZ);

		if (depthFrame[i].isValid() && g_drawDepth && colorFrame[i].isValid())
		{
			//printf("Drawing : %d\n",i);
			const nite::UserMap& userLabels = userTrackerFrame[i].getUserMap();
			const nite::UserId* pLabels = userLabels.getPixels();

			const openni::DepthPixel* pDepthRow = (const openni::DepthPixel*)depthFrame[i].getData();
			const openni::RGB888Pixel* pImageRow = (const openni::RGB888Pixel*)colorFrame[i].getData();
			//openni::RGB888Pixel* pTexRow = m_pTexMap + depthFrame.getCropOriginY() * m_nTexMapX;
			int rowSize = depthFrame[i].getStrideInBytes() / sizeof(openni::DepthPixel);

			glPushMatrix();
			glTranslatef(translateX[i],translateY[i],0);
			glTranslatef(xShifter[i],0,meanZ[i]);
			if(soraMode)
				glRotatef(90,1,0,0);
			glRotatef(theta[i],0,1,0);
			glTranslatef(-(xShifter[i]),0,-meanZ[i]);
			const openni::DepthPixel* pDepth = pDepthRow;
			const openni::RGB888Pixel* pImage = pImageRow;
			glBegin(GL_POINTS);  
			for (int y = 0; y < depthFrame[i].getHeight(); ++y)
			{

				//openni::RGB888Pixel* pTex = pTexRow + depthFrame.getCropOriginX();

				for (int x = 0; x < depthFrame[i].getWidth(); ++x, ++pDepth, ++pLabels,++pImage)
				{
					if (*pDepth != 0)
					{
						//if (*pLabels != 0)//||g_drawBackground)
						{
							glColor3f(pImage->r/225.0,pImage->g/225.0,pImage->b/225.0);
							//int nHistValue = m_pDepthHist[*pDepth];
							//float realx,realy;
							//m_pUserTracker[i]->convertDepthCoordinatesToJoint(x,depthFrame[i].getHeight()-y,pDepth[0],&realx,&realy);
							//glVertex3f(realx/60+xShifter[i],realy/60,pDepth[0]/60);
							glVertex3f(x/10.0-24+xShifter[i]-10,y/10.0-32,pDepth[0]/zFactor);
						}
					}
				}
				//pImageRow += rowSize;
				//pDepthRow += rowSize;
				//pTexRow += m_nTexMapX;
			}
			glEnd();
			glPopMatrix();
		}
	}


	
	// Swap the OpenGL display buffers
	glutSwapBuffers();

}
void SampleViewer::MouseMotion(int x,int y)
{
	if(buttonState)
	{
		//printf("x : %d , y : %d\n",x,y);
		if(buttonPos == 0)
			theta[0] = thetaKeep[0] + x - clickX ;
		else
		{
			theta[0] = thetaKeep[0] + x - clickX ;
			theta[1] = thetaKeep[1] + x - clickX ;
		}

	}
}
void SampleViewer::SelectingMouseEvent(int button,int state,int xx,int yy)
{
	float factor[3] = {1, 1, 1};
	float xShifter[3] = {-30,30,40};
	
	if(state==GLUT_DOWN && button==GLUT_LEFT_BUTTON)
	{
		printf("x = %d, y = %d\n",xx,yy);
		const openni::RGB888Pixel* pImage = (const openni::RGB888Pixel*)snapColorFrame[nowDevice].getData();
		for (int y = 0; y < snapDepthFrame[nowDevice].getHeight(); ++y)
		{

			//openni::RGB888Pixel* pTex = pTexRow + depthFrame.getCropOriginX();

			for (int x = 0; x < snapDepthFrame[nowDevice].getWidth(); ++x)
			{
				if(abs(xx-(x/10.0-24+xShifter[nowDevice]-10))<=2 && abs(yy-y/10.0-32)<=2)
				{
					printf("r = %d,g = %d, b  = %d\n",(pImage+y*640+x)->r,(pImage+y*640+x)->g,(pImage+y*640+x)->b);
				}


			}
		}
	}
}
void SampleViewer::MouseEvent(int button,int state,int x,int y)
{

	//printf("x : %d , y : %d\n",x,y);
	if(x<GL_WIN_SIZE_X/2)
	{
		if(state==GLUT_DOWN && button==GLUT_LEFT_BUTTON)
		{
			buttonState = true;
			clickX = x;
		}
		if(state==GLUT_UP && button==GLUT_LEFT_BUTTON)
		{
			buttonState = false;
			thetaKeep[0] = theta[0];
		}
		buttonPos = 0;
		printf("left\n");
	}
	else
	{
		if(state==GLUT_DOWN && button==GLUT_LEFT_BUTTON)
		{
			buttonState = true;
			clickX = x;
		}
		if(state==GLUT_UP && button==GLUT_LEFT_BUTTON)
		{
			buttonState = false;
			thetaKeep[0] = theta[0];
			thetaKeep[1] = theta[1];
		}
		buttonPos = 1;
		printf("right\n");
	}

}
void SampleViewer::scaleCalibration()//only for calibration obj
{
	pointf center;
	double wariai;
	int centerX=32,centerY=24;
	center.setToZero();
	for(int j=0;j<4;j++)
		center.z+=calibrationRef[BASE][j].z;
	center.z/=4;
	for(int i=0;i<deviceNum;i++)
	{
		for(int j=0;j<4;j++)
		{
			wariai = calibrationRef[i][j].z / center.z;
			calibrationRef[i][j].x = (wariai * (calibrationRef[i][j].x-centerX))+centerX;
			calibrationRef[i][j].y = (wariai * (calibrationRef[i][j].y-centerY))+centerY;
		}
	}
	printf("after view calibration:\n");
	for(int i=0;i<deviceNum;i++)
		for(int j=0;j<4;j++)
			printf("device %d color %d : x=%f , y=%f , z=%f\n",i,j,calibrationRef[i][j].x,calibrationRef[i][j].y,calibrationRef[i][j].z);
}
void SampleViewer::getCrossVector()
{
	for(int i=0;i<deviceNum;i++)
	{
		vectorRY[i].x=calibrationRef[i][0].x-calibrationRef[i][1].x;
		vectorRY[i].y=calibrationRef[i][0].y-calibrationRef[i][1].y;
		vectorRY[i].z=calibrationRef[i][0].z-calibrationRef[i][1].z;
		vectorBG[i].x=calibrationRef[i][2].x-calibrationRef[i][3].x;
		vectorBG[i].y=calibrationRef[i][2].y-calibrationRef[i][3].y;
		vectorBG[i].z=calibrationRef[i][2].z-calibrationRef[i][3].z;
	}
}
void SampleViewer::getCalibrationCenter()
{
	pointf center;
	for(int i=0;i<deviceNum;i++)
	{
		center.setToZero();
		for(int j=0;j<4;j++)
		{
			center.x+=calibrationRef[i][j].x;
			center.y+=calibrationRef[i][j].y;
			center.z+=calibrationRef[i][j].z;
		}
		calibrationCenter[i].x = center.x/4;
		calibrationCenter[i].y = center.y/4;
		calibrationCenter[i].z = center.z/4;

	}
}

void SampleViewer::getTranslateT(int base)
{
	for(int i=0;i<deviceNum;i++)
	{
		translateT[i].x = (calibrationCenter[base].x-calibrationCenter[i].x)*trueFactor;
		translateT[i].y = (calibrationCenter[base].y-calibrationCenter[i].y)*trueFactor;
		translateT[i].z = (calibrationCenter[base].z-calibrationCenter[i].z)*trueFactor;
	}

}

void SampleViewer::rotate(float degree,pointf &test,pointf c)
{
	double r = ((double)degree)*2*3.14159/360;
	double x = test.x - c.x,y=test.y - c.y, z=test.z - c.z;
	test.x = x*cos(r)+z*sin(r) + c.x;
	test.y = y + c.y;
	test.z = -1*x*sin(r)+z*cos(r) + c.z;
}

void SampleViewer::getRotateR(int base)
{
	pointf nowCali[4],rollingTest[4];
	for(int i=0;i<deviceNum;i++)
	{
		rotateR[i].setToZero();
		if(i==base)
			continue;
		for(int j=0;j<4;j++)
		{
			nowCali[j].copyFrom(calibrationRef[i][j]);
			nowCali[j].x+=translateT[i].x;
			nowCali[j].y+=translateT[i].y;
			nowCali[j].z+=translateT[i].z;
		}
		//y rolling test start
		double minDistance = 999999999,minAngle = 2*3.14159,nowTheta=-1;
		vector testRY,testBG,minTestRY,minVectorRY;
		for(int theta = 1; theta <= 360; theta++ )
		{
			double r = ((double)theta)*2*3.14159/360;
			double totalDistance = 0;
			for(int j=0;j<4;j++)
			{
				double x = nowCali[j].x - calibrationCenter[base].x,y=nowCali[j].y - calibrationCenter[base].y,z=nowCali[j].z - calibrationCenter[base].z;
				rollingTest[j].x = x*cos(r)+z*sin(r) + calibrationCenter[base].x;
				rollingTest[j].y = y + calibrationCenter[base].y;
				rollingTest[j].z = -1*x*sin(r)+z*cos(r) + calibrationCenter[base].z;
				//totalDistance += rollingTest[j].getDistance(calibrationRef[base][j]);
			}
			testRY.x = rollingTest[0].x-rollingTest[1].x;
			testRY.y = rollingTest[0].y-rollingTest[1].y;
			testRY.z = rollingTest[0].z-rollingTest[1].z;
			testBG.x = rollingTest[2].x-rollingTest[3].x;
			testBG.y = rollingTest[2].y-rollingTest[3].y;
			testBG.z = rollingTest[2].z-rollingTest[3].z;
			double angle = testRY.getAngle(vectorRY[BASE]);
			if(minAngle>angle)
			{
				nowTheta = theta;
				minAngle = angle;
				minTestRY.copyFrom(testRY);
				//minVectorRY.copyFrom(vectorRY[BASE]);
			}
		}
		if(nowTheta == -1)
		{
			printf("something ERROR in angle calculation for theta didnt fund minDis = %f\n",minDistance);
			system("pause");
		}
		else
		{
			rotateR[i].y = nowTheta;
			printf("minAngle = %lf\n",minAngle);
			printf("mintestRY : ");
			minTestRY.print();
			printf("BASEVector : ");
			vectorRY[BASE].print();
			printf("vectorRY[%d] : "  ,i);
			vectorRY[i].print();
			//double angle = vectorRY[BASE].getAngle(vectorRY[i]);
			//printf("realAngle = %lf\n",angle);
		}

	}
}
openni::Status SampleViewer::InitOpenGL(int argc, char **argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowPosition(650,230);
	glutInitWindowSize(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
	glutCreateWindow (m_strSampleName);
	// 	glutFullScreen();
	//glutSetCursor(GLUT_CURSOR_NONE);

	InitOpenGLHooks();

	glDisable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);

	glEnableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);

	return openni::STATUS_OK;

}
void SampleViewer::InitOpenGLHooks()
{
	glutKeyboardFunc(glutKeyboard);
	glutDisplayFunc(glutDisplay);
	glutIdleFunc(glutIdle);
	glutMouseFunc(glutMouse);
	glutMotionFunc(glutMouseMotion);
}

void SampleViewer::OnKey(unsigned char key, int /*x*/, int /*y*/)
{
	std::fstream file2;
	std::fstream file;
	std::fstream file3;
	switch (key)
	{
	case 27:
		Finalize();
		exit (1);
	case 'a':
		if(!selectingMode&&humanDisplayMode)
			translateX[0]+=-1;
		else
		{
			nowX--;
			if(nowX < 0)
				nowX = 0; 
			newChange = true;
		}
		break;
	case 'd':
		if(!selectingMode&&humanDisplayMode)
			translateX[0]+=1;
		else
		{
			nowX++;
			if(nowX >= DEPTH_WIDTH)
				nowX = DEPTH_WIDTH-1; 
			newChange = true;
		}
		break;
	case 's':
		if(!selectingMode&&humanDisplayMode)
			translateY[0]+=1;
		else
		{
			nowY++;
			if(nowY >= DEPTH_HEIGHT)
				nowY = DEPTH_HEIGHT-1; 
			newChange = true;
		}

		break;
	case 'S':
		file3.open("translate.txt", std::fstream::out | std::fstream::trunc);
		file3<<translateX[0]<<" "<<translateY[0]<<std::endl;
		file3.close();
		break;
	case 'w':
		if(!selectingMode&&humanDisplayMode)
			translateY[0]+=-1;
		else
		{
			nowY--;
			if(nowY <0)
				nowY = 0; 
			newChange = true;
		}
		break;
	case 'k':
		if(soraMode)
			soraMode=false;
		else
			soraMode=true;
		break;
	case 'c':
		if(viewMode)
			viewMode=false;
		else
			viewMode=true;
		break;
	case 'x':
		// Draw bounding box?
		g_drawBoundingBox = !g_drawBoundingBox;
		break;
	case 'b':
		// Draw background?
		g_drawBackground = !g_drawBackground;
		break;
	case 'f':
		// Draw frame ID
		g_drawFrameId = !g_drawFrameId;
		break;
	case 'q':
		viewingID++;
		if(viewingID>=deviceNum)
			viewingID=0;
		break;

	case 'y':
		if(selectingMode)
		{
			calibrationRef[nowDevice][nowColor].x = nowX;
			calibrationRef[nowDevice][nowColor].y = nowY;
			calibrationRef[nowDevice][nowColor].z = nowZ;
			nowColor++;
			if(nowColor>=4)
			{
				nowDevice++;
				nowColor=0;
				selectingMode = false;
				if(nowDevice>=deviceNum)//selectingMode end
				{                       //go to human display mode
					selectingMode = false;
					getTheRealSpot();
					//humanDisplayMode = true;
					//scaleCalibration();//for calibration obj 
					getCrossVector();
					getCalibrationCenter();//calculate the calibration obj center
					getTranslateT(0);//calculate the translate
					getRotateR(0);//onaji above
					std::fstream file;  
					file.open("data.txt", std::fstream::out | std::fstream::trunc);
					for(int i=0;i<deviceNum;i++)
					{
						printf("translate %d : ",i);
						translateT[i].print();
						printf("rotate %d : ",i);
						rotateR[i].print();
						
						file<<translateT[i].x<<" "<<translateT[i].y<<" "<<translateT[i].z<<std::endl;
						file<<rotateR[i].x<<" "<<rotateR[i].y<<" "<<rotateR[i].z<<std::endl;
						
					}
					file.close();
				}
			}
			nowX=0;
			nowY=0;
			if(selectingMode)
				findNextSpot();
		}
		break;
	case 'n':
		if(selectingMode)
		{
			findNextSpot();
		}
		break;
	case 'p':
		takePicture = true;
		break;
	case 'o':
		if(denseMode)
			denseMode = false;
		else
			denseMode = true;
		break;
	case 'v':
		if(scaleMode)
			scaleMode = false;
		else
			scaleMode = true;
		break;
	case 'h':
		if(humanDisplayMode)
			humanDisplayMode = false;
		else
			humanDisplayMode = true;
		break;
	case 'u':
		if(humanDisplayMode)
			trackingID[userDeviceSwitcher]++;
		break;
	case 'i':
		if(humanDisplayMode)
		{
			if(++userDeviceSwitcher>=deviceNum)
				userDeviceSwitcher = 0;
			printf("now userDeviceSwitcher = %d\n",userDeviceSwitcher);
		}
		break;
	case 'r':
		if(rotationMode)
			rotationMode = false;
		else
			rotationMode = true;
		break;
	case 'l':
		file.open("data.txt",std::fstream::in);
		if(!file.is_open())
		{
			printf("ERROR in load data.txt\n");
			break;
		}
		for(int i=0;i<deviceNum;i++)
		{
			file>>translateT[i].x>>translateT[i].y>>translateT[i].z;
			file>>rotateR[i].x>>rotateR[i].y>>rotateR[i].z;
			printf("translate %d : ",i);
			translateT[i].print();
			printf("rotate %d : ",i);
			rotateR[i].print();		
		}
		break;	
	case 'L':
		file2.open("translate.txt",std::fstream::in);
		if(!file2.is_open())
		{
			printf("ERROR in load data.txt\n");
			break;
		}

		file2>>translateX[0]>>translateY[0];
		printf("translate x : %f,translate y : %f\n",translateX[0],translateY[0]);
		file2.close();
		break;	
	case 't':
		if(traditionMode)
			traditionMode = false;
		else
			traditionMode = true;
		break;
	}

}

vector cross(pointf a,pointf b)
{
	vector axb;
	axb.x=a.y*b.z-a.z*b.y;
	axb.y=a.z*b.x-a.x*b.z;
	axb.z=a.x*b.y-a.y*b.x;
	return axb;
}
void calculateNormalMap(pointf *pointCloud)
{
	for (int y = 0; y < DEPTH_HEIGHT-1; ++y)
		for (int x = 0; x < DEPTH_WIDTH-1; ++x)
		{
			int index = y*DEPTH_WIDTH+x,right = y*DEPTH_WIDTH+x+1 , bot = (y+1)*DEPTH_WIDTH+x;
			if(pointCloud[index].type==0)
			{
				pointf a,b;
				a.x = pointCloud[right].x - pointCloud[index].x;
				a.y = pointCloud[right].y - pointCloud[index].y;
				a.z = pointCloud[right].z - pointCloud[index].z;
				b.x = pointCloud[bot].x - pointCloud[index].x;
				b.y = pointCloud[bot].y - pointCloud[index].y;
				b.z = pointCloud[bot].z - pointCloud[index].z;
				pointCloud[index].normal.copyFrom(cross(b,a));
				pointCloud[index].normal.normalize();
			}
		}
}

int reference(int x,int y)
{
	int X=(x+500)/4,Y=(y+500)/3;
	if(X<0||X>=VOLUME_X||Y<0||Y>=VOLUME_Y)
	{
		printf("x=%d,y=%d\n",X,Y);
		system("pause");
	}
	return Y*VOLUME_X+X;
}

void SampleViewer::humanDisplay()
{
	frameCounter++;
	nite::UserTrackerFrameRef userTrackerFrame[MAX_DEVICE];
	openni::VideoFrameRef depthFrame[MAX_DEVICE],colorFrame[MAX_DEVICE];
	for(int i=0;i<deviceNum;i++)
	{
		
		nite::Status rc = m_pUserTracker[i]->readFrame(&userTrackerFrame[i]);
		if (rc != nite::STATUS_OK)
		{
			printf("GetNextData failed\n");
			return;
		}
		m_colorStream[i].readFrame(&colorFrame[i]);
		depthFrame[i] = userTrackerFrame[i].getDepthFrame();
	}
	for(int i=0;i<VOLUME_X*VOLUME_Y;i++)
		volume[i].reset();
	


	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	if(viewMode)
		gluPerspective( /* field of view in degree */ View_Distance,
		/* aspect ratio */ 1.0,
		/* Z near */ 1, /* Z far */ 10000);
	else
		glOrtho(-80,80,-60,60,0,10000);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	//gluLookAt(0 ,0 ,0,  /* eye is at () */
	//	0.0,0.0 , View_Distance,      /* center is at (0,0,0) */
	//	0.0, -1.0,0.0); 
	gluLookAt(0 ,0 ,0,  /* eye is at () */
		0.0,0 , 1,      /* center is at (0,0,0) */
		0.0, 1.0,0.0); 


	float factor[3] = {1, 1, 1};
	float xShifter[3] = {0,0,0};
	// check if we need to draw depth frame to texture
	double avgZ=0;
	int outlier[MAX_DEVICE] = {0},countIt[MAX_DEVICE] = {0} , outlierBase[MAX_DEVICE] = {0} , countItBase[MAX_DEVICE] = {0};
	for(int i=0;i<deviceNum;i++)
	{
		int sumZ = 0,sumX = 0,sumY = 0;
		int num = 0;
		
		if (depthFrame[i].isValid() && g_drawDepth && colorFrame[i].isValid())
		{
			const nite::UserMap& userLabels = userTrackerFrame[i].getUserMap();
			const nite::UserId* pLabels = userLabels.getPixels();
			const nite::UserId* isHuman = userLabels.getPixels();
			const nite::Array<nite::UserData>& users = userTrackerFrame[i].getUsers();

			const openni::DepthPixel* pDepthRow = (const openni::DepthPixel*)depthFrame[i].getData();
			const openni::RGB888Pixel* pImageRow = (const openni::RGB888Pixel*)colorFrame[i].getData();
			int rowSize = depthFrame[i].getStrideInBytes() / sizeof(openni::DepthPixel);

			if(trackingID[i] >= users.getSize())
				trackingID[i] = 0;

			/*if(trackingID[i]<users.getSize())
			{
				if(users[trackingID[i]].isNew())
				{
					m_pUserTracker[i]->startSkeletonTracking( users[trackingID[i]].getId() );
				}
				pointf head;
				if(users[trackingID[i]].getSkeleton().getJoint(nite::JOINT_TORSO).getPositionConfidence()<0.5)
					skeletonCaptured = false;
				else
					skeletonCaptured = true;
				head.x  = users[trackingID[i]].getSkeleton().getJoint(nite::JOINT_TORSO).getPosition().x;
				head.y  = users[trackingID[i]].getSkeleton().getJoint(nite::JOINT_TORSO).getPosition().y;
				head.z  = users[trackingID[i]].getSkeleton().getJoint(nite::JOINT_TORSO).getPosition().z;
				float realx,realy;
				m_pUserTracker[trackingID[i]]->convertJointCoordinatesToDepth(head.x,head.y,head.z,&realx,&realy);

				pointf avg;
				avg.setToZero();
				int count = 0;
				for(int y=realy-10;y<realy+10;y++)
					for(int x=realx-10;x<realx+10;x++)
						if(pDepthRow[y*DEPTH_WIDTH+x]!=0&&isHuman[y*DEPTH_WIDTH+x]!=0)
						{
							count++;
							avg.x+=x;
							avg.y+=y;
							avg.z+=pDepthRow[y*DEPTH_WIDTH+x];
						}
				if(count!=0)
				{
					avg.x/=count;
					avg.y/=count;
					avg.z/=count;
				}
				torso[i].copyFrom(avg);
				int x = realx , y = realy;
				torso[i].x=realx;
				torso[i].y=realy;
				torso[i].z=pDepthRow[y*DEPTH_WIDTH+x];
			}
			else
				skeletonCaptured = false;*/
			/*skeletonCaptured = false;
			for(int j=0;j<users.getSize();j++)
			{
				if(users[j].isNew())
				{
					m_pUserTracker[i]->startSkeletonTracking( users[trackingID[i]].getId() );
				}
				if(j==trackingID[i])
				{
					pointf head;
					if(users[trackingID[i]].getSkeleton().getJoint(nite::JOINT_TORSO).getPositionConfidence()<0.5)
						skeletonCaptured = false;
					else
						skeletonCaptured = true;
					head.x  = users[trackingID[i]].getSkeleton().getJoint(nite::JOINT_TORSO).getPosition().x/trueFactor;
					head.y  = users[trackingID[i]].getSkeleton().getJoint(nite::JOINT_TORSO).getPosition().y/trueFactor;
					head.z  = users[trackingID[i]].getSkeleton().getJoint(nite::JOINT_TORSO).getPosition().z/trueFactor;
					torso[i].copyFrom(head);
					//float realx,realy;
					//m_pUserTracker[trackingID[i]]->convertJointCoordinatesToDepth(head.x,head.y,head.z,&realx,&realy);

					/*pointf avg;
					avg.setToZero();
					int count = 0;
					for(int y=realy-10;y<realy+10;y++)
						for(int x=realx-10;x<realx+10;x++)
							if(pDepthRow[y*DEPTH_WIDTH+x]!=0&&isHuman[y*DEPTH_WIDTH+x]!=0)
							{
								count++;
								avg.x+=x;
								avg.y+=y;
								avg.z+=pDepthRow[y*DEPTH_WIDTH+x];
							}
					if(count!=0)
					{
						avg.x/=count;
						avg.y/=count;
						avg.z/=count;
					}
					torso[i].copyFrom(avg);*/
					
					/*int xx = realx , yy = realy;
					torso[i].x=realx;
					torso[i].y=realy;
					torso[i].z=head.z;*/
				//}
				/*if(trackingID[i]!=j)
				{
					m_pUserTracker[i]->stopSkeletonTracking(users[j].getId());
				}*/
			//}
			float realX,realY;
			for (int y = 0; y < depthFrame[i].getHeight(); ++y)
			{
				const openni::DepthPixel* pDepth = pDepthRow;
				const openni::RGB888Pixel* pImage = pImageRow;

				for (int x = 0; x < DEPTH_WIDTH; ++x, ++pDepth, ++pLabels,++pImage)
				{
					int index = y*DEPTH_WIDTH+x;
					if(i==BASE)
						basePointCloud[index].type = -1;
					pointCloud[i][index].type = -1;
					if (*pDepth != 0)
					{
						
						if (*pLabels != 0)
						{
							

							m_pUserTracker[i]->convertDepthCoordinatesToJoint(x,y,pDepth[0],&realX,&realY);
							if(i==BASE)
							{
								basePointCloud[index].x = realX;
								basePointCloud[index].y = realY;
								basePointCloud[index].z = pDepth[0];
								basePointCloud[index].type = 0;//human point
								basePointCloud[index].color.set(pImage->r/255.0,pImage->g/255.0,pImage->b/255.0);
								int ref = reference((int)realX,(int)realY);
								volume[ref].addPoint(basePointCloud[index]);
								
							}
							
							{
								pointCloud[i][index].x = realX+translateT[i].x;
								pointCloud[i][index].y = realY+translateT[i].y;
								pointCloud[i][index].z = pDepth[0]+translateT[i].z;
								pointCloud[i][index].color.set(pImage->r/255.0,pImage->g/255.0,pImage->b/255.0);
								pointCloud[i][index].type = 0;//human point
							}
							sumX+=realX;
							sumY+=realY;
							sumZ+=pDepth[0];
							num++;

						}
						else
						{
							if(i==BASE)
							{
								basePointCloud[index].x = 0;
								basePointCloud[index].y = 0;
								basePointCloud[index].z = 0;
								basePointCloud[index].type = -1;//non human point
							}
							
							{
								pointCloud[i][index].x = 0;
								pointCloud[i][index].y = 0;
								pointCloud[i][index].z = 0;
								pointCloud[i][index].type = -1;//non human point
							}
						}
					}
				}
				pImageRow += rowSize;
				pDepthRow += rowSize;
				//pTexRow += m_nTexMapX;
			}
			
		}
		
		if(num!=0)
		{
			//meanZ[i] = ((float)sumZ)/num;
			meanX[i] = ((float)sumX)/num;
			meanY[i] = ((float)sumY)/num;
			const openni::DepthPixel* pDepthRow = (const openni::DepthPixel*)depthFrame[i].getData();
			meanZ[i] = pDepthRow[depthFrame[i].getWidth()*((int)meanY[i])+(int)meanX[i]]/zFactor;
			avgZ=((float)sumZ)/num;
			//float realx,realy;
			//m_pUserTracker[i]->convertDepthCoordinatesToJoint((int)meanX[i],depthFrame[i].getHeight()-((int)meanY[i]),meanZ[i],&realx,&realy);
			humanCenter[i].x = meanX[i];
			humanCenter[i].y = meanY[i];
			humanCenter[i].z = avgZ;
		}
		else
		{
			meanZ[i]=0;
			avgZ=0;
		}
		if(i!=BASE)
		{
			//int index;
			for (int y = 0; y < depthFrame[i].getHeight(); ++y)
			{
				for (int x = 0; x < DEPTH_WIDTH; ++x)
				{
					int index = y*DEPTH_WIDTH+x;
					if(basePointCloud[index].type==0)
					{
						//printf("say yes\n");
						pointCloud[BASE][index].copyFrom(basePointCloud[index]);
						rotate(-1*rotateR[i].y,pointCloud[BASE][index],humanCenter[BASE]);
					}
					if(pointCloud[i][index].type==0)
					{
						rotate(rotateR[i].y,pointCloud[i][index],humanCenter[i]);
					}
					
				}
			}

			calculateNormalMap(pointCloud[i]);
			calculateNormalMap(pointCloud[BASE]);
			pointf begin,end;
			begin.setToZero();
			end.setToZero();
			for (int y = 0; y < depthFrame[i].getHeight(); ++y)
			{
				for (int x = 0; x < DEPTH_WIDTH; ++x)
				{
					int index = y*DEPTH_WIDTH+x;
					
					if(pointCloud[i][index].type==0)
					{
						if(pointCloud[i][index].normal.z>0)
						{
							float weight = pointCloud[i][index].normal.z;
							begin.x+=weight*pointCloud[i][index].x;
							begin.y+=weight*pointCloud[i][index].y;
							begin.z+=weight*pointCloud[i][index].z;
							begin.count+=weight;
							countItBase[i]++;
						}
						else
						{
							outlierBase[i]++;
						}
					}
					if(pointCloud[BASE][index].type==0)
					{
						if(pointCloud[BASE][index].normal.z>0)
						{
							float weight = pointCloud[BASE][index].normal.z;
							end.x+=weight*basePointCloud[index].x;
							end.y+=weight*basePointCloud[index].y;
							end.z+=weight*basePointCloud[index].z;
							end.count+=weight;
							countIt[i]++;
						}
						else
						{
							outlier[i]++;
						}
					}
				}
			}
			begin.DoAvg();
			end.DoAvg();
			realTranslate[i].doVector(begin,end);
			for (int y = 0; y < depthFrame[i].getHeight(); ++y)
			{
				for (int x = 0; x < DEPTH_WIDTH; ++x)
				{
					int index = y*DEPTH_WIDTH+x;
					if(pointCloud[i][index].type==0)
					{
						pointCloud[i][index].x += realTranslate[i].x;
						pointCloud[i][index].y += realTranslate[i].y;
						pointCloud[i][index].z += realTranslate[i].z;
						int ref = reference(pointCloud[i][index].x,pointCloud[i][index].y);
						volume[ref].addPoint(pointCloud[i][index]);
					}
				}
			}
		}

	}
	if(traditionMode)
	{
		pointf translateTH[MAX_DEVICE];
		for(int i=0;i<deviceNum;i++)
		{
			translateTH[i].setToZero();
			if(i!=BASE)
			{
				translateTH[i].x = meanX[BASE] - meanX[i];
				translateTH[i].y = meanY[BASE] - meanY[i];
				translateTH[i].z = meanZ[BASE] - meanZ[i];
			}
		}
		for(int i=0;i<deviceNum;i++)
		{
			if (depthFrame[i].isValid() && g_drawDepth && colorFrame[i].isValid())
			{
				//printf("Drawing : %d\n",i);
				const nite::UserMap& userLabels = userTrackerFrame[i].getUserMap();
				const nite::UserId* pLabels = userLabels.getPixels();
			
				const nite::Array<nite::UserData>& users = userTrackerFrame[i].getUsers();



				const openni::DepthPixel* pDepthRow = (const openni::DepthPixel*)depthFrame[i].getData();
				const openni::RGB888Pixel* pImageRow = (const openni::RGB888Pixel*)colorFrame[i].getData();
				//openni::RGB888Pixel* pTexRow = m_pTexMap + depthFrame.getCropOriginY() * m_nTexMapX;
				int rowSize = depthFrame[i].getStrideInBytes() / sizeof(openni::DepthPixel);

				glPushMatrix();
				glTranslatef(translateX[i],translateY[i],0);
															/*
			glTranslatef(xShifter[i],0,meanZ[i]);
			if(soraMode)
				glRotatef(90,1,0,0);
			glRotatef(theta[i],0,1,0);
			glTranslatef(-(xShifter[i]),0,-meanZ[i]);*/
			//glTranslatef(translateT[i].x,translateT[i].y,translateT[i].z);
			/*pointf T;
			T.x = torso[BASE].x - torso[i].x;
			T.y = torso[BASE].y - torso[i].y;
			T.z = torso[BASE].z - torso[i].z;
			glTranslatef(T.x,T.y,T.z);*/
				if(i!=BASE);
												//glTranslatef(realTranslate[i].x/trueFactor,realTranslate[i].y/trueFactor,realTranslate[i].z/trueFactor);
			/*rotate(rotateR[i].y,torso[i],torso[BASE]);
			pointf T;
			T.x = torso[BASE].x - torso[i].x;
			T.y = torso[BASE].y - torso[i].y;
			T.z = torso[BASE].z - torso[i].z;*/
			//glTranslatef(T.x,T.y,T.z);
			//glTranslatef(translateTH[i].x/10,translateTH[i].y/10,translateTH[i].z);
				pointf t;
				skeletonCaptured = false;
				if(skeletonCaptured)
				{
					if(rotationMode)
					{
						theta[i] += 0.3;
						printf("%f\n",theta[i]);
					}
					t.x = torso[i].x;
					t.y = torso[i].y;
					t.z = torso[i].z;
					glTranslatef(t.x,t.y,t.z);
					//glTranslatef(calibrationCenter[i].x-34,calibrationCenter[i].y-32,calibrationCenter[i].z);
					glRotatef(rotateR[i].y+theta[i],0,1,0);
					//glTranslatef(-1*(calibrationCenter[i].x-34),-1*(calibrationCenter[i].y-32),-1*calibrationCenter[i].z);
					glTranslatef(-t.x,-t.y,-t.z);
				}
				else
				{
							//glTranslatef(humanCenter[i].x/trueFactor,humanCenter[i].y/trueFactor,humanCenter[i].z/trueFactor);
				//glRotatef(rotateR[i].y+theta[i],0,1,0);
				//glTranslatef(-humanCenter[i].x/trueFactor,-humanCenter[i].y/trueFactor,-humanCenter[i].z/trueFactor);
				}



				float basicDx = 0.2,basicDy = 0.2,basicDz = 1;
				float centerX=320,centerY=240;
				glBegin(GL_POINTS);  
				for (int y = 0; y < depthFrame[i].getHeight(); ++y)
				{
					const openni::DepthPixel* pDepth = pDepthRow;
					const openni::RGB888Pixel* pImage = pImageRow;
					//openni::RGB888Pixel* pTex = pTexRow + depthFrame.getCropOriginX();

					for (int x = 0; x < depthFrame[i].getWidth(); ++x, ++pDepth, ++pLabels,++pImage)
					{
						if (*pDepth != 0)
						{
							if (*pLabels != 0)
							{
								int index = y*DEPTH_WIDTH+x;
								glColor3f(pImage->r/225.0,pImage->g/225.0,pImage->b/225.0);
								//int nHistValue = m_pDepthHist[*pDepth];
								double wariai = (pDepth[0]/zFactor) / meanZ[BASE];
								if(scaleMode==false)
									wariai = 1;
								double realX,realY;
								realX = (wariai * (x-centerX))+centerX;
								realY = (wariai * (y-centerY))+centerY;
								float realx,realy;
								//m_pUserTracker[i]->convertDepthCoordinatesToJoint(x,y,pDepth[0],&realx,&realy);
								//glVertex3f(realx/trueFactor,realy/trueFactor,pDepth[0]/trueFactor);
								if(i==BASE)
								{
									glVertex3f(basePointCloud[index].x/trueFactor,basePointCloud[index].y/trueFactor,basePointCloud[index].z/trueFactor);
								}
								else
								{
									//if(pointCloud[i][index].normal.x>0)
										glVertex3f((pointCloud[i][index].x+realTranslate[i].x)/trueFactor,(pointCloud[i][index].y+realTranslate[i].y)/trueFactor,(pointCloud[i][index].z+realTranslate[i].z)/trueFactor);
								}
								//glVertex3f(realX/10.0-24+xShifter[i]-10,realY/10.0-32,pDepth[0]/zFactor);
								/*if(denseMode)
								{
									glVertex3f(realX/10.0-24+xShifter[i]-10+basicDx,realY/10.0-32,pDepth[0]/zFactor);
									glVertex3f(realX/10.0-24+xShifter[i]-10-basicDx,realY/10.0-32,pDepth[0]/zFactor);
									glVertex3f(realX/10.0-24+xShifter[i]-10,realY/10.0-32+basicDy,pDepth[0]/zFactor);
									glVertex3f(realX/10.0-24+xShifter[i]-10,realY/10.0-32-basicDy,pDepth[0]/zFactor);
									glVertex3f(realX/10.0-24+xShifter[i]-10,realY/10.0-32,pDepth[0]/zFactor+basicDz);
									glVertex3f(realX/10.0-24+xShifter[i]-10,realY/10.0-32,pDepth[0]/zFactor-basicDz);
								}*/
								/*else
								{
									glVertex3f(x/10.0-24+xShifter[i]-10,y/10.0-32,pDepth[0]/10);
									if(denseMode)
									{
										glVertex3f(x/10.0-24+xShifter[i]-10+basicDx,y/10.0-32,pDepth[0]/10);
										glVertex3f(x/10.0-24+xShifter[i]-10-basicDx,y/10.0-32,pDepth[0]/10);
										glVertex3f(x/10.0-24+xShifter[i]-10,y/10.0-32+basicDy,pDepth[0]/10);
										glVertex3f(x/10.0-24+xShifter[i]-10,y/10.0-32-basicDy,pDepth[0]/10);
										glVertex3f(x/10.0-24+xShifter[i]-10,y/10.0-32,pDepth[0]/10+basicDz);
										glVertex3f(x/10.0-24+xShifter[i]-10,y/10.0-32,pDepth[0]/10-basicDz);
									}
								}*/
							}
						}
					}
					pImageRow += rowSize;
					pDepthRow += rowSize;
					//pTexRow += m_nTexMapX;
				}
				glEnd();
				//printf("avg depth=%f\n",sum/num);
				glPopMatrix();
				/*
				// begin draw head point
				if(i==0)
					glColor3f(1,0,0);
				else if(i==1)
					glColor3f(0,1,0);
				glPointSize(8);
				glBegin(GL_POINTS); 
					glVertex3f(torso[i].x,torso[i].y,torso[i].z);
				glEnd();
				glPointSize(1);*/
				if(frameCounter%SHOW_DATA_PER_FRAME==0)
				{
					printf("kinect %d : ",i);
					printf("realtranslate : ");
					realTranslate[i].print();
					printf("outlier : %d\n",outlier[i]);
					printf("count it :%d\n",countIt[i]);
					printf("outlier base : %d\n",outlierBase[i]);
					printf("count it base : %d\n",countItBase[i]);
					frameCounter = 0;
				}


																																						//draw boxing
			/*pointf realCenter,halfSide;
			halfSide.x = 13;
			halfSide.y = 20;
			halfSide.z = 15;
			realCenter.x = calibrationCenter[i].x-34;
			realCenter.y = calibrationCenter[i].y-32;
			realCenter.z = calibrationCenter[i].z;
			if(i==0)
				glColor3f(0,0,1);
			else
				glColor3f(0,1,0);
			glBegin(GL_LINES);
				glVertex3f(realCenter.x+halfSide.x,realCenter.y+halfSide.y,realCenter.z-halfSide.z);
				glVertex3f(realCenter.x+halfSide.x,realCenter.y+halfSide.y,realCenter.z+halfSide.z);
				
				glVertex3f(realCenter.x+halfSide.x,realCenter.y+halfSide.y,realCenter.z+halfSide.z);
				glVertex3f(realCenter.x+halfSide.x,realCenter.y-halfSide.y,realCenter.z+halfSide.z);
				glVertex3f(realCenter.x+halfSide.x,realCenter.y+halfSide.y,realCenter.z+halfSide.z);
				glVertex3f(realCenter.x-halfSide.x,realCenter.y+halfSide.y,realCenter.z+halfSide.z);
				glVertex3f(realCenter.x-halfSide.x,realCenter.y-halfSide.y,realCenter.z+halfSide.z);
				glVertex3f(realCenter.x-halfSide.x,realCenter.y-halfSide.y,realCenter.z-halfSide.z);
				glVertex3f(realCenter.x-halfSide.x,realCenter.y-halfSide.y,realCenter.z+halfSide.z);
				glVertex3f(realCenter.x-halfSide.x,realCenter.y+halfSide.y,realCenter.z+halfSide.z);
				glVertex3f(realCenter.x-halfSide.x,realCenter.y-halfSide.y,realCenter.z+halfSide.z);
				glVertex3f(realCenter.x+halfSide.x,realCenter.y-halfSide.y,realCenter.z+halfSide.z);
				glVertex3f(realCenter.x+halfSide.x,realCenter.y+halfSide.y,realCenter.z-halfSide.z);
				glVertex3f(realCenter.x+halfSide.x,realCenter.y-halfSide.y,realCenter.z-halfSide.z);
				glVertex3f(realCenter.x+halfSide.x,realCenter.y+halfSide.y,realCenter.z-halfSide.z);
				glVertex3f(realCenter.x-halfSide.x,realCenter.y+halfSide.y,realCenter.z-halfSide.z);
				glVertex3f(realCenter.x-halfSide.x,realCenter.y-halfSide.y,realCenter.z-halfSide.z);
				glVertex3f(realCenter.x+halfSide.x,realCenter.y-halfSide.y,realCenter.z-halfSide.z);
				glVertex3f(realCenter.x-halfSide.x,realCenter.y-halfSide.y,realCenter.z-halfSide.z);
				glVertex3f(realCenter.x-halfSide.x,realCenter.y+halfSide.y,realCenter.z-halfSide.z);
			glEnd();*/

			}
		}
	}
	else
	{
		glBegin(GL_POINTS);  
		glPushMatrix();
		for(int i = 0;i<VOLUME_X*VOLUME_Y;i++)
		{
			if(volume[i].NA == false)
				for(int j=0;j<volume[i].numOfPoints;j++)
				{
					glColor3f(volume[i].pointList[j].color.r,volume[i].pointList[j].color.g,volume[i].pointList[j].color.b);
					glVertex3f(volume[i].pointList[j].x/trueFactor,volume[i].pointList[j].y/trueFactor,volume[i].pointList[j].z/trueFactor);

				}
		}
		glPopMatrix();
		glEnd();
	}

	
	// Swap the OpenGL display buffers
	glutSwapBuffers();
	//system("pause");
}