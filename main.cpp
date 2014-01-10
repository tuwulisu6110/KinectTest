/*******************************************************************************
*                                                                              *
*   PrimeSense NiTE 2.0 - User Viewer Sample                                   *
*   Copyright (C) 2012 PrimeSense Ltd.                                         *
*                                                                              *
*******************************************************************************/

#include "Viewer.h"

vector crosst(pointf a,pointf b)
{
	vector axb;
	axb.x=a.y*b.z-a.z*b.y;
	axb.y=a.z*b.x-a.x*b.z;
	axb.z=a.x*b.y-a.y*b.x;
	return axb;
}

int main(int argc, char** argv)
{
	openni::Status rc = openni::STATUS_OK;

	/*pointf a,b;
	a.x=1;a.y=0;a.z=0;
	b.x=0;b.y=1;b.z=0;
	vector ans = crosst(b,a);
	ans.print();
	system("pause");*/

	SampleViewer sampleViewer("User Viewer");

	rc = sampleViewer.Init(argc, argv);
	
	if (rc != openni::STATUS_OK)
	{
		return 1;
	}
	sampleViewer.Run();
}
