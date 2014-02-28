#ifndef _VOXEL_H_
#define _VOXEL_H_
#include "pointf.h"
#include <vector>
#define DLIMIT 20
struct voxel
{
public:
	pointf *pointList;
	int numOfPoints;
	int listSize;
	bool NA;
	voxel()
	{
		NA = true;
		numOfPoints = 0;
		listSize = 0;
	}
	~voxel()
	{
		if(NA == false)
			delete [] pointList;
	}
	void reset()
	{
		if(NA == false)
			delete [] pointList;
		NA = true;
		numOfPoints = 0;
		listSize = 0;
	}
	void addPoint(pointf t)
	{
		if(NA)
		{
			listSize = 100;
			pointList = new pointf[listSize];
			NA = false;
		}
		if(numOfPoints >= listSize)
		{
			listSize*=2;
			pointf *buffer = new pointf[listSize];
			for(int i=0;i<listSize;i++)
				buffer[i].copyFrom(pointList[i]);
			delete [] pointList;
			pointList = buffer;
		}
		double minD=21474836;
		int minI=-1;
		for(int i=0;i<numOfPoints;i++)
		{
			double d = pointList[i].getDistance(t);
			if(minD>d)
			{
				minI = i;
				minD = d;
			}
		}
		if(minD<=DLIMIT)
		{
			if(minI==-1)
			{
				printf("afject\n");
				system("pause");
			}
			pointList[minI].x=(pointList[minI].x+t.x)/2;
			pointList[minI].y=(pointList[minI].y+t.y)/2;
			pointList[minI].z=(pointList[minI].z+t.z)/2;
		}
		else
		{
			pointList[numOfPoints++].copyFrom(t);
		}
	}
	void sortPointList()
	{
		pointf b;
		for(int i=0;i<numOfPoints-1;i++)
			for(int j=i+1;j<numOfPoints;j++)
				if(pointList[i].z>pointList[j].z)
				{
					b.copyFrom(pointList[i]);
					pointList[i].copyFrom(pointList[j]);
					pointList[j].copyFrom(b);
				}
	}
};

#endif