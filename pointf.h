#ifndef _POINTF_H_
#define _POINTF_H_
#include <math.h>
#include <stdio.h>
#include"vector.h"
struct pointf
{
public:
	vector normal;
	double x;
	double y;
	double z;
	int type;
	int count;
	pointf()
	{
		setToZero();
	}

	void setToZero()
	{
		x=0;
		y=0;
		z=0;
		count=0;
	}
	void copyTo(pointf &des)//??
	{
		des.x = x;
		des.y = y;
		des.z = z;
		des.count = count;
		des.type = type;
		des.normal.copyFrom(normal);
	}
	void copyFrom(pointf source)
	{
		x = source.x;
		y = source.y;
		z = source.z;
		count = source.count;
		type = source.type;
		normal.copyFrom(source.normal);
		
	}
	double getDistance(pointf p2)
	{
		return sqrt((x-p2.x)*(x-p2.x)+(y-p2.y)*(y-p2.y)+(z-p2.z)*(z-p2.z));
	}

	void print()
	{
		printf("x = %f , y = %f , z = %f\n",x,y,z);
	}

	void DoAvg()
	{
		if(count!=0)
		{
			x/=count;
			y/=count;
			z/=count;
		}
	}
};
#endif