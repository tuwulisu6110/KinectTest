#ifndef _VECTOR_H_
#define _VECTOR_H_
#include <math.h>
#include <stdio.h>
struct pointf;
struct vector
{
public:
	double x;
	double y;
	double z;
	vector()
	{
		setToZero();
	}
	void setToZero()
	{
		x=0;
		y=0;
		z=0;
	}
	void copyFrom(vector source)
	{
		x = source.x;
		y = source.y;
		z = source.z;
	}
	double getAngle(vector p2)
	{
		double dot = x*p2.x+y*p2.y+z*p2.z;
		double length = sqrt(x*x+y*y+z*z)*sqrt(p2.x*p2.x+p2.y*p2.y+p2.z*p2.z);//fking the buggggggggggggggggggggg
		//if(acos(dot/length)<3.14159)
		return acos(dot/length);
		//else
			//return (2*3.14159)-acos(dot/length);
	}
	void normalize()
	{
		double len = sqrt( x*x+y*y+z*z );
		x /= len;
		y /= len;
		z /= len;
	}
	void doVector(pointf &begin,pointf &end);
	void print()
	{
		printf("x = %f , y = %f , z = %f\n",x,y,z);
	}
};
#endif