#ifndef _COLOR_H_
#define _COLOR_H_
struct mcolor
{
public:
	float r;
	float g;
	float b;
	void set(float rr,float gg ,float bb)
	{
		r=rr;
		g=gg;
		b=bb;
	}
};
#endif