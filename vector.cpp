#include "vector.h"
#include "pointf.h"
void vector::doVector(pointf &begin,pointf &end)
{
	x=end.x-begin.x;
	y=end.y-begin.y;
	z=end.z-begin.z;
}