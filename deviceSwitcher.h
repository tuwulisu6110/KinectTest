#ifndef _DEVICESWITCHER_H_
#define _DEVICESWITCHER_H_

struct deviceSwitcher
{
public:
	deviceSwitcher(int n)
	{
		maxState = n;
		nowState = n;
	}
	void nextState()
	{
		if(nowState+1<=maxState)
			nowState++;
		else
			nowState = 0;
	}
	void prevState()
	{
		if(nowState-1>=0)
			nowState--;
		else
			nowState=maxState;
	}
	bool checkDeviceInUse(int i)
	{
		if(nowState == maxState || nowState == i)
			return true;
		else
			return false;
	}
private:
	int nowState;
	int maxState;

};

#endif