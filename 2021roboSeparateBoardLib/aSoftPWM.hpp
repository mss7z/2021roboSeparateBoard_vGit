#ifndef A_SOFT_PWM_INCLUDE_GUARD_HPP
#define A_SOFT_PWM_INCLUDE_GUARD_HPP

#include "mbed.h"

namespace rob{

namespace aSoftPWM_internal{
	
class SoftTimer{
	private:
	Timer t;
	int retTime,lastRetTime;
	bool retIsReset;
	public:
	//約(1x10^6)/(0x3fff)=61.036[Hz]
	static const int TIME_MASK=0x3fff;
	SoftTimer():lastRetTime(0){
		t.start();
	}
	void calc(){
		retTime=t.read_us()&TIME_MASK;
		
		// /|/|こういう波形の立下りのところかどうか
		retIsReset=retTime<lastRetTime;
		
		lastRetTime=retTime;
	}
	bool getIsReset()const{
		return retIsReset;
	}
	int getTime()const{
		return retTime;
	}
};

class aSoftPWM{
	private:
	static const int INSTANCES_LEN=255;
	static int instancesIndex;
	static aSoftPWM *instances[INSTANCES_LEN];
	static SoftTimer timer;
	static Ticker ticker;
	static bool isNotTickerStart;
	
	static int addInstance(aSoftPWM *p);
	static void delInstance(const int index);
	
	
	public:
	static void update();
	
	private:
	DigitalOut out;
	bool isOn;
	int turnOffTime;
	int selfInstanceIndex;
	void turnOnInternal(){
		if(turnOffTime!=0){
			out=1;
		}
		isOn=true;
	}
	void turnOffInternal(const int time){
		if(isOn){
			if(turnOffTime<=time){
				out=0;
				isOn=false;
			}
		}
	}
	public:
	aSoftPWM(const PinName pinArg):
		out(pinArg),isOn(false),turnOffTime(0)
	{
		selfInstanceIndex=addInstance(this);
		if(isNotTickerStart){
			//分解度2^(7)=128
			ticker.attach_us(callback(aSoftPWM::update),timer.TIME_MASK>>7);
			isNotTickerStart=false;
		}
	}
	~aSoftPWM(){
		delInstance(selfInstanceIndex);
	}
	
	float set(const float mult);
	
	float operator=(const float mult){
		return set(mult);
	}
};

}
using aSoftPWM_internal::aSoftPWM;
}



#endif