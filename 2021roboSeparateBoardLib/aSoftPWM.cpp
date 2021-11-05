#include "aSoftPWM.hpp"


namespace rob{
namespace aSoftPWM_internal{
	
//const int aSoftPWM::INSTANCES_LEN=255;
int aSoftPWM::instancesIndex=0;
aSoftPWM *aSoftPWM::instances[INSTANCES_LEN];
SoftTimer aSoftPWM::timer;
Ticker aSoftPWM::ticker;
bool aSoftPWM::isNotTickerStart=true;

int aSoftPWM::addInstance(aSoftPWM *p){
	if(!(instancesIndex<INSTANCES_LEN)){
		return INSTANCES_LEN;
	}
	instances[instancesIndex]=p;
	return instancesIndex++;
}
void aSoftPWM::delInstance(const int index){
	if(index==INSTANCES_LEN){
		return;
	}
	instances[index]=NULL;
}

void aSoftPWM::update(){
	timer.calc();

	for(int i=0;i<instancesIndex;i++){
		if(instances[i]==NULL){
			continue;
		}
		if(timer.getIsReset()){
			instances[i]->turnOnInternal();
		}
		instances[i]->turnOffInternal(timer.getTime());
	}
}


float aSoftPWM::set(const float mult){
	turnOffTime=mult*timer.TIME_MASK;
	return mult;
}

}
}