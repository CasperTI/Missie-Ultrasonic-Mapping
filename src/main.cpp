/*
 * main.cpp
 *
 *  Created on: 28 aug. 2015
 *      Author: Casper
 */

#include "SLAM.hpp"

int main(){
	UltraSonicMap theMap;
	int sensorResults[5] = { 374, 374, 374, 374, 374 }; //Should be initialized at 375
	//PropCom propcom();
	//while(1){
		/*for (int sensorNum; sensorNum < sensorCount; sensorNum++){
			sensorResults[sensorNum] = propcom::getDistance(sensorNum);
		}*/
		theMap.Speed = 1;
		theMap.rotation = 0;
		for (int newOffset = 0; newOffset < 50;++ newOffset){
			std::cout << "Speed is " << theMap.posOffset[0] << " and " << theMap.posOffset[1] << std::endl;
			std::vector<int> curMap = theMap.createLocalMap(sensorResults);
			theMap.processGlobalMap(curMap);
		}
	//}
	return 0;
}


