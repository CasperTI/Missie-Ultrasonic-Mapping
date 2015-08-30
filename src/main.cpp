//file main.cpp

/**
*               __
*    _________ / /_  ____  ________  ____________  _____
*   /___/ __ \/ __ \/ __ \/ ___/ _ \/ ___/ ___/ / / / _ \
*  / / / /_/ / /_/ / /_/ / /  /  __(__  ) /__/ /_/ /  __/
* /_/  \____/_.___/\____/_/   \___/____/\___/\__,_/\___/
*
*
* @file main.cpp
* @date Created ...
*
* @author Casper Wolf
* @version 2.1
*
* @section LICENSE
* License: newBSD
*
* Copyright © 2015, HU University of Applied Sciences Utrecht.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
* - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
* - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
* - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
* THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
* BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
* GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
* OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/



#include "SLAM.hpp"

// A stand-alone main for use without the sensors, use with sensors commented out
/*
 *
 *  In this main, the rotation can be set (and altered troughout the process) by setting theMap.rotation, and the position of the ROSBEE can be set using
 *  the array posOffset, with [0] being the X and [1] the Y coordinate of the ROSBEE its movement.
 *
 */


int main(){
	UltraSonicMap theMap;
	int sensorResults[5] = { 374, 374, 374, 374, 374 }; //Should be initialized at 375
	//PropCom propcom();
	//while(1){
		/*for (int sensorNum; sensorNum < sensorCount; sensorNum++){
			sensorResults[sensorNum] = propcom::getDistance(sensorNum);
		}*/
		theMap.rotation = 0;
		for (int newOffset = 0; newOffset < 50;++ newOffset){
			std::vector<int> curMap = theMap.createLocalMap(sensorResults);
			theMap.processGlobalMap(curMap);
		}
	//}
	return 0;
}


