//file SLAM.cpp
/**
*               __
*    _________ / /_  ____  ________  ____________  _____
*   /___/ __ \/ __ \/ __ \/ ___/ _ \/ ___/ ___/ / / / _ \
*  / / / /_/ / /_/ / /_/ / /  /  __(__  ) /__/ /_/ /  __/
* /_/  \____/_.___/\____/_/   \___/____/\___/\__,_/\___/
*
*
* @file SLAM.cpp
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

UltraSonicMap::UltraSonicMap(){

}


int UltraSonicMap::toIndex(int curX, int curY){
	int returnVal = curY > 1 ? ((curY - 1) * xLen) + curX : curX;
	return returnVal;
}

std::vector<int> UltraSonicMap::insertRow(std::vector<int> theVector, int direction){ // 1 = +x  2 = -x  3 = +y 4 = -y
	int emptySpace = 0;
	switch (direction){
	case 1: // add row to the left
	{
		xGlobal++;
		for (int i = 0; i < (yGlobal); i++){
			theVector.insert(
					theVector.begin() + (xGlobal * (i))
			,1,emptySpace);
		}
	}
		break;
	case 2: // add row to the right
		{
			xGlobal++;
			for (int i = 0; i < (yGlobal - 1); i++){
				theVector.insert(
						theVector.begin() + (xGlobal * (i + 1) - 1)
				,1,emptySpace);
			}
		}
		break;
	case 3: // add row at the top
		theVector.insert(theVector.begin(),xGlobal,emptySpace);
		yGlobal++;
		break;
	case 4: // add row at the bottom
		theVector.insert(theVector.end() - 1,xGlobal,emptySpace);
		yGlobal++;
		break;
	default:
		break;
	}
	return theVector;
}

void UltraSonicMap::addMaps(std::vector<int> localMap, int* mapOffset){ // take the global map and add the local map measurements on the correct location to it, at this stage the parts that fall outside of the old global map have been added as empty area already
	for (int pos = 0; pos <  ( xLen * yLen) ; pos++){

		// calculate offset for pos in globalmap, globalmap dimensions are xGlobal times yGlobal, local map uses the constant xLen times yLen
		// 0,0 in local map is always the upper left corner
		// the corresponding 0,0 in the global map is is the global map upper left corner + mapOffset

		int curX = (pos % xLen) + (mapOffset[0]);
		int curY = ((pos - (pos % yLen))/yLen) + (mapOffset[1]);
		int globalIndex = curY == 0 ? curX : (curY) * xGlobal + curX;
		if ( localMap[pos] > 0 && localMap[pos] != ROSBEE) {
			theGlobalMap[globalIndex] += localMap[pos];
		} else if (localMap[pos] == ROSBEE) {
			theGlobalMap[globalIndex] = ROSBEE;
		}
	}
}

void UltraSonicMap::processGlobalMap(std::vector<int> localMap){
	// calculate (if any) how many rows to add and where
	int xRows = (posOffset[0] > 0 ? 1:-1) * (posOffset[0] - (posOffset[0] % cmSquare))/cmSquare+ (posOffset[0]%cmSquare != 0 ? 1:0); // total of rows extra needed in currently headed direction, as absolute value
	int yRows =  (posOffset[1] > 0 ? 1:-1) * (posOffset[1] - (posOffset[1] % cmSquare))/cmSquare+ (posOffset[1]%cmSquare != 0 ? 1:0);

	//rowsAdded =  rows added in respectively x+ x- y+ y- directions seen from localmap's 0,0

	int xRowsToAdd = xRows - (posOffset[0] >= 0 ? rowsAdded[0] : rowsAdded[1]);
	int yRowsToAdd = yRows - (posOffset[1] >= 0 ? rowsAdded[2] : rowsAdded[3]);

	if (xRowsToAdd > 0){
		rowsAdded[(posOffset[0] > 0 ? 0 : 1)]++;
		theGlobalMap = (posOffset[0] < 0 ? insertRow(theGlobalMap,2) : insertRow(theGlobalMap,1));
	}
	if (yRowsToAdd > 0){
		rowsAdded[(posOffset[1] > 0 ? 2 : 3)]++;
		theGlobalMap = (posOffset[1] < 0 ? insertRow(theGlobalMap,4) : insertRow(theGlobalMap,3));
	}
	if (newMap){
		newMap = false;
		theGlobalMap = localMap;
	} else {
		mapOffset[0] = ((rowsAdded[0] * cmSquare) - posOffset[0]) / cmSquare; // map distance from 0,0 is a possible extension of map minus the current position of the ROSBEE, as this is negative when it's far away thus making the result larger
		mapOffset[1] = ((rowsAdded[2] * cmSquare) - posOffset[1]) / cmSquare;
		addMaps(localMap,mapOffset);
	}
	std::string curLoc = "";
	std::cout << "The current rowsize is " << xGlobal << " and the current column size is " << yGlobal << std::endl << "Current map size is " << theGlobalMap.size() << std::endl;
	std::cout << "|";
	for (unsigned int i = 0; i < theGlobalMap.size(); i++){ // prints the GLOBAL map
		if (easyMode){
				std::cout << (theGlobalMap[i] != 0 ? (theGlobalMap[i] == ROSBEE ? "R" : "B") : "0");
		} else {
			std::cout << theGlobalMap[i];
		}
		curLoc += theGlobalMap[i];
		if ((i+1)%xGlobal  == 0 && i > 1){
			std::cout << "|" << std::endl << (i - 1 < theGlobalMap.size() ? "|" : " ");
		}
	}
}

int* UltraSonicMap::getComponents(double givenAngle,double givenDistance,int* theComponents){
	double curAngle = givenAngle;
	double detAngle;
	if (rotation >= 0){
		if (curAngle + rotation > 180) {
			detAngle = -180 + (rotation - (180 - curAngle));
		} else {
			detAngle = rotation + curAngle;
		}
	} else {
		if (curAngle + rotation < -180) {
			detAngle = 180 + (rotation + (180 + curAngle));
		} else {
			detAngle = curAngle + rotation;
		}
	}
	bool invertX = false;
	bool invertY = false;
	detAngle >= 0 ? invertX = false : invertX = true;
	if (invertX){
		detAngle *=-1;
	}
	detAngle > 90 ? invertY = true : invertY = false;
	if (invertY){
		detAngle = 180 - detAngle;
	}
	double distance = givenDistance;
	double xSlope,ySlope;
	if ((int) detAngle % 90 != 0){
		xSlope = xReach +  (
				cos(
						((int)detAngle % 90) * (PI / 180) // soscastoa, cos(alpha) divided by the distance from 0,0  equals the x component of the slope
				) * (
						distance/cmSquare
				)
			);
		ySlope  = yReach + (
				sin(
						((int)detAngle % 90) * (PI / 180) // soscastoa, sin(alpha) divided by the distance from 0,0  equals the y component of the slope
				) * (
						distance/cmSquare
				)
			);
	} else {
		xSlope = detAngle == 90 ? xReach : detAngle == 0 ? xLen - 1  : 0;
		ySlope = detAngle == 90 ? yLen - 1 : detAngle == 0 ? yReach : 0;
		if (invertY) {
			xSlope = xLen - xSlope;
		}
	}

	if (invertY && !(int)detAngle%90 == 0){
		xSlope = xLen - xSlope;
	}
	if (invertX && !(int)detAngle%90 == 0){
		ySlope = yLen - ySlope;
	}
	theComponents[0] = xSlope;
	theComponents[1] = ySlope;
	return theComponents;
}

std::vector<int> UltraSonicMap::createLocalMap(int *theSensorReadings) {
	std::vector<int> newMap(xLen * yLen);
	newMap.at(toIndex((xReach),(yReach))) = ROSBEE; // ROSBEE placed in the middle
	for (int sens = 0; sens < sensorCount; sens++){
		if (theSensorReadings[sens] < 375){
			int theComponents[2] = {0,0};
			int* theReturnval;
			theReturnval = getComponents(sensorAngles[sens],theSensorReadings[sens] + sensorDistances[sens],theComponents);
			theComponents[0] = theReturnval[0];
			theComponents[1] = theReturnval[1];
			int theIndex = toIndex((int)theComponents[0],(int)theComponents[1]);
			newMap.at(theIndex) =  OBJECT;
		}
	}
	return newMap;
}

