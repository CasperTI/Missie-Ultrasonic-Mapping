//file SLAM.hpp
/**
*               __
*    _________ / /_  ____  ________  ____________  _____
*   /___/ __ \/ __ \/ __ \/ ___/ _ \/ ___/ ___/ / / / _ \
*  / / / /_/ / /_/ / /_/ / /  /  __(__  ) /__/ /_/ /  __/
* /_/  \____/_.___/\____/_/   \___/____/\___/\__,_/\___/
*
*
* @file SLAM.hpp
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

#ifndef SLAM_HPP_
#define SLAM_HPP_

#include <iostream>
#include <vector>
#include <cmath>


class UltraSonicMap{
public:
	/**
	 * The constructor
	 */
	UltraSonicMap();
	/**
	 * A function to convert an X and Y coordinate to an index in the 1-dimensional vector in the LOCAL map (always same size)
	 * @param curX the X coordinate
	 * @param curY the Y coordinate
	 * @return the matching index
	 */
	int toIndex(int curX, int curY);
	/**
	 *A function to add an extra row to the global map
	 *@param theVector
	 *@param direction an enum value
	 */
	std::vector<int> insertRow(std::vector<int> theVector, int direction);// 1 = +x  2 = -x  3 = +y 4 = -y
	/**
	 * The function to add the local map measurements to the global map, using the offset given by mapOffset. At this stage the global map is already enlarged to fit the whole local map
	 * @param localMap a vector containing the local map measurements
	 * @param mapOffset the offset from the upper left corner of the global map to the upper left corner of the local map
	 */
	void addMaps(std::vector<int> localMap, int* mapOffset);// take the global map and add the local map measurements on the correct location to it, at this stage the parts that fall outside of the old global map have been added as empty area already
	/**
	 * The processing of the global map after the local map has been made. This function takes the local map, calculates how and where to enlarge the global map and then calls addMaps to add the measurements to the global map
	 * @param localMap a std::vector containing the local map measurements
	 */
	void processGlobalMap(std::vector<int> localMap);
	/**
	 * A function to calculate the X and Y coordinates a vector from 0,0 with the given distance and angle points to.
	 * @param givenAngle a double value to determine the angle of the vector
	 * @param givenDistance a double value containing the length of the vector
	 * @param theComponents the vector the result is written to
	 * @return an array with the x coordinate and y coordinate respectively at index [0] and [1]
	 */
	int* getComponents(double givenAngle,double givenDistance,int* theComponents);
	/**
	 * Creates the local map (currently measured values) by processing the sensorreadings onto an empty constant-sized temporary map
	 * @param theSensorReadings the measured sensor values
	 */
	std::vector<int> createLocalMap(int *theSensorReadings);

	int posOffset[2] = { 0,0 };/**< the offset of the ROSBEE from the point it started at */
	int rotation = 0; /**< the current rotation of the ROSBEE, rounded to an integer value and ranging from -180 to 180 degrees */
	int Speed = 0; /**< the speed of the ROSBEE in cm per tick */

private:
	const double PI = 3.14159265358979323846; /**<  a simple solution to using PI */

	const static int sensorCount = 5; /**< Amount of ultrasonic sensors or any kind of sensor for that matter */

	const static int xReach = 38; /**< the X component of the area the sensors can reach when rotated */
	const static int yReach = 38;/**< the Y component of the area the sensors can reach when rotated */

	const static int ROSBEE = -1; /**< the 'ID' of the ROSBEE itself, converted to an R when shown */
	const static int OBJECT = 1; /**< the amount to be added to the map when something is detected at said position */

	const int sensorAngles[5] = { -170, -30, 0, 30, 170 }; /**< The angles of the sensors, can't be more than sensor count */
	const int sensorDistances[5] = { 10, 10, 10, 10, 10 }; /**< distance from each sensor to middle of ROSBEE, basically an offset from 0,0 in the direction of the corresponding sensorAngle */

	int xLen = (xReach * 2) + 1;/**< Total width of the map, like the Y component both are the max range of the sensors multiplied by two with an extra 1, so the number is always odd in order to have a middle row */
	int yLen = (yReach * 2) + 1;/**< Total length of the map, works like the aforementioned X component */
	int xGlobal = xLen; /**< Width of global map, this is initially the same as the local map but grows as the ROSBEE moves */
	int yGlobal = yLen; /**< Length of global map, this is initially the same as the local map but grows as the ROSBEE moves */

	int mapOffset[2] = {0,0}; /**< The total offset of the global map upper left corner to the local map upper left corner */

	int rowsAdded[4]  = {0,0,0,0}; /**<  Rows added in respectively x+ x- y+ y- directions seen from localmap's 0,0 */

	int cmSquare = 10; /**< The distance '1' represents in the map in centimeters */

	bool newMap = true; /**< When true, the global map is a copy of the local map, afterwards this is set to false; basically a debounce */
	bool easyMode = true; /**< Used for testing, setting easymode to true replaces all numeric values in the map with 'B'-s; note that the ROSBEE's 'R' remains unchanged */

	std::vector<int> theGlobalMap; /**< The actual global map, as a vector */
};


#endif /* SLAM_HPP_ */
