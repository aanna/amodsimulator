/*
 * AMODSimulator.h
 *
 *  Created on: Mar 23, 2015
 *      Author: haroldsoh
 */

#ifndef AMODSIMULATOR_H_
#define AMODSIMULATOR_H_

#include "AMODTypes.hpp"
#include "AMODObject.hpp"
#include "AMODCar.hpp"
#include <vector>


namespace AMODBase {

class AMODSimulator {
public:
	AMODSimulator() {};

	virtual ~AMODSimulator() {};

	virtual int init() = 0;
    virtual int update() = 0;
    
	virtual AMODBase::ReturnCode createObjectsInWorld(const std::vector<AMODBase::AMODObject*> &objects) = 0;
    
    // update o's position
	virtual AMODBase::Position updateObjectPosition(AMODBase::AMODObject *o) = 0;
    
	virtual AMODBase::ReturnCode dispatchVehicle(AMODBase::AMODCar *car, const AMODBase::Position &to) = 0;
	virtual void getArrivals(std::vector<std::pair<int, double>> *arrivals ) = 0;
	virtual void clearArrivals() = 0; // call after getting arrivals to clear the queue
    
   	virtual AMODBase::ReturnCode pickupPassenger(AMODBase::AMODCar *car) = 0;
    virtual void getPickups(std::vector<std::pair<int, double>> *pickups ) = 0;
    virtual void clearPickups() = 0; // call after getting pickups to clear the queue
    
    virtual double getDrivingDistance(const AMODBase::Position &from, const AMODBase::Position &to) = 0;
    virtual double getDistance(const AMODBase::Position &from, const AMODBase::Position &to) = 0;
    
    virtual double getCurrentTime() = 0; //get the current time in seconds.

    
};

} /* namespace AMOD */

#endif /* AMODSIMULATOR_H_ */
