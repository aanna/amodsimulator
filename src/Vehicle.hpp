/*
 * Vehicle.h
 *
 *  Created on: Mar 23, 2015
 *      Author: haroldsoh
 */

#ifndef Vehicle_H_
#define Vehicle_H_

#include "Types.hpp"
#include "Entity.hpp"

#include <unordered_set>
#include <list>

namespace amod {


class Vehicle : public Entity {
public:
	Vehicle(int id = 0);
	Vehicle(int id, std::string name, Position pos, int capacity, VehicleStatus status);
	virtual ~Vehicle();

	virtual VehicleStatus getStatus();
	virtual void setStatus(VehicleStatus s);

	virtual double getSpeed() const;
	virtual void setSpeed(double speed);

	virtual void setCustomerId(int cust_id);
    virtual int getCustomerId() const;
	virtual void clearCustomerId();

	virtual void setCapacity(int capacity);
	virtual int getCapacity() const;

	virtual void setWaypoints(std::list<Position> &waypoints);
	virtual void getWaypoints(std::list<Position> *waypoints);


private:
	VehicleStatus status_;
	int capacity_;
	double speed_;

	int customer_id_;
	std::list<Position> waypoints_;

};

} /* namespace AMOD */

#endif /* Vehicle_H_ */
