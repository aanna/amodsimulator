/*
 * Customer.hpp
 *
 *  Created on: Mar 27, 2015
 *      Author: haroldsoh
 */

#ifndef CUSTOMER_HPP_
#define CUSTOMER_HPP_

#include "Entity.hpp"

namespace amod {

class Customer: public amod::Entity {
public:
	Customer();
	Customer(int id, std::string name, amod::Position pos, int assigned_vehicle, bool in_vehicle_);
	virtual ~Customer();

	virtual void setAssignedVehicleId(int veh_id);
	virtual void clearAssignedVehicleId();
	virtual int getAssignedVehicleId();

	virtual void setInVehicle(bool in_vehicle);
	virtual bool isInVehicle();

private:
	bool in_vehicle_;
	int veh_id_;
};

} /* namespace AMODBase */

#endif /* CUSTOMER_HPP_ */
