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
	enum Status {
		FREE,
		WAITING_FOR_PICKUP,
		IN_VEHICLE,
		WAITING_FOR_DROPOFF,
		WAITING_FOR_ASSIGNMENT,
	};

public:
	Customer(int id = 0, std::string name = "", amod::Position pos = Position(), int assigned_vehicle = 0, bool in_vehicle_ = false);
	virtual ~Customer();

	virtual void setAssignedVehicleId(int veh_id);
	virtual void clearAssignedVehicleId();
	virtual int getAssignedVehicleId();

	virtual void setStatus(Status s);
	virtual Status getStatus() const;

	// convenience functions
	virtual void setInVehicle();
	virtual bool isInVehicle();

private:
	Status status_;
	int veh_id_;
};

typedef Customer::Status CustomerStatus;

} /* namespace AMODBase */

#endif /* CUSTOMER_HPP_ */
