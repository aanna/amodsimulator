/*
 * Location.hpp
 *
 *  Created on: Mar 27, 2015
 *      Author: haroldsoh
 */

#ifndef LOCATION_HPP_
#define LOCATION_HPP_

#include "Types.hpp"
#include "Entity.hpp"

#include <unordered_set>
#include <vector>
#include <string>

namespace amod {

class Location : public Entity {
public:
	Location ();
	Location(int id, std::string name, Position pos, int capacity);
	virtual ~Location();

	// Functions to modify the vehicles at this location
	// Location vehicles are free vehicles in this area.
	// but have not been picked up from this location. It does not
	// include the people who have been dropped off at this location.
	virtual int getNumVehicles() const;
	virtual void addVehicleId(int veh_id);
	virtual void removeVehicleId(int veh_id);
	virtual void getVehicleIds(std::unordered_set<int> *veh_ids);
	virtual void getVehicleIds(std::unordered_set<int>::const_iterator *bitr, std::unordered_set<int>::const_iterator *eitr);
	virtual void clearVehicleIds();

	// Functions to modify the customers at this location
	// Location customers are people who are currently at this node
	// i.e., those instantiated here or have been dropped off.
	virtual int getNumCustomers() const;
	virtual void getCustomerIds(std::unordered_set<int> *cust_ids);
	virtual void getCustomerIds(std::unordered_set<int>::const_iterator *bitr, std::unordered_set<int>::const_iterator *eitr);
	virtual void addCustomerId(int cust_id);
	virtual void removeCustomerId(int cust_id);
	virtual void clearCustomerIds();

	virtual void setCapacity(int capacity);
	virtual int getCapacity() const;

	// operators for KDTree operation
	// inlined for convenience
	int dims() const { return 2; };
	double & operator[](int i) { return getPosition()[i]; };
	double operator[](int i) const { return getPosition()[i]; };

	bool operator==(Location &rhs) {
		// two locations that have the same position are considered equal
		return (getPosition() == rhs.getPosition());
	}

private:
	std::unordered_set<int> vehicle_ids_;
	std::unordered_set<int> customer_ids_;
	int capacity_;
};

} /* namespace amod */

#endif /* LOCATION_HPP_ */
