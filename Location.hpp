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

	virtual void addVehicleId(int veh_id);
	virtual void removeVehicleId(int veh_id);
	virtual void getVehicleIds(std::unordered_set<int> *veh_ids);
	virtual void clearVehicleIds();
	virtual int getNumVehicles();

	virtual void setCapacity(int capacity);
	virtual int getCapacity() const;
private:
	std::unordered_set<int> vehicle_ids_;
	int capacity_;
};

} /* namespace amod */

#endif /* LOCATION_HPP_ */
