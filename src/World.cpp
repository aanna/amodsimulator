/*
 * World.cpp
 *
 *  Created on: Mar 27, 2015
 *      Author: haroldsoh
 */

#include "World.hpp"

namespace amod {

World::World():  current_time_(0) {
	// TODO Auto-generated constructor stub

}

World::~World() {
	// TODO Auto-generated destructor stub
}

void World::populate(std::vector<Location> &locations, std::vector<Vehicle> &vehicles, std::vector<Customer> &customers) {
	for (auto l : locations) {
		locations_[l.getId()] = l;
	}
    
    for (auto v : vehicles) {
        vehicles_[v.getId()] = v;
    }
    
    for (auto c : customers) {
        customers_[c.getId()] = c;
    }
    
}

void World::addVehicle(const Vehicle &veh) {
	vehicles_[veh.getId()] = veh;
}

void World::setVehicle(const Vehicle &veh) {
	vehicles_[veh.getId()] = veh;
}

void World::addVehicles(const std::vector<Vehicle> &vehs) {
	for (auto v : vehs) {
		addVehicle(v);
	}
}


void World::removeVehicle(int veh_id) {
	vehicles_.erase(veh_id);
}

void World::removeVehicles(std::vector<int> &veh_ids) {
	for (auto id : veh_ids) {
		removeVehicle(id);
	}
}

Vehicle World::getVehicle(int veh_id) {
	auto it = vehicles_.find(veh_id);
	if (it != vehicles_.end()) {
		return it->second;
	}
	return Vehicle();
}

Vehicle * World::getVehiclePtr(int veh_id) {
	auto it = vehicles_.find(veh_id);
	if (it != vehicles_.end()) {
		return &(it->second);
	}
	return nullptr;
}


void World::getVehicles(std::vector<Vehicle> *vehs) const {
	if (!vehs) {
		throw std::runtime_error("World::getVehicles vehs pointer is null");
	}
	for (auto it=vehicles_.begin(); it != vehicles_.end(); ++it) {
		vehs->push_back(it->second);
	}
}
    
void World::getVehicles(std::unordered_map<int, Vehicle>::const_iterator* bitr,
		std::unordered_map<int, Vehicle>::const_iterator* eitr) const {
	*bitr = vehicles_.begin();
	*eitr = vehicles_.end();
}


int World::getNumVehicles() const {
	return (int) vehicles_.size();
}


void World::addCustomer(const Customer &cust) {
	customers_[cust.getId()] = cust;
}

void World::setCustomer(const Customer &cust) {
	customers_[cust.getId()] = cust;
}

void World::addCustomers(const std::vector<Customer> &custs) {
    for (auto v : custs) {
        addCustomer(v);
    }
}


void World::removeCustomer(int cust_id) {
	customers_.erase(cust_id);
}

void World::removeCustomers(std::vector<int> &cust_ids) {
    for (auto id : cust_ids) {
        removeCustomer(id);
    }
}

Customer World::getCustomer(int cust_id) {
    auto it = customers_.find(cust_id);
    if (it != customers_.end()) {
        return it->second;
    }
    return Customer();
}

Customer * World::getCustomerPtr(int cust_id) {
    auto it = customers_.find(cust_id);
    if (it != customers_.end()) {
        return &(it->second);
    }
    return nullptr;
}


void World::getCustomers(std::unordered_map<int, Customer>::const_iterator* bitr,
		std::unordered_map<int, Customer>::const_iterator* eitr) {
	*bitr = customers_.begin();
	*eitr = customers_.end();
}


void World::getCustomers(std::vector<Customer> *custs) {
    if (!custs) {
        throw std::runtime_error("World::getCustomers custs pointer is null");
    }
    for (auto it=customers_.begin(); it != customers_.end(); ++it) {
        custs->push_back(it->second);
    }
}


int World::getNumCustomers() {
    return (int) customers_.size();
}


void World::addLocation(const Location &loc) {
	locations_[loc.getId()] = loc;
}

void World::setLocation(const Location &loc) {
	locations_[loc.getId()] = loc;
}

void World::addLocations(const std::vector<Location> &locs) {
	for (auto loc : locs) {
		addLocation(loc);
	}
}

void World::removeLocation(int loc_id) {
	locations_.erase(loc_id);
}

void World::removeLocations(std::vector<int> &loc_ids) {
	for (auto id : loc_ids) {
		removeLocation(id);
	}
}

Location World::getLocation(int loc_id) {
	auto it = locations_.find(loc_id);
	if (it != locations_.end()) {
		return it->second;
	}
	return Location();
}

Location * World::getLocationPtr(int loc_id) {
	auto it = locations_.find(loc_id);
	if (it != locations_.end()) {
		return &(it->second);
	}
	return nullptr;
}


void World::getLocations(std::vector<Location> *locs) {
	if (!locs) {
		throw std::runtime_error("World::getLocations: locs pointer is null");
	}
	for (auto it=locations_.begin(); it != locations_.end(); ++it) {
		locs->push_back(it->second);
	}
}

void World::getLocations(std::unordered_map<int, Location>::const_iterator* bitr,
		std::unordered_map<int, Location>::const_iterator* eitr) {
	*bitr = locations_.begin();
	*eitr = locations_.end();
}

int World::getNumLocations() {
	return (int) locations_.size();
}

void World::addEvent(Event &event) {
	events_[event.id] = event;
}

void World::setEvent(Event &event) {
	events_[event.id] = event;
}

void World::addEvents(const std::vector<Event> &events) {
	for (auto event : events) {
		addEvent(event);
	}
}


void World::getEvents(std::vector<Event> *events) {
	if (!events) {
		throw std::runtime_error("World::getEvents: events pointer is null");
	}
	for (auto it=events_.begin(); it != events_.end(); ++it) {
		events->push_back(it->second);
	}
}


void World::removeEvent(int event_id) {
	auto it = events_.find(event_id);
	if (it != events_.end()) {
		events_.erase(it);
	}
}

void World::clearEvents() {
	events_.clear();
}


int World::getNumEvents() {
	return (int) events_.size();
}

double World::getCurrentTime() {
	return current_time_;
}

void World::setCurrentTime(double current_time) {
	current_time_ = current_time;
}


} /* namespace amod */
