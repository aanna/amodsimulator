/*
 * World.hpp
 *
 *  Created on: Mar 27, 2015
 *      Author: haroldsoh
 */

#ifndef WORLD_HPP_
#define WORLD_HPP_

#include <vector>
#include <map>

#include "Vehicle.hpp"
#include "Location.hpp"
#include "Customer.hpp"
#include "Types.hpp"
#include "Event.hpp"

namespace amod {

class World {
public:
	World();
	virtual ~World();

    virtual void populate(std::vector<Location> &locations,
                          std::vector<Vehicle> &vehicles,
                          std::vector<Customer> &customers);

	virtual void addVehicle(const Vehicle &veh);
	virtual void setVehicle(const Vehicle &veh);
	virtual void addVehicles(std::vector<Vehicle> &vehs);
	virtual void removeVehicle(int veh_id);
	virtual void removeVehicles(std::vector<int> &veh_ids);
	virtual Vehicle getVehicle(int veh_id);
	virtual void getVehicles(std::vector<Vehicle> *vehs);
	virtual int getNumVehicles();

    virtual void addLocation(const Location &loc);
    virtual void setLocation(const Location &loc);
    virtual void addLocations(std::vector<Location> &locs);
    virtual void removeLocation(int loc_id);
    virtual void removeLocations(std::vector<int> &loc_ids);
    virtual Location getLocation(int loc_id);
    virtual void getLocations(std::vector<Location> *locs);
    virtual int getNumLocations();

    virtual void addCustomer(const Customer &cust);
    virtual void setCustomer(const Customer &cust);
    virtual void addCustomers(std::vector<Customer> &custs);
    virtual void removeCustomer(int cust_id);
    virtual void removeCustomers(std::vector<int> &cust_ids);
    virtual Customer getCustomer(int cust_id);
    virtual void getCustomers(std::vector<Customer> *custs);
    virtual int getNumCustomers();

	virtual void addEvent(Event &event);
	virtual void setEvent(Event &event);
	virtual void addEvents(const std::vector<Event> &events);
	virtual void getEvents(std::vector<Event> *events);
	virtual void removeEvent(int event_id);
	virtual void clearEvents();
	virtual int getNumEvents();

	virtual double getCurrentTime();
	virtual void setCurrentTime(double current_time);

private:
	std::map<int, Vehicle> vehicles_;
	std::map<int, Location> locations_;
	std::map<int, Customer> customers_;
	std::map<int, Event> events_;
	double current_time_;
};

} /* namespace amod */

#endif /* WORLD_HPP_ */
