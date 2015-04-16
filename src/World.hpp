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
#include <unordered_map>

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

    // accessors
    virtual double getCurrentTime();
    
    virtual Vehicle getVehicle(int veh_id);
    virtual Vehicle * getVehiclePtr(int veh_id);
    virtual void getVehicles(std::vector<Vehicle> *vehs) const;
    virtual void getVehicles(std::unordered_map<int, Vehicle>::const_iterator* bitr,
    		std::unordered_map<int, Vehicle>::const_iterator* eitr) const;
    virtual int getNumVehicles() const;
    
    virtual Location getLocation(int loc_id);
    virtual Location * getLocationPtr(int loc_id);
    virtual void getLocations(std::vector<Location> *locs);
    virtual void getLocations(std::unordered_map<int, Location>::const_iterator* bitr,
    		std::unordered_map<int, Location>::const_iterator* eitr);
    virtual int getNumLocations();
    
    virtual Customer getCustomer(int cust_id);
    virtual Customer* getCustomerPtr(int cust_id);
    virtual void getCustomers(std::vector<Customer> *custs);
    virtual void getCustomers(std::unordered_map<int, Customer>::const_iterator* bitr,
    		std::unordered_map<int, Customer>::const_iterator* eitr);
    virtual int getNumCustomers();
    
    virtual void getEvents(std::vector<Event> *events);
    virtual int getNumEvents();
    
    // adders, setters and removers
	virtual void addVehicle(const Vehicle &veh);
	virtual void setVehicle(const Vehicle &veh);
	virtual void addVehicles(const std::vector<Vehicle> &vehs);
	virtual void removeVehicle(int veh_id);
	virtual void removeVehicles(std::vector<int> &veh_ids);

    virtual void addLocation(const Location &loc);
    virtual void setLocation(const Location &loc);
    virtual void addLocations(const std::vector<Location> &locs);
    virtual void removeLocation(int loc_id);
    virtual void removeLocations(std::vector<int> &loc_ids);
    
    virtual void addCustomer(const Customer &cust);
    virtual void setCustomer(const Customer &cust);
    virtual void addCustomers(const std::vector<Customer> &custs);
    virtual void removeCustomer(int cust_id);
    virtual void removeCustomers(std::vector<int> &cust_ids);

	virtual void addEvent(Event &event);
	virtual void setEvent(Event &event);
	virtual void addEvents(const std::vector<Event> &events);
	virtual void removeEvent(int event_id);
	virtual void clearEvents();

	virtual void setCurrentTime(double current_time);

private:
	std::unordered_map<int, Vehicle> vehicles_;
	std::unordered_map<int, Location> locations_;
	std::unordered_map<int, Customer> customers_;
	std::map<long long, Event> events_;
	double current_time_;
    
    
    
};

} /* namespace amod */

#endif /* WORLD_HPP_ */
