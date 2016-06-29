/*
 * Vehicle.cpp
 *
 *  Created on: Mar 23, 2015
 *      Author: haroldsoh
 */

#include "Vehicle.hpp"

namespace amod {
    
typedef Vehicle::Status VehicleStatus;

    Vehicle::Vehicle(int id) : status_(UNKNOWN), capacity_(1), speed_(0), customer_id_(0), customer2_id_(0), location_id_(0) {
        // TODO Auto-generated constructor stub
        Entity::setId(id);
    }
    
    Vehicle::Vehicle(int id, std::string name, Position pos, int capacity, VehicleStatus status) :
    Entity(id, name, pos), capacity_(capacity), status_(status), speed_(0), customer_id_(0), customer2_id_(0)
    {
        return;
    }
    
    Vehicle::~Vehicle() {
        // TODO Auto-generated destructor stub
    }
    
    VehicleStatus Vehicle::getStatus() const{
        return status_;
    }
    
    void Vehicle::setStatus(Vehicle::Status s) {
        status_ = s;
    }
    
    double Vehicle::getSpeed() const {
        return speed_;
    }
    
    void Vehicle::setSpeed(double speed) {
        speed_ = speed;
    };
    
    void Vehicle::setCustomerId(int cust_id) {
        customer_id_ = cust_id;
    }
    
    int Vehicle::getCustomerId() const {
        return customer_id_;
    }
    
    void Vehicle::clearCustomerId() {
        customer_id_ = 0;
    }
    
	void Vehicle::setSecondCustomerId(int cust2_id) {
		customer2_id_ = cust2_id;
	}

    int Vehicle::getSecondCustomerId() const {
    	return customer2_id_;
    }

	void Vehicle::clearSecondCustomerId() {
		customer2_id_ = 0;
	}

    
    void Vehicle::setCapacity(int capacity) {
        capacity_ = capacity;
    }
    
    int Vehicle::getCapacity() const {
        return capacity_;
    }
    
    void Vehicle::setWaypoints(std::list<Position> &waypoints) {
        waypoints_ = waypoints;
    }
    
    void Vehicle::getWaypoints(std::list<Position> *waypoints) {
        *waypoints = waypoints_;
    }
    
    int Vehicle::getLocationId() {
        return location_id_;
    }
    
    void Vehicle::setLocationId(int loc_id) {
        location_id_ = loc_id;
    }
    
    
} /* namespace AMOD */
