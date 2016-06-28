/*
 * Simulator.h
 *
 *  Created on: Mar 23, 2015
 *      Author: haroldsoh, Kasia
 */

#ifndef Simulator_H_
#define Simulator_H_

#include "Types.hpp"
#include "Entity.hpp"
#include "Vehicle.hpp"
#include "World.hpp"
#include "Booking.hpp"
#include <vector>


namespace amod {

class Simulator {
public:
	Simulator() : verbose_(false) {};

	virtual ~Simulator() {};

    // init
    // initializes the Simulator with the world_state.
    // if the call is successful, it returns amod::SUCESSS. Otherwise, it returns
    // one of the amod::ReturnCode error codes.
    virtual amod::ReturnCode  init(amod::World *world_state) = 0;
    
    // update
    // updates the world_state
    // if the call is successful, it returns amod::SUCESSS. Otherwise, it returns
    // one of the amod::ReturnCode error codes.
    virtual amod::ReturnCode  update(amod::World *world_state) = 0;
    
    // low level commands
    
    // dispatchVehicle
    // dispatches vehicle with id veh_id to Position to. If the call is successful,
    // the vehicle status is status is set to start_status. When the vehicle arrives,
    // an event is triggered and the vehicle's status is set to end_status.
    // if the call is successful, it returns amod::SUCESSS. Otherwise, it returns
    // one of the amod::ReturnCode error codes.
    virtual amod::ReturnCode dispatchVehicle(amod::World *world_state,
                                             int veh_id,
                                             const amod::Position &to,
                                             amod::VehicleStatus start_status = VehicleStatus::BUSY,
                                             amod::VehicleStatus end_status = VehicleStatus::FREE) = 0;
    
    virtual amod::ReturnCode dispatchSharedVehicle(amod::World *world_state,
    										int vehId, int booking1stID, int booking2ndID,
    										const amod::Position &firstPickup, const amod::Position &secondPickup,
    										const amod::Position &firstDropoff, const amod::Position &secondDropoff,
    					amod::VehicleStatus veh_start_status = VehicleStatus::BUSY,
    					amod::VehicleStatus veh_end_status = VehicleStatus::FREE) = 0;

    // pickupCustomer
    // picks up customer with id cust_id using vehicle with id veh_id. If the call is successful,
    // the vehicle status is status is set to start_status. After the customer is picked up,
    // an event is triggered and the vehicle's status is set to end_status.
    // if the call is successful, it returns amod::SUCESSS. Otherwise, it returns
    // one of the amod::ReturnCode error codes.
    virtual amod::ReturnCode pickupCustomer(amod::World *world_state,
                                            int veh_id, int cust_id,
                                            amod::VehicleStatus start_status = VehicleStatus::PICKING_UP,
                                            amod::VehicleStatus end_status = VehicleStatus::HIRED) = 0;
    
    // dropoffCustomer
    // drops off customer with id cust_id using vehicle with id veh_id. If the call is successful,
    // the vehicle status is status is set to start_status. After the customer is dropped off,
    // an event is triggered and the vehicle's status is set to end_status.
    // if the call is successful, it returns amod::SUCESSS. Otherwise, it returns
    // one of the amod::ReturnCode error codes.
    virtual amod::ReturnCode dropoffCustomer(amod::World *world_state,
                                             int veh_id, int cust_id,
                                             amod::VehicleStatus status = VehicleStatus::DROPPING_OFF,
                                             amod::VehicleStatus end_status = VehicleStatus::FREE) = 0;
    
    
    // teleportCustomer
    // teleports the customer with cust_id to location closest to Position to. This simulates
    // transport via train or some other mode that doesn't contribute to road congestion
    // When the custer arrives,
	// an event is triggered and the vehicle's status is set to end_status.
	// if the call is successful, it returns amod::SUCESSS. Otherwise, it returns
	// one of the amod::ReturnCode error codes.
	virtual amod::ReturnCode teleportCustomer(amod::World *world_state,
											 int cust_id,
											 const amod::Position &to,
											 amod::CustomerStatus cust_start_status = CustomerStatus::TELEPORTING,
											 amod::CustomerStatus cust_end_status = CustomerStatus::FREE) = 0;



    // setCustomerStatus
    // sets a customer status
    virtual void setCustomerStatus(amod::World *world_state, int cust_id, CustomerStatus status) = 0;


    // Medium level commands, i.e., makes basic tasks easier to do with default events
    // automatically triggered.
    
    
    // serviceBooking
    // services the amod::Booking booking. This automatically simulates servicing a booking call
    // from dispatch to dropoff. Specifically:
    // The vehicle specified by booking.veh_id is dispatched from it's position to the position of
    // booking.cust_id (with status MOVING_TO_PICKUP). Upon arrival, an event is triggered.
    // The vehicle then waits to picks up the customer with status PICKING_UP. Upon pickup, an
    // event is triggered and the vehicle is then dispatched to the position booking.destination
    // with status MOVING_TO_DROPOFF. Upon arrival at the destination, an arrival event is triggered
    // and the vehicle begins to drop off the customer, with status DROPPING_OFF. When the passenger
    // is dropped off, a dropped off event is triggered and the vehicle is set to FREE.
    // if the call is successful, it returns amod::SUCESSS. Otherwise, it returns
    // one of the amod::ReturnCode error codes.
    virtual amod::ReturnCode serviceBooking(amod::World *world_state, const amod::Booking &booking) = 0;
    
    virtual amod::ReturnCode serviceSharedBookings(amod::World *world_state, const amod::Booking &booking1,
    		const amod::Booking &booking2, int vehId1, int vehId2) = 0;

    // distance functions
    // returns the driving distance from Position from to Position to. This may not be the Euclidean
    // distance on a road network.
    virtual double getDrivingDistance(const amod::Position &from, const amod::Position &to) = 0;
    virtual double getDrivingDistance(int from_loc_id, int to_loc_id) = 0;
    
    // getDistance
    // returns the Euclidean distance from Position from to Position to.
    virtual double getDistance(const amod::Position &from, const amod::Position &to) = 0;

    virtual void setVerbose(bool v) {
        verbose_ = v;    
    }
    
    virtual bool getVerbose() {
        return verbose_;
    }
    
    
protected:
    bool verbose_;
    
};

} /* namespace AMOD */

#endif /* Simulator_H_ */
