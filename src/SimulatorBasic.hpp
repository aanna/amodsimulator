/*
 * SimulatorBasic.hpp
 *
 *  Created on: Mar 27, 2015
 *      Author: haroldsoh
 */

#ifndef SimulatorBasic_HPP_
#define SimulatorBasic_HPP_

#include "World.hpp"
#include "Simulator.hpp"
#include "Booking.hpp"
#include "Utility.hpp"
#include "KDTree.hpp"

#include <cmath>
#include <unordered_map>
#include <random>
#include <iostream>
#include <ctime>
#include <cstdio>

namespace amod {

class SimulatorBasic: public Simulator {
public:
    // constructor
    // initalize the simulator, optionally with a simulation resolution in seconds
    // default resolution is 0.1 seconds.
    SimulatorBasic(double resolution = 0.1);
	virtual ~SimulatorBasic();

    // init
    // initializes the Simulator with the world_state.
    // if the call is successful, it returns amod::SUCESSS. Otherwise, it returns
    // one of the amod::ReturnCode error codes.
	virtual amod::ReturnCode  init(amod::World *world_state);
    
    // update
    // updates the world_state
    // if the call is successful, it returns amod::SUCESSS. Otherwise, it returns
    // one of the amod::ReturnCode error codes.
    virtual amod::ReturnCode  update(amod::World *world_state);

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
                                             amod::VehicleStatus veh_start_status = VehicleStatus::BUSY,
                                             amod::VehicleStatus veh_end_status = VehicleStatus::FREE
                                             );
    
    // pickupCustomer
    // picks up customer with id cust_id using vehicle with id veh_id. If the call is successful,
    // the vehicle status is status is set to start_status. After the customer is picked up,
    // an event is triggered and the vehicle's status is set to end_status.
    // if the call is successful, it returns amod::SUCESSS. Otherwise, it returns
    // one of the amod::ReturnCode error codes.
    virtual amod::ReturnCode pickupCustomer(amod::World *world_state,
                                            int veh_id, int cust_id,
                                            amod::VehicleStatus start_status = VehicleStatus::PICKING_UP,
                                            amod::VehicleStatus end_status = VehicleStatus::HIRED);
    
    // dropoffCustomer
    // drops off customer with id cust_id using vehicle with id veh_id. If the call is successful,
    // the vehicle status is status is set to start_status. After the customer is dropped off,
    // an event is triggered and the vehicle's status is set to end_status.
    // if the call is successful, it returns amod::SUCESSS. Otherwise, it returns
    // one of the amod::ReturnCode error codes.
    virtual amod::ReturnCode dropoffCustomer(amod::World *world_state,
                                             int veh_id, int cust_id,
                                             amod::VehicleStatus status = VehicleStatus::DROPPING_OFF,
                                             amod::VehicleStatus end_status = VehicleStatus::FREE);


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
											 amod::CustomerStatus cust_end_status = CustomerStatus::FREE);


    // setCustomerStatus
    // sets a customer status
    virtual void setCustomerStatus(amod::World *world_state, int cust_id, CustomerStatus status);


    // Medium level commands, i.e., makes basic tasks easier to do with default events
    // automatically triggered.
    
    
    // serviceBooking
    // services the amod::Booking booking.
    // For teleportation, this simply teleports the customer to the destination.
    // For amod travel, this automatically simulates servicing a booking call
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
    virtual amod::ReturnCode serviceBooking(amod::World *world_state, const amod::Booking &booking);

    // distance functions
    // returns the driving distance from Position from to Position to. This may not be the Euclidean
    // distance on a road network.
    virtual double getDrivingDistance(const amod::Position &from, const amod::Position &to);
    
    // getDistance
    // returns the Euclidean distance from Position from to Position to.
    virtual double getDistance(const amod::Position &from, const amod::Position &to);
    
    // SimulatorBasic specific functions
    
    // Sets the parameters of the distributions used in the basic simulator. All the
    // distributions used are truncated normals with parameters mean, standard dev,
    // minimum and maximum.
    
    // setVehicleSpeedParams
    // sets the vehicles speed distribution parameters in m/s
    virtual void setVehicleSpeedParams(double mean, double sd, double min, double max);
    
    // sets the pickup time distribution parameters in seconds
    virtual void setPickupDistributionParams(double mean, double sd, double min, double max);
    
    // sets the dropoff time distribution parameters in seconds
    virtual void setDropoffDistributionParams(double mean, double sd, double min, double max);

    // sets the teleportation time distribution parameters in seconds
    virtual void setTeleportDistributionParams(double mean, double sd, double min, double max);
private:
    double resolution_; // resolution of simulation in seconds
    amod::World state_; // ideally, the true simulator will maintain it's own internal state

    // KDTree to store locations
    kdt::KDTree<amod::Location> loc_tree_;
    bool using_locations_;

    // objects for random number generation
    std::default_random_engine eng;
    std::normal_distribution<> normal_dist;
    
    long long event_id_; // so that events have different ids
    
    // structures to manage dispatches, pickups and dropoffs
    struct Dispatch {
        int booking_id;  //0 if manual dispatch
    	int veh_id;
    	int to_loc_id;
        int from_loc_id;
    	Position from;
    	Position to;
        Position grad; //normalized gradient
        Position curr;
        amod::VehicleStatus veh_end_status;
    };
    
    struct Pickup {
        int booking_id;  //0 if manual dispatch
        int veh_id;
        int cust_id;
        int loc_id;
        double pickup_time;
        amod::VehicleStatus veh_end_status;
    };
    
    struct Dropoff {
        int booking_id;  //0 if manual dispatch
        int veh_id;
        int cust_id;
        int loc_id;
        double dropoff_time;
        amod::VehicleStatus veh_end_status;
    };
    
    struct Teleport {
    	int cust_id;
    	int loc_id;
    	double teleport_arrival_time;
    	amod::CustomerStatus cust_end_status;
    };

    struct TruncatedNormalParams {
        std::normal_distribution<>::param_type par;
        double min;
        double max;
    };

    std::unordered_map<int, Booking> bookings_;
    std::unordered_map<int, Dispatch> dispatches_;
    std::multimap<double, Pickup> pickups_;
    std::multimap<double, Dropoff> dropoffs_;
    std::multimap<double, Teleport> teleports_;
    
    // parameters for pickup distribution simulation
    TruncatedNormalParams pickup_params_;
    
    // parameters for dropoff distribution simulation
    TruncatedNormalParams dropoff_params_;
    
    // parameters for vehicle speeds
    TruncatedNormalParams speed_params_;

    // parameters for teleportation time
    TruncatedNormalParams teleport_params_;

    // internal functions
    virtual void simulateVehicles(amod::World *world_state);
    virtual void simulateCustomers(amod::World *world_state);
    virtual void simulatePickups(amod::World *world_state);
    virtual void simulateDropoffs(amod::World *world_state);
    virtual void simulateTeleports(amod::World *world_state);
    
    
    // internal helper functions that really execute the dispatch, pickup and dropoff functions
    
    virtual amod::ReturnCode dispatchVehicle(amod::World *world_state,
                                             int veh_id,
                                             const amod::Position &to,
                                             amod::VehicleStatus veh_start_status,
                                             amod::VehicleStatus veh_end_status,
                                             int booking_id);
    
    virtual amod::ReturnCode pickupCustomer(amod::World *world_state,
                                            int veh_id,
                                            int cust_id,
                                            amod::VehicleStatus start_status,
                                            amod::VehicleStatus end_status,
                                            int booking_id);
    
    
    virtual amod::ReturnCode dropoffCustomer(amod::World *world_state,
                                             int veh_id,
                                             int cust_id,
                                             amod::VehicleStatus start_status,
                                             amod::VehicleStatus end_status,
                                             int booking_id);
    
    // generates the random truncated normal
    virtual double genRandTruncNormal(TruncatedNormalParams &params);

    // checks if the vehicle specified in the Dispatch d has arrived
    virtual bool hasArrived(const Dispatch &d);
    
    // returns the euclidean distance between a and b
    double eucDist(const Position &a, const Position &b);
};

} /* namespace amod */

#endif /* SimulatorBasic_HPP_ */
