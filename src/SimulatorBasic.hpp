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

#include <cmath>
#include <unordered_map>
#include <random>
#include <iostream>

namespace amod {

class SimulatorBasic: public Simulator {
public:
    SimulatorBasic(double resolution = 0.1);
	virtual ~SimulatorBasic();

	virtual amod::ReturnCode  init(amod::World *world_state);
    virtual amod::ReturnCode  update(amod::World *world_state);

    // low level commands
    virtual amod::ReturnCode dispatchVehicle(int veh_id, const amod::Position &to);
    virtual amod::ReturnCode pickupCustomer(int veh_id, int cust_id);
    virtual amod::ReturnCode dropoffCustomer(int veh_id, int cust_id);


    // med level commands
    virtual amod::ReturnCode serviceBooking(const amod::Booking &booking);

    // distance functions
    virtual double getDrivingDistance(const amod::Position &from, const amod::Position &to);
    virtual double getDistance(const amod::Position &from, const amod::Position &to);
    
    // SimulatorBasic specific funtions
    virtual void setVehicleSpeedParams(double mean, double sd, double min, double max);
    virtual void setPickupDistributionParams(double mean, double sd, double min, double max);
    virtual void setDropoffDistributionParams(double mean, double sd, double min, double max);

private:
    double resolution_; // resolution of simulation in seconds
    amod::World state_; // ideally, the true simulator will maintain it's own internal state

    std::default_random_engine eng;
    std::normal_distribution<> normal_dist;
    
    int event_id_; // so that events have different ids
    
    struct Dispatch {
        int booking_id;  //0 if manual dispatch
    	int veh_id;
    	Position from;
    	Position to;
        Position grad; //normalized gradient
        Position curr;
        
    };
    
    struct Pickup {
        int booking_id;  //0 if manual dispatch
        int veh_id;
        int cust_id;
        double pickup_time;
        
    };
    
    struct Dropoff {
        int booking_id;  //0 if manual dispatch
        int veh_id;
        int cust_id;
        double dropoff_time;
        
    };
    
    struct TruncatedNormalParams {
        std::normal_distribution<>::param_type par;
        double min;
        double max;
    };

    std::unordered_map<int, Booking> bookings_;
    std::unordered_map<int, Dispatch> dispatches_;
    std::map<double, Pickup> pickups_;
    std::map<double, Dropoff> dropoffs_;
    
    // parameters for pickup distribution simulation
    TruncatedNormalParams pickup_params_;
    
    // parameters for dropoff distribution simulation
    TruncatedNormalParams dropoff_params_;
    
    // parameters for vehicle speeds
    TruncatedNormalParams speed_params_;

    // internal functions
    virtual void simulateVehicles(amod::World *world_state);
    virtual void simulateCustomers(amod::World *world_state);
    virtual void simulatePickups(amod::World *world_state);
    virtual void simulateDropoffs(amod::World *world_state);
    
    virtual amod::ReturnCode dispatchVehicle(int veh_id, const amod::Position &to, int booking_id);
    virtual amod::ReturnCode pickupCustomer(int veh_id, int cust_id, int booking_id);
    virtual amod::ReturnCode dropoffCustomer(int veh_id, int cust_id, int booking_id);
    
    
    virtual double genRandTruncNormal(TruncatedNormalParams &params);

    virtual bool hasArrived(const Dispatch &d);
    
    double eucDist(const Position &a, const Position &b);
};

} /* namespace amod */

#endif /* SimulatorBasic_HPP_ */
