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

#include <cmath>
#include <unordered_map>

namespace amod {

class SimulatorBasic: public Simulator {
public:
	SimulatorBasic();
	virtual ~SimulatorBasic();

	virtual amod::ReturnCode  init(amod::World *world_state);
    virtual amod::ReturnCode  update(amod::World *world_state);

    // low level commands
    virtual amod::ReturnCode dispatchVehicle(int veh_id, const amod::Position &to);
    virtual amod::ReturnCode pickupCustomer(int veh_id, int cust_id);
    virtual amod::ReturnCode dropoffCustomer(int veh_id, int cust_id);

    // med level commands
    virtual amod::ReturnCode serviceBooking(int veh_id, const amod::Booking &booking);

    // distance functions
    virtual double getDrivingDistance(const amod::Position &from, const amod::Position &to);
    virtual double getDistance(const amod::Position &from, const amod::Position &to);

private:
    amod::World state_;

    struct Dispatch {
    	int veh_id;
    	Position from;
    	Position to;
    };

    std::unordered_map<int, Dispatch> current_dispatches;

    virtual void simulateVehicles(amod::World *world_state);
    virtual void simulatePickups(amod::World *world_state);
    virtual void simulateDropoffs(amod::World *world_state);

};

} /* namespace amod */

#endif /* SimulatorBasic_HPP_ */
