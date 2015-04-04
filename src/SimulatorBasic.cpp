/*
 * SimulatorBasic.cpp
 *
 *  Created on: Mar 27, 2015
 *      Author: haroldsoh
 */

#include "SimulatorBasic.hpp"

namespace amod {
    
    SimulatorBasic::SimulatorBasic(double resolution, bool verbose):
        resolution_(resolution), verbose_(verbose), event_id_(0)
    {
        // just return
    }
    
    SimulatorBasic::~SimulatorBasic() {
        // just return
    }
    
    amod::ReturnCode  SimulatorBasic::init(amod::World *world_state) {
        state_ = *world_state;
        return amod::SUCCESS;
    }
    
    amod::ReturnCode  SimulatorBasic::update(amod::World *world_state) {
        // updates the world state
        state_.setCurrentTime( world_state->getCurrentTime() + resolution_ );
        
        // simulate the vehicles
        simulateVehicles(world_state);
        
        // simulate the pickups
        simulatePickups(world_state);
        
        // simulate the dropoffs
        simulateDropoffs(world_state);
        
        // simulate the customers (this is currently a placeholder for future expansion)
        simulateCustomers(world_state);
        
        world_state->setCurrentTime( state_.getCurrentTime() );
        
        return amod::SUCCESS;
    }
    
    amod::ReturnCode SimulatorBasic::dispatchVehicle(amod::World *world_state,
                                                     int veh_id,
                                                     const amod::Position &to,
                                                     amod::VehicleStatus start_status,
                                                     amod::VehicleStatus end_status
                                                     ) {
        return dispatchVehicle(world_state, veh_id, to, start_status, end_status, 0);
    }
    
    amod::ReturnCode SimulatorBasic::pickupCustomer(amod::World *world_state,
                                                    int veh_id, int cust_id,
                                                    amod::VehicleStatus start_status,
                                                    amod::VehicleStatus end_status
                                                    ) {
        return pickupCustomer(world_state, veh_id, cust_id, start_status, end_status, 0);
    }
    
    amod::ReturnCode SimulatorBasic::dropoffCustomer(amod::World *world_state,
                                                     int veh_id, int cust_id,
                                                     amod::VehicleStatus start_status,
                                                     amod::VehicleStatus end_status
                                                     ) {
        return dropoffCustomer(world_state, veh_id, cust_id, start_status, end_status, 0);
    }
    
    
    // internal helper functions
    
    amod::ReturnCode SimulatorBasic::dispatchVehicle(amod::World *world_state,
                                                     int veh_id,
                                                     const amod::Position &to,
                                                     amod::VehicleStatus start_status,
                                                     amod::VehicleStatus end_status,
                                                     int booking_id) {
        // check if vehicle already exists in the dispatch map
        auto it = dispatches_.find(veh_id);
        if (it != dispatches_.end()) {
            if (verbose_) std::cout << "Vehicle ID found (it is currently being dispatched!): " << veh_id << std::endl;
            return amod::VEHICLE_CANNOT_BE_DISPATCHED;
        }
        
        // create a new dispatch
        Vehicle veh = state_.getVehicle(veh_id);
        if (!veh.getId()) {
            if (verbose_) std::cout << "Can't get vehicle from world_state" << std::endl;
            return amod::CANNOT_GET_VEHICLE;
        }
        
        Dispatch dp;
        dp.booking_id = booking_id;
        dp.veh_id = veh_id;
        dp.from = veh.getPosition();
        dp.to = to;
        dp.curr = dp.from;
        double dx = to.x - dp.from.x;
        double dy = to.y - dp.from.y;
        double rd = sqrt( dx*dx + dy*dy);
        dp.grad = Position( dx/rd , dy/rd);
        dp.veh_end_status = end_status;
        
        if (dp.from == dp.to) {
            return amod::SOURCE_EQUALS_DESTINATION;
        }
        
        
        // add it to the dispatch qmap
        dispatches_[veh_id] = dp;
        
        // update the vehicle status
        veh.setStatus(start_status);
        world_state->setVehicle(veh);
        
        return amod::SUCCESS;
    }
    
    amod::ReturnCode SimulatorBasic::pickupCustomer(amod::World *world_state,
                                                    int veh_id, int cust_id,
                                                    amod::VehicleStatus start_status,
                                                    amod::VehicleStatus end_status,
                                                    int booking_id) {
        // check that vehicle is at the same location as the customer
        Vehicle veh = state_.getVehicle(veh_id);
        if (!veh.getId()) {
            return amod::CANNOT_GET_VEHICLE;
        }
        
        Customer cust = state_.getCustomer(cust_id);
        if (!cust_id) {
            return amod::CANNOT_GET_CUSTOMER;
        }
        
        if (getDistance(veh.getPosition(), cust.getPosition()) > 1e-3 ) {
            return amod::VEHICLE_NOT_AT_CUSTOMER_LOCATION;
        }
        
        // add a pickup to simulate
        double pickup_time = state_.getCurrentTime() + genRandTruncNormal(pickup_params_);
        if (verbose_) std::cout << "Future Pickup time : " << pickup_time << std::endl;
        Pickup p{booking_id, veh_id, cust_id, pickup_time, end_status};
        pickups_.emplace(pickup_time, p);
        
        // set the vehicle's start status
        veh.setStatus(start_status);
        world_state->setVehicle(veh);
        
        return amod::SUCCESS;
    }
    
    amod::ReturnCode SimulatorBasic::dropoffCustomer(amod::World *world_state,
                                                     int veh_id, int cust_id,
                                                     amod::VehicleStatus start_status,
                                                     amod::VehicleStatus end_status,
                                                     int booking_id) {
        // check that vehicle is at the same location as the customer
        Vehicle veh = world_state->getVehicle(veh_id);
        if (!veh.getId()) {
            return amod::CANNOT_GET_VEHICLE;
        }
        
        if (veh.getCustomerId() != cust_id) {
            return amod::VEHICLE_DOES_NOT_HAVE_CUSTOMER;
        }
        
        Customer cust = state_.getCustomer(cust_id);
        if (!cust_id) {
            return amod::CANNOT_GET_CUSTOMER;
        }
        
        if (getDistance(veh.getPosition(), cust.getPosition()) > 1e-3 ) {
            return amod::VEHICLE_NOT_AT_CUSTOMER_LOCATION;
        }
        
        // add a pickup to simulate
        double dropoff_time = state_.getCurrentTime() + genRandTruncNormal(dropoff_params_);
        //if (verbose_) std::cout << "Future Dropoff time : " << dropoff_time << std::endl;
        Dropoff doff{booking_id, veh_id, cust_id, dropoff_time, end_status};
        dropoffs_.emplace(dropoff_time, doff);
        
        // sets the status of the vehicle
        veh.setStatus(start_status);
        world_state->setVehicle(veh);
        
        // return success
        return amod::SUCCESS;
    }
    
    amod::ReturnCode SimulatorBasic::serviceBooking(amod::World *world_state, const amod::Booking &booking) {
        // add booking to internal structure
        bookings_[booking.id] = booking;
        
        // dispatch the vehicle to the customer's position
        Position from = state_.getVehicle(booking.veh_id).getPosition();
        Position to = state_.getCustomer(booking.cust_id).getPosition();
        
        if (from == to) {
            // from == to, so we just pickup the customer
            return pickupCustomer(world_state, booking.veh_id, booking.cust_id, amod::PICKING_UP, amod::HIRED, booking.id);
        } else {
            // from != to, so we need to move the vehicle
            return dispatchVehicle(world_state, booking.veh_id, to, amod::MOVING_TO_PICKUP, amod::HIRED, booking.id);
        }
    }
    
    double SimulatorBasic::getDrivingDistance(const amod::Position &from, const amod::Position &to) {
        return getDistance(from, to);
    }
    
    double SimulatorBasic::getDistance(const amod::Position &from, const amod::Position &to) {
        return sqrt(pow( from.x - to.x ,2.0) + pow( from.y - to.y ,2.0));
    }
    
    void SimulatorBasic::simulateVehicles(amod::World *world_state) {
        // for all vehicles in dispatches_
        for (auto it = dispatches_.begin(); it != dispatches_.end(); ) {
            // move vehicle
            Vehicle veh = world_state->getVehicle(it->second.veh_id);

            double dist = genRandTruncNormal(speed_params_);
            double x_dist = dist*(it->second.grad.x)*resolution_;
            double y_dist = dist*(it->second.grad.y)*resolution_;
            
            it->second.curr.x += x_dist;
            it->second.curr.y += y_dist;
            
            veh.setPosition(it->second.curr);

            // set associated customer id
            int cust_id = veh.getCustomerId();
            Customer cust = world_state->getCustomer(cust_id);
            if (cust_id) {
                cust.setPosition(it->second.curr);
            }
            
            // if vehicle (specified by the dispatch) has arrived
            if (hasArrived(it->second)) {
                // vehicle has arrived
                if (verbose_) std::cout << veh.getId() << " has arrived at " << it->second.to.x << " " << it->second.to.y << std::endl;
                it->second.curr = it->second.to;
                veh.setPosition(it->second.curr);
                
                // set associated customer position
                if (cust_id) {
                    cust.setPosition(it->second.curr);
                }
                
                // trigger arrival event
                std::vector<int> entities = {veh.getId()};
                
                Event ev(amod::EVENT_ARRIVAL, ++event_id_, "VehicleArrival", state_.getCurrentTime(), entities);
                world_state->addEvent(ev);
                
                // set the vehicle status
                veh.setStatus(it->second.veh_end_status);
                
                // update the world state
                world_state->setVehicle(veh); //update the vehicle in the world state
                state_.setVehicle(veh); // update the internal state
                
                if (cust_id) {
                    world_state->setCustomer(cust);
                    state_.setCustomer(cust);
                }
                
                // if dispatch has non-zero booking id
                if (it->second.booking_id) {
                    // pickup the customer
                    int bid = it->second.booking_id;
                    
                    if (cust.isInVehicle()) {
                        dropoffCustomer(world_state, veh.getId(), cust.getId(), amod::DROPPING_OFF, amod::FREE, bid);
                    } else {
                        pickupCustomer(world_state, bookings_[bid].veh_id, bookings_[bid].cust_id, amod::PICKING_UP, amod::FREE, bid);
                    }
                }
                
                dispatches_.erase(it++);

            } else {
                ++it;
                //if (verbose_) std::cout << "Vehicle pos:" << veh.getPosition().x << " " << veh.getPosition().y << std::endl;
                
                // trigger move event
				std::vector<int> entities = {veh.getId()};
				if (cust_id) entities.push_back(cust_id);
				Event ev(amod::EVENT_MOVE, ++event_id_, "VehicleMoved", state_.getCurrentTime(), entities);
                world_state->addEvent(ev);

                // update the world_state and internal state
                world_state->setVehicle(veh); //update the vehicle in the world state
                state_.setVehicle(veh);
                
                // update the customer position
                if (cust_id) {
                    world_state->setCustomer(cust);
                    state_.setCustomer(cust);
                }
            }
        }
    }
    
    void SimulatorBasic::simulateCustomers(amod::World *world_state) {
        // this is not necessary (at the moment) as performed using by simulateVehicles.
        return;
    }
    
    bool SimulatorBasic::hasArrived(const Dispatch &d) {
        //Position diff(d.to.x - d.curr.x, d.to.y - d.curr.y);
        //return !((sign(diff.x) == sign(d.grad.x)) && (sign(diff.x) == sign(d.grad.x)));
        double dist_to_dest = getDistance(d.from, d.to);
        double dist_to_curr = getDistance(d.from, d.curr);
        return (dist_to_curr >= dist_to_dest);
    }
    
    void SimulatorBasic::simulatePickups(amod::World *world_state) {
        auto it = pickups_.begin();
        while (it != pickups_.end()) {
            if (it->first <= state_.getCurrentTime()) {
                
                if (verbose_) std::cout << it->second.veh_id << " has picked up " << it->second.cust_id << " at time " << it->first << std::endl;
                
                // create pickup event
                std::vector<int> entity_ids = {it->second.veh_id, it->second.cust_id};
                Event ev(amod::EVENT_PICKUP, ++event_id_, "CustomerPickup", it->first, entity_ids);
                world_state->addEvent(ev);
                
                Vehicle veh = world_state->getVehicle(it->second.veh_id);
                veh.setCustomerId(it->second.cust_id);
                Customer cust = world_state->getCustomer(it->second.cust_id);
                cust.setAssignedVehicleId(it->second.veh_id);
                cust.setInVehicle(true);
                
                // sets vehicle state
                veh.setStatus(it->second.veh_end_status);
                
                // update the world state and internal state
                world_state->setCustomer(cust);
                world_state->setVehicle(veh);
                state_.setCustomer(cust);
                state_.setVehicle(veh);
                
                int bid = it->second.booking_id;
                if (bid) {
                    ReturnCode rc = dispatchVehicle(world_state, veh.getId(), bookings_[bid].destination,
                                                    amod::MOVING_TO_DROPOFF, amod::HIRED,
                                                    bid);
                    if (rc != amod::SUCCESS) {
                        if (verbose_) std::cout << kErrorStrings[rc] << std::endl;
                    }
                }
                
                // erase item
                pickups_.erase(it);
                it = pickups_.begin();
            } else {
                break;
            }
        }
    }
    
    void SimulatorBasic::simulateDropoffs(amod::World *world_state) {
        auto it = dropoffs_.begin();
        while (it != dropoffs_.end()) {
            if (it->first <= state_.getCurrentTime()) {
                // create dropoff event
                if (verbose_) std::cout << it->second.veh_id << " has dropped off " << it->second.cust_id << " at time " << it->first << std::endl;

                std::vector<int> entity_ids = {it->second.veh_id, it->second.cust_id};
                Event ev(amod::EVENT_DROPOFF, ++event_id_, "CustomerDropoff", it->first, entity_ids);
                world_state->addEvent(ev);
                
                Vehicle veh = world_state->getVehicle(it->second.veh_id);
                veh.clearCustomerId();
                veh.setStatus(it->second.veh_end_status);
                
                Customer cust = world_state->getCustomer(it->second.cust_id);
                cust.clearAssignedVehicleId();
                cust.setInVehicle(false);
                
                // if is part of a booking, clear it since the vehicle has fropped off the custmer
                int bid = it->second.booking_id;
                if (bid) {
                    bookings_.erase(bid);
                }
                
                // update the external world and internal state
                world_state->setCustomer(cust);
                world_state->setVehicle(veh);
                state_.setCustomer(cust);
                state_.setVehicle(veh);
                
                // erase item
                dropoffs_.erase(it);
                it = dropoffs_.begin();
            } else {
                break;
            }
        }
    }
    
    void SimulatorBasic::setPickupDistributionParams(double mean, double sd, double min, double max) {
        std::normal_distribution<>::param_type par{mean, sd};
        pickup_params_.par = par;
        pickup_params_.max = max;
        pickup_params_.min = min;
    }
    
    void SimulatorBasic::setDropoffDistributionParams(double mean, double sd, double min, double max) {
        std::normal_distribution<>::param_type par{mean, sd};
        dropoff_params_.par = par;
        dropoff_params_.max = max;
        dropoff_params_.min = min;
    }
    
    void SimulatorBasic::setVehicleSpeedParams(double mean, double sd, double min, double max) {

        std::normal_distribution<>::param_type par{mean, sd};
        speed_params_.par = par;
        speed_params_.max = max;
        speed_params_.min = min;
    }
    
    double SimulatorBasic::genRandTruncNormal(TruncatedNormalParams &params) {
        normal_dist.param(params.par);
        double r = normal_dist(eng);
        if (r > params.max) r = params.max;
        if (r < params.min) r = params.min;
        return r;
    }
    
    
} /* namespace amod */
