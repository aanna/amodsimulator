//
//  ManagerBasic.cpp
//  AMODBase
//
//  Created by Harold Soh on 29/3/15.
//  Copyright (c) 2015 Harold Soh. All rights reserved.
//

#include "ManagerBasic.hpp"

namespace amod {
    ManagerBasic::ManagerBasic() {
        return;
    }
    
    ManagerBasic::~ManagerBasic() {
        if (out.is_open()) out.close();
        return;
    }
    
    amod::ReturnCode ManagerBasic::init(World *world_state) {
        out.open("logfile.txt");
        if (!out.is_open()) {
            return amod::FAILED;
        }
        return amod::SUCCESS;
    }
    
    amod::ReturnCode ManagerBasic::update(World *world_state) {
        Simulator *sim = Manager::getSimulator();
        if (!sim) {
            return amod::SIMULATOR_IS_NULLPTR;
        }
        
        // get simulator time
        double current_time = world_state->getCurrentTime();
        
        // get events
        std::vector<Event> events;
        world_state->getEvents(&events);
        
        // respond to events
        for (auto e:events) {
            out << e.t << " Event " << e.id << " " << e.type << " " << e.name << " Entities: ";
            for (auto ent: e.entity_ids) {
                out << ent << ",";
            }
            out << " ";
            
            if (e.type == EVENT_MOVE || e.type == EVENT_ARRIVAL || e.type == EVENT_PICKUP || e.type == EVENT_DROPOFF) {
                amod::Vehicle veh = world_state->getVehicle(e.entity_ids[0]);
                out << veh.getPosition().x << " " << veh.getPosition().y << std::endl;
            }
        }
        // clear events
        world_state->clearEvents();
        
        // dispatch bookings
        auto itr = bookings_.begin();
        bool no_free_vehicles = false;
        while (itr != bookings_.end()) {

            // check if the time is less
            if (itr->first <= current_time) {
                
                // service this booking
                Customer cust = world_state->getCustomer(itr->second.cust_id);
                
                // assign a vehicle to this customer booking
                
                // find closest free vehicle
                // simple iterative method
                double min_dist = -1;
                int best_veh_id = 0;
                std::unordered_map<int, Vehicle>::const_iterator begin_itr, end_itr;
                world_state->getVehicles(&begin_itr, &end_itr);
                for (auto vitr=begin_itr; vitr != end_itr; ++vitr) {
                    double dist = sim->getDistance(vitr->second.getPosition(), cust.getPosition());
                    if (min_dist < 0 || dist < min_dist) {
                        amod::VehicleStatus status = vitr->second.getStatus();
                        if (status == VehicleStatus::PARKED || status == VehicleStatus::FREE ) {
                            min_dist = dist;
                            best_veh_id = vitr->second.getId();
                        }
                    }
                }
                
                if (best_veh_id) {
                    Booking bk = itr->second;
                    bk.veh_id = best_veh_id;
                    
                    // tell the simulator to service this booking
                    amod::ReturnCode rc = sim->serviceBooking(world_state, bk);
                    if (rc!= amod::SUCCESS) {
                        return rc;
                    }
                    
                    // erase the booking
                    bookings_.erase(itr);
                    
                    // set to the earliest booking
                    itr = bookings_.begin();
                } else {
                	// cannot find a proper vehicle, move to next booking
                    itr++;
                    sim_->setCustomerStatus(world_state, itr->second.cust_id,
                    		amod::CustomerStatus::WAITING_FOR_ASSIGNMENT);
                }
                
            } else {
            	// exceeded the current time
                break;
            }

        }
        
        return amod::SUCCESS;
    }
    
    amod::ReturnCode ManagerBasic::loadBookings(const std::vector<Booking> &bookings) {
        for (auto b : bookings) {
            bookings_.emplace( b.booking_time, b);
        }
        return amod::SUCCESS;
    }
    
    amod::ReturnCode ManagerBasic::loadBookingsFromFile(const std::string filename) {
        std::ifstream in(filename.c_str());
        if (!in.good()) {
            std::cout << "Cannot read: " << filename << std::endl;
            return amod::ERROR_READING_BOOKINGS_FILE;
        }
        
        while (in.good()) {
            Booking b;
            in >> b.id >> b.booking_time >> b.cust_id >> b.destination.x >> b.destination.y;
            if (b.id) bookings_.emplace(b.booking_time, b); //only positive booking ids allowed
        }
        
        return amod::SUCCESS;
    }
}
