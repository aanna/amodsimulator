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
        if (fout_.is_open()) fout_.close();
        return;
    }
    
    amod::ReturnCode ManagerBasic::init(World *world_state) {
        
        // get number of available vehicles
        num_avail_veh_ = 0;
        std::unordered_map<int, Vehicle>::const_iterator begin_itr, end_itr;
        world_state->getVehicles(&begin_itr, &end_itr);
        for (auto vitr=begin_itr; vitr != end_itr; ++vitr) {
            if (vitr->second.getStatus() == VehicleStatus::FREE ||
                vitr->second.getStatus() == VehicleStatus::PARKED) {
                ++num_avail_veh_;
            }
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
        // std::clock_t start = std::clock();
        // double duration = 0;

        std::vector<Event> events;
        world_state->getEvents(&events);
        if (fout_.is_open()) fout_.precision(10);
        // respond to events
        for (auto e:events) {

            if (e.type == EVENT_DROPOFF) {
                ++num_avail_veh_;
            }

        }
        // clear events
        world_state->clearEvents();

        // dispatch bookings
        auto itr = bookings_.begin();
        int num_skipped = 0;
        int num_processed = 0;
        std::list<std::list<Booking>::iterator> to_erase;
        while (itr != bookings_.end()) {

        	Booking bk = *itr;

            // check if the time is less
            if (bk.booking_time <= current_time) {
            	++num_processed;
                // Get the relevant customer
                Customer cust = world_state->getCustomer(bk.cust_id);
                if (!cust.getId()) {
                    // invalid customer id
                    // delete this booking
                    std::cout << "Invalid customer id. Deleting booking " << bk.id << std::endl;
                    to_erase.emplace_back(itr);
                    ++itr;
                    continue;
                }
                
                // check that customer is free
                if (!(cust.getStatus() == CustomerStatus::FREE ||
                      cust.getStatus() == CustomerStatus::WAITING_FOR_ASSIGNMENT)) {
                    // customer is not free or not waiting for assignment
                    // we skip this booking
                    //std::cout << "Booking type: " << itr->second.travel_mode << std::endl;
                    //std::cout << "Cust " << cust.getId() << " is not free, status " << cust.getStatus() << std::endl;
                    ++itr;
                    ++num_skipped;
                    continue;
                } else {
                    //std::cout << "Cust " << cust.getId() << " is free. Proceeding to assign. ";
                }

                // check for teleportation
                if (bk.travel_mode == amod::Booking::TELEPORT) {
                    sim_->teleportCustomer(world_state, bk.cust_id, bk.destination);
                    to_erase.emplace_back(itr);
                    ++itr;
                    continue;
                }
                


                // check if we have vehicles to dispatch
                //std::cout << world_state->getCurrentTime() << ": Num Available Veh: " << num_avail_veh_ << std::endl;
                if (num_avail_veh_ == 0) {
                    sim_->setCustomerStatus(world_state, bk.cust_id,
                                            amod::CustomerStatus::WAITING_FOR_ASSIGNMENT);
                    itr++;
                    continue;
                }
                
                
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
                    bk.veh_id = best_veh_id;
                    
                    // tell the simulator to service this booking
                    //std::cout << "... Assign bk " << bk.id << " car " << best_veh_id << std::endl;
                    amod::ReturnCode rc = sim->serviceBooking(world_state, bk);
                    if (rc!= amod::SUCCESS) {
                        // destroy booking and print error code
                        std::cout << amod::kErrorStrings[rc] << std::endl;
                    } else {
                        --num_avail_veh_;
                    }
                    
                    // erase the booking
                    to_erase.emplace_back(itr);
                    ++itr;
                } else {
                    // cannot find a proper vehicle, move to next booking
                    //std::cout << "... no car found " << std::endl;
                    sim_->setCustomerStatus(world_state, bk.cust_id,
                                            amod::CustomerStatus::WAITING_FOR_ASSIGNMENT);
                    ++itr;
                }



            } else {
                // exceeded the current time
                break;
            }
            
        }

        // erase the bookings that were serviced/removed
        for (auto vitr=to_erase.begin(); vitr!=to_erase.end(); ++vitr) {
        	bookings_.erase(*vitr);
        }

//        duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
//        std::cout<<"Booking processing (" << num_processed <<   ") : "<< duration <<'\n';
//        std::cout<<" processing : "<< duration << ",   ";

        // output
        if (int num_waiting_cust = getNumWaitingCustomers(world_state)) {
            std::cout << world_state->getCurrentTime() << ": " << "Queue Size: " << num_waiting_cust <<
            		" Num Skipped: " << num_skipped << std::endl;
        }
        return amod::SUCCESS;
    }
    
    amod::ReturnCode ManagerBasic::loadBookings(const std::vector<Booking> &bookings) {
        for (auto b : bookings) {
            bookings_.emplace_back(b);
        }

        bookings_.sort();

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
            in >> b.id >> b.booking_time >> b.cust_id >> b.source.x >> b.source.y >> b.destination.x >> b.destination.y >> b.travel_mode;
            if (b.id && in.good()) bookings_.emplace_back( b); //only positive booking ids allowed
        }
        /*
         for (auto itr = bookings_.begin(); itr != bookings_.end(); itr++) {
         auto &b = itr->second;
         std::cout << b.id << ": " << b.booking_time << " " << b.cust_id << " " << b.travel_mode << std::endl;
         }
         */
        bookings_.sort();
        return amod::SUCCESS;
    }
    
    
    int ManagerBasic::getNumWaitingCustomers(amod::World *world_state, int loc_id) {
        
        if (!world_state) return 0;
        
        if (loc_id) { //loc_id > 0?
            int num_cust = 0;
            // get iterators
            std::unordered_set<int>::const_iterator bitr, eitr;
            Location *ploc = world_state->getLocationPtr(loc_id);
            if (ploc) {
                ploc->getCustomerIds(&bitr, &eitr);
            } else {
                // incorrect loc_id
                return 0;
            }
            
            // loop through all customers at location specific by loc_id
            for (auto itr = bitr; itr != eitr; ++itr) {
                int cust_id = *itr; // get customer id
                // get customer pointer and check status
                Customer *cust = world_state->getCustomerPtr(cust_id);
                if (cust) {
                    if (cust->getStatus() == CustomerStatus::WAITING_FOR_ASSIGNMENT) {
                        num_cust++;
                    }
                } else {
                    throw std::runtime_error("Customer in location does not exist!");
                }
            }
            
            // return number of waiting customers
            return num_cust;
        }
        
        // if loc_id == 0, then we want all customers from all locations.
        // get waiting customers from all locations
        // get iterators for Locations
        std::unordered_map<int, Location>::const_iterator bitr, eitr;
        world_state->getLocations(&bitr, &eitr);
        int num_cust = 0;
        for (auto itr=bitr; itr!=eitr; itr++) {
            num_cust += getNumWaitingCustomers(world_state, itr->second.getId());
        }
        return num_cust;
    }
    
}
