//
//  ManagerMatchRebalance.cpp
//  AMODBase
//
//  Created by Harold Soh on 29/3/15.
//  Copyright (c) 2015 Harold Soh. All rights reserved.
//

#include "ManagerMatchRebalance.hpp"

namespace amod {
    ManagerMatchRebalance::ManagerMatchRebalance() {

        distance_cost_factor_ = 1;
        waiting_time_cost_factor_ = 1;
        bookings_itr_ = bookings_.begin();
		gurobi_env_ = new GRBEnv();
		matching_interval_ = 60; //every 60 seconds
		next_matching_time_ = matching_interval_;
        return;
    }
    
    ManagerMatchRebalance::~ManagerMatchRebalance() {
        if (out.is_open()) out.close();
        delete gurobi_env_;
        return;
    }
    
    amod::ReturnCode ManagerMatchRebalance::init(World *world_state) {
        out.open("logfile.txt");
        if (!out.is_open()) {
            return amod::FAILED;
        }
        
        // get number of available vehicles
        std::unordered_map<int, Vehicle>::const_iterator begin_itr, end_itr;
        world_state->getVehicles(&begin_itr, &end_itr);
        for (auto vitr=begin_itr; vitr != end_itr; ++vitr) {
            if (vitr->second.getStatus() == VehicleStatus::FREE ||
                vitr->second.getStatus() == VehicleStatus::PARKED) {
                available_vehs_.insert(vitr->second.getId());
            }
        }
        
        next_matching_time_ = world_state->getCurrentTime() + matching_interval_;
        return amod::SUCCESS;
    }
    
    amod::ReturnCode ManagerMatchRebalance::update(World *world_state) {
        Simulator *sim = Manager::getSimulator();
        if (!sim) {
            return amod::SIMULATOR_IS_NULLPTR;
        }
        
        // get simulator time
        double current_time = world_state->getCurrentTime();
        
        // get events
        std::vector<Event> events;
        world_state->getEvents(&events);
        out.precision(10);
        // respond to events
        for (auto e:events) {
            out << e.t << " Event " << e.id << " " << e.type << " " << e.name << " Entities: ";
            for (auto ent: e.entity_ids) {
                out << ent << ",";
            }
            out << " ";
            
            if (e.type == EVENT_MOVE || e.type == EVENT_ARRIVAL ||
            		e.type == EVENT_PICKUP || e.type == EVENT_DROPOFF) {
                amod::Vehicle veh = world_state->getVehicle(e.entity_ids[0]);
                out << veh.getPosition().x << " " << veh.getPosition().y << std::endl;
            }
            
            // increment the available vehicles
            if (e.type == EVENT_DROPOFF) {
                available_vehs_.insert(e.entity_ids[0]);
            }

            // output the location sizes
            if (e.type == EVENT_LOCATION_CUSTS_SIZE_CHANGE ||
            		e.type == EVENT_LOCATION_VEHS_SIZE_CHANGE) {
                amod::Location * ploc = world_state->getLocationPtr(e.entity_ids[0]);
                int curr_size = (e.type == EVENT_LOCATION_VEHS_SIZE_CHANGE)? ploc->getNumVehicles(): ploc->getNumCustomers();
                out << curr_size << " " << ploc->getPosition().x << " " << ploc->getPosition().y << std::endl;
            }

        }
        // clear events
        world_state->clearEvents();
        
        // dispatch bookings by solving the matching problem
        while (bookings_itr_ != bookings_.end()) {
        	// check if the time is less
			if (bookings_itr_->first <= current_time) {
				// ensure that the customer is available (if not, we discard the booking)
				Customer *cust = world_state->getCustomerPtr(bookings_itr_->second.cust_id);
				if (cust->getStatus() == CustomerStatus::FREE ||
					cust->getStatus() == CustomerStatus::WAITING_FOR_ASSIGNMENT) {
					bookings_queue_[bookings_itr_->second.id] = bookings_itr_->second;
				}

	            // erase the booking
	            bookings_.erase(bookings_itr_);

	            // set to the earliest booking
	            bookings_itr_ = bookings_.begin();
			} else {
				break;
			}
        }

        std::cout << world_state->getCurrentTime() << ": Queue Size: " << bookings_queue_.size() << std::endl;

        if (next_matching_time_ >= world_state->getCurrentTime()) {
        	// perform matching and increase next matching time
        	next_matching_time_ = world_state->getCurrentTime() + matching_interval_;
        	return solveMatching(world_state);
        }
        return amod::SUCCESS;
    }
    
    amod::ReturnCode ManagerMatchRebalance::loadBookings(const std::vector<Booking> &bookings) {
        for (auto b : bookings) {
            bookings_.emplace( b.booking_time, b);
        }

        bookings_itr_ = bookings_.begin();

        return amod::SUCCESS;
    }
    
    amod::ReturnCode ManagerMatchRebalance::loadBookingsFromFile(const std::string filename) {
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
        
        bookings_itr_ = bookings_.begin();

        return amod::SUCCESS;
    }
    
    void ManagerMatchRebalance::setCostFactors(double distance_cost_factor, double waiting_time_cost_factor) {
    	distance_cost_factor_ = distance_cost_factor;
    	waiting_time_cost_factor_ = waiting_time_cost_factor;

    }


    void ManagerMatchRebalance::setMatchingInterval(double matching_interval) {
    	matching_interval_ = matching_interval;
    }

    double ManagerMatchRebalance::getMatchingInterval() {
    	return matching_interval_;
    }


    int ManagerMatchRebalance::getNumWaitingCustomers(amod::World *world_state, int loc_id) {
        
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
    

    amod::ReturnCode ManagerMatchRebalance::solveMatching(amod::World *world_state) {

    	if (!world_state) {
    		throw std::runtime_error("solveMatching: world_state is nullptr!");
    	}

    	if (available_vehs_.size() == 0) return amod::SUCCESS; // no vehicles to distribute
    	if (bookings_queue_.size() == 0) return amod::SUCCESS; // no bookings to service

    	// set up the optimization algorithm (with Gurobi)
    	//GRBEnv* env = 0;
    	GRBVar** matching_var = 0;

    	try {
    		// Model
    		GRBModel matching_model = GRBModel(*gurobi_env_);
    		matching_model.set(GRB_StringAttr_ModelName, "matching");


    		// Create decision variables
    		matching_var = new GRBVar* [available_vehs_.size()];

    		int i, j;
    		i = j = 0;
    		std::unordered_map<int, int> index_to_booking_id;
    		std::unordered_map<int, int> index_to_vehicle_id;
    		auto vitr = available_vehs_.begin();
    		for (i=0, vitr = available_vehs_.begin(); vitr != available_vehs_.end(); ++vitr, ++i){
    			index_to_vehicle_id[i] = *vitr;
    			matching_var[i] = matching_model.addVars(bookings_queue_.size(), GRB_BINARY);
    			matching_model.update();

    			// loop through bookings to get bookings that can be served by this vehicle
    			auto bitr = bookings_queue_.begin();
    			for (j=0, bitr = bookings_queue_.begin(); bitr != bookings_queue_.end(); ++bitr, ++j) {
    				// store this index so we can find it easily again
    				index_to_booking_id[j] = bitr->first;

					// get cost
    				Vehicle *veh = world_state->getVehiclePtr(*vitr);
    				Customer *cust = world_state->getCustomerPtr(bitr->second.cust_id);

    				double total_invert_cost = 0;
					double dist_cost = distance_cost_factor_*(sim_->getDrivingDistance(veh->getPosition(), cust->getPosition()));
					if (dist_cost < 0) {
						// this vehicle cannot service this booking
						total_invert_cost = -1.0;
					} else {
						double time_cost = waiting_time_cost_factor_*(std::max(0.0, world_state->getCurrentTime() - bitr->second.booking_time));
						total_invert_cost = 1.0/(1.0 + dist_cost + time_cost);
					}

					// add this variable to the solve
					std::stringstream vname;
					vname << "match" << *vitr << " " << bitr->first; // vehicle to booking
					matching_var[i][j].set(GRB_DoubleAttr_Obj, total_invert_cost);
					matching_var[i][j].set(GRB_StringAttr_VarName, vname.str());

    			}
    		}
    		// Update model to integrate new variables
    		matching_model.update();

    		// Maximize the inverted costs
    		matching_model.set(GRB_IntAttr_ModelSense, GRB_MAXIMIZE); //



    		// Add constraints
			// sum xij over j <= 1, i belongs to vehicles

    		for (i=0; i < available_vehs_.size();++i){
    			GRBLinExpr sum_elems_i = 0;
    			for (j=0; j<bookings_queue_.size();++j) {
    				sum_elems_i += matching_var[i][j];
    			}
    			std::ostringstream cname;
    			cname << "1customer_for_each_vehicle" << i;
    			matching_model.addConstr(sum_elems_i <= 1, cname.str());
    		}

    		// Add constraints
    		// sum xij over i <= 1, j belongs to customers
    		for (j=0; j<bookings_queue_.size();++j){
    			GRBLinExpr sum_elems_j = 0;
    			for (i=0; i < available_vehs_.size();++i) {
    				sum_elems_j += matching_var[i][j];
    			}
    			std::ostringstream cname;
    			cname << "1vehicle_for_each_customer" << j;
    			matching_model.addConstr(sum_elems_j <= 1, cname.str());
    		}

    		matching_model.update();

			// Solve
			matching_model.optimize();


	    	// perform dispatches
			std::cout << "\nOBJECTIVE: " << matching_model.get(GRB_DoubleAttr_ObjVal) << std::endl;
			std::cout << "SOLUTION:" << std::endl;

			for (i=0; i < available_vehs_.size();++i){
				for (j=0; j < bookings_queue_.size();++j) {
					if(matching_var[i][j].get(GRB_DoubleAttr_X) > 0){
						// vehicle is assigned to this booking
						int bid = index_to_booking_id[j];
						int veh_id = index_to_vehicle_id[j];

						bookings_queue_[bid].veh_id = veh_id;
						amod::ReturnCode rc = sim_->serviceBooking(world_state, bookings_queue_[bid]);
						if (rc!= amod::SUCCESS) {
							std::cout << amod::kErrorStrings[rc] << std::endl;
						}

						// mark the car as no longer available
						available_vehs_.erase(veh_id);

						// erase the booking
						bookings_queue_.erase(bid);
					}
				}
			}


    	} catch(GRBException& e) {
    		std::cout << "Error code = " << e.getErrorCode() << std::endl;
    		std::cout << e.getMessage() << std::endl;
    	} catch(std::exception &e) {
    		std::cout << "Exception during optimization" << e.what() << std::endl;
    	}

		// deletes
		// commented out since this is apparently done by gurobi
		for (int i=0; i<available_vehs_.size(); ++i) {
			delete matching_var[i];
		}
		delete [] matching_var;

    	return amod::SUCCESS;
    }



}
