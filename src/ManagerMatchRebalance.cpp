//
//  ManagerMatchRebalance.cpp
//  AMODBase
//
//  Created by Harold Soh on 29/3/15.
//  Copyright (c) 2015 Harold Soh. All rights reserved.
//

#include "ManagerMatchRebalance.hpp"

//#define USE_GUROBI

namespace amod {
    ManagerMatchRebalance::ManagerMatchRebalance() {
        match_method = ASSIGNMENT;
        distance_cost_factor_ = 1;
        waiting_time_cost_factor_ = 1;
        output_move_events_ = true;
        bookings_itr_ = bookings_.begin();
        use_bookings_file_ = false;
        
		matching_interval_ = 5; //every 5 seconds
		next_matching_time_ = matching_interval_;
        event_id_ = 0;
        
        rebalancing_interval_ = 0;
        next_rebalancing_time_ = 0;

        return;
    }
    
    ManagerMatchRebalance::~ManagerMatchRebalance() {
        if (fout_.is_open()) fout_.close();
        

        return;
    }
    
    amod::ReturnCode ManagerMatchRebalance::init(World *world_state) {

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
   
    bool ManagerMatchRebalance::isBookingValid(amod::World *world, const amod::Booking &bk) {
        // checks if there is a path from the source to the destination
        if (sim_->getDrivingDistance(bk.source, bk.destination) < 0) {
            return false;
        }

        // checks that the source is reacheable from all stations
        if (stations_.size() > 0) {
            // assumption is that we are using stations and that vehicles are all 
            // initially located at the stations.
            amod::Customer *cust = world->getCustomerPtr(bk.cust_id);
            bool path_found = false;
            for (auto itr = stations_.begin(); itr !=  stations_.end(); ++itr) {
                auto *l = &(itr->second);

                double dist_cost = -1;
                if (cust->getLocationId()) {
                    dist_cost = sim_->getDrivingDistance(l->getId(), cust->getLocationId());
                } else {
                    dist_cost = sim_->getDrivingDistance(l->getPosition(), cust->getPosition());
                }
                
                if (dist_cost >= 0) {
                    path_found = true;
                    break;
                }
            }        
            
            if (!path_found) return false; //no path from any station to this point. 
        }
        
        
        return true;
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
        if (fout_.is_open()) fout_.precision(10);
        // respond to events
        for (auto e:events) {
            if (e.type == EVENT_ARRIVAL ||  e.type == EVENT_DROPOFF) {
                amod::Vehicle veh = world_state->getVehicle(e.entity_ids[0]);
                
                // make this vehicle available again
                if (veh.getStatus() == VehicleStatus::FREE || veh.getStatus() == VehicleStatus::PARKED) {
                    if (verbose_) std::cout << "Making vehicle " << veh.getId() << " available for redispatch." << std::endl;
                	available_vehs_.insert(e.entity_ids[0]);
                }

            } 
        }

        
        // dispatch bookings by solving the matching problem
        //if (verbose_) std::cout << "Manager Current time: " << current_time << std::endl;
        updateBookingsFromFile(current_time); // load bookings from file (will do nothing if not using file)
        bookings_itr_ = bookings_.begin();
        while (bookings_itr_ != bookings_.end()) {
        	// check if the time is less
			if (bookings_itr_->first <= current_time) {

				if (bookings_itr_->second.id == 0) {
		            // erase the booking
		            bookings_.erase(bookings_itr_);

		            // set to the earliest booking
		            bookings_itr_ = bookings_.begin();
		            continue;
				}
				
				// issue a booking received event
				Event ev(amod::EVENT_BOOKING_RECEIVED, --event_id_, 
                         "BookingReceived", world_state->getCurrentTime(), 
                         {bookings_itr_->second.id, bookings_itr_->second.cust_id});
                world_state->addEvent(ev);

                
                // check that booking is valid 
                if (!isBookingValid(world_state, bookings_itr_->second)) {
                    // issue a booking discarded event
                    Event ev(amod::EVENT_BOOKING_CANNOT_BE_SERVICED, --event_id_, 
                             "BookingDiscarded", world_state->getCurrentTime(), 
                             {bookings_itr_->second.id, NO_SUITABLE_PATH});
                    world_state->addEvent(ev);  
                    
                    // erase and set to earliest booking
                    bookings_.erase(bookings_itr_);
                    bookings_itr_ = bookings_.begin();
                    continue;
                }
                
                
				// ensure that the customer is available (if not, we discard the booking)
				Customer *cust = world_state->getCustomerPtr(bookings_itr_->second.cust_id);
				if (cust->getStatus() == CustomerStatus::FREE ||
					cust->getStatus() == CustomerStatus::WAITING_FOR_ASSIGNMENT) {

                    // check for teleportation
                    if (bookings_itr_->second.travel_mode == amod::Booking::TELEPORT) {
                        sim_->teleportCustomer(world_state, bookings_itr_->second.cust_id, bookings_itr_->second.destination);
                        bookings_.erase(bookings_itr_);
                        
                        // set to the earliest booking
                        bookings_itr_ = bookings_.begin();
                        continue;
                    }
                    
                    // add this to the bookings_queue for matching
					bookings_queue_[bookings_itr_->second.id] = bookings_itr_->second;

					// assign this customer to a station
                    if (stations_.size() > 0) {
                        int st_id = getClosestStationId(cust->getPosition());
                        stations_[st_id].addCustomerId(cust->getId());
                    }
				} else {
                    // issue a booking discarded event
                    Event ev(amod::EVENT_BOOKING_CANNOT_BE_SERVICED, --event_id_, 
                             "BookingDiscarded", world_state->getCurrentTime(), 
                             {bookings_itr_->second.id, CUSTOMER_NOT_FREE});
                    world_state->addEvent(ev);    
                }

	            // erase the booking
	            bookings_.erase(bookings_itr_);

	            // set to the earliest booking
	            bookings_itr_ = bookings_.begin();
			} else {
				break;
			}
        }

        if (next_matching_time_ <= world_state->getCurrentTime()) {
        	// perform matching and increase next matching time
        	next_matching_time_ = world_state->getCurrentTime() + matching_interval_;
            //std::cout << next_matching_time_ << std::endl;
            //if (verbose_) std::cout << world_state->getCurrentTime() << ": Before Queue Size : " << bookings_queue_.size() << std::endl;
        	//if (verbose_) std::cout << world_state->getCurrentTime() << ": Available Vehicles: " << available_vehs_.size() << std::endl;
            amod::ReturnCode rc = amod::FAILED;
            if (match_method == GREEDY) {
                rc = solveMatchingGreedy(world_state);
            } else if (match_method == ASSIGNMENT) {
                rc = solveMatching(world_state);
            } else {
                throw std::runtime_error("No such matching method");
            }
        	//if (verbose_) std::cout << world_state->getCurrentTime() << ": After Queue Size  : " << bookings_queue_.size() << std::endl;
        	//if (verbose_) std::cout << world_state->getCurrentTime() << ": Available Vehicles: " << available_vehs_.size() << std::endl;
            
            // return if we encounter a failure
            if (rc != amod::SUCCESS) {
                return rc;
            }
        }
        
        
        if (next_rebalancing_time_ <= world_state->getCurrentTime()) {
            amod::ReturnCode rc = solveRebalancing(world_state);
            next_rebalancing_time_ = world_state->getCurrentTime() + rebalancing_interval_;
            // return if we encounter a failure
            if (rc != amod::SUCCESS) {
                return rc;
            }
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
        bfin_.open(filename.c_str());
        if (!bfin_.good()) {
            if (verbose_) std::cout << "Cannot read: " << filename << std::endl;
            return amod::ERROR_READING_BOOKINGS_FILE;
        }
        
        use_bookings_file_ = true;
        /*
        for (auto itr = bookings_.begin(); itr != bookings_.end(); itr++) {
            auto &b = itr->second;
            if (verbose_) std::cout << b.id << ": " << b.booking_time << " " << b.cust_id << " " << b.travel_mode << std::endl;
        }
        */
        
        return amod::SUCCESS;
    }
    
    amod::ReturnCode ManagerMatchRebalance::updateBookingsFromFile(double curr_time) {
        if (!use_bookings_file_) return amod::SUCCESS;
        
        if (!bfin_.good()) {
            if (verbose_) std::cout << "Error reading Bookings file!" << std::endl;
            return amod::ERROR_READING_BOOKINGS_FILE;
        }
        
        // add the last thing we read because this wouldn't have been added the last time
        if (last_booking_read_.id != 0 && last_booking_read_.booking_time <= curr_time) {
            bookings_.emplace(last_booking_read_.booking_time, last_booking_read_);
            last_booking_read_ = Booking();
        } else if (last_booking_read_.id != 0) {
            // have not reached the necessary time
            return amod::SUCCESS;
        } 
                
        while (bfin_.good()) {
            Booking b;
            bfin_ >> b.id >> b.booking_time >> b.cust_id >> b.source.x >> b.source.y >> b.destination.x >> b.destination.y >> b.travel_mode;
            //if (verbose_) std::cout << b.id << " " << b.booking_time << std::endl;
            if (b.id && bfin_.good()) {
                if (b.booking_time <= curr_time) {
                    bookings_.emplace(b.booking_time, b); //only positive booking ids allowed
                } else {
                    // don't emplace it just yet
                    last_booking_read_ = b;
                }
            }
            if (b.booking_time > curr_time) {
                break; //assumes bookings file orders bookings by time
            }
        }       
        
        //if (verbose_) std::cout << "Loaded " << bookings_.size() << " Bookings" << 
        //    "; last time: " << last_booking_read_.booking_time << ", " << last_booking_read_.id << std::endl;
        
        
        return amod::SUCCESS;
    }
    
    void ManagerMatchRebalance::setCostFactors(double distance_cost_factor, double waiting_time_cost_factor) {
    	distance_cost_factor_ = distance_cost_factor;
    	waiting_time_cost_factor_ = waiting_time_cost_factor;

    }


    void ManagerMatchRebalance::setMatchingInterval(double matching_interval) {
    	matching_interval_ = matching_interval;
    }

    double ManagerMatchRebalance::getMatchingInterval() const {
    	return matching_interval_;
    }

    void ManagerMatchRebalance::setRebalancingInterval(double rebalancing_interval) {
    	rebalancing_interval_ = rebalancing_interval;
        next_rebalancing_time_ = 0.0;
    }

    double ManagerMatchRebalance::getRebalancingInterval() const {
    	return rebalancing_interval_;
    }

    void ManagerMatchRebalance::loadStations(std::vector<amod::Location> &stations, const amod::World &world_state)
    {
        
        if (stations.size() <= 0) return;
        
    	// create a map for quick lookup based on id
    	for (auto l : stations) {
    		stations_[l.getId()] = l;
    	}

    	// create a tree for quick lookup of location ids
    	stations_tree_.build(stations);

    	// assign vehicles to stations
		std::unordered_map<int, Vehicle>::const_iterator begin_itr, end_itr;
		world_state.getVehicles(&begin_itr, &end_itr);

		for (auto itr = begin_itr; itr != end_itr; ++itr) {
			int st_id = getClosestStationId(itr->second.getPosition());
			stations_[st_id].addVehicleId(itr->second.getId()); // vehicle belongs to this station.
			veh_id_to_station_id_[itr->second.getId()] = st_id;
		}
		return;
    }

    void ManagerMatchRebalance::setDemandEstimator(amod::DemandEstimator *sde) {
    	dem_est_ = sde;
    }


    // **********************************************************
    // PRIVATE FUNCTIONS
    // **********************************************************

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
    


    // use glpk to solve
    amod::ReturnCode ManagerMatchRebalance::solveMatching(amod::World *world_state) {

    	if (!world_state) {
    		throw std::runtime_error("solveMatching: world_state is nullptr!");
    	}

    	if (available_vehs_.size() == 0) return amod::SUCCESS; // no vehicles to distribute
    	if (bookings_queue_.size() == 0) return amod::SUCCESS; // no bookings to service


    	// create variables for solving lp
    	long nbookings = bookings_queue_.size();
    	long nvehs = available_vehs_.size();
    	int nvars = nbookings*nvehs;

        
    	// set up the problem
        glp_prob *lp;
        lp = glp_create_prob();
        glp_set_prob_name(lp, "matching");
        glp_set_obj_dir(lp, GLP_MAX);
        glp_add_cols(lp, nvars);

        // add the structural variables (decision variables)
		std::unordered_map<int, std::pair<int,int>> index_to_ids;

		int k = 1;
		for (auto vitr = available_vehs_.begin(); vitr != available_vehs_.end(); ++vitr){
			// loop through bookings to get bookings that can be served by this vehicle
			for (auto bitr = bookings_queue_.begin(); bitr != bookings_queue_.end(); ++bitr) {
				// store the indices so we can find them again
				index_to_ids[k] = {bitr->first, *vitr};

				// get cost
				Vehicle *veh = world_state->getVehiclePtr(*vitr);
				Customer *cust = world_state->getCustomerPtr(bitr->second.cust_id);

				double total_invert_cost = 0;
                double dist = 0;
                if (veh->getLocationId() && cust->getLocationId()) {
                    dist = sim_->getDrivingDistance(veh->getLocationId(), cust->getLocationId());
                } else {
                    dist = sim_->getDrivingDistance(veh->getPosition(), cust->getPosition());
                }
            
				double dist_cost = distance_cost_factor_*(dist);
				if (dist_cost < 0) {
					// this vehicle cannot service this booking
					total_invert_cost = -1.0;
				} else {
					double time_cost = waiting_time_cost_factor_*(std::max(0.0, world_state->getCurrentTime() - bitr->second.booking_time));
					total_invert_cost = 1.0/(1.0 + dist_cost + time_cost);
				}

				// add this variable to the solver
				std::stringstream ss;
				ss << "x " << bitr->first << " " << *vitr;
				const std::string& tmp = ss.str();
				const char* cstr = tmp.c_str();
				glp_set_col_name(lp, k, cstr);
				//glp_set_col_kind(lp, k, GLP_BV); // use this if you want to run this as a MIP
				glp_set_col_bnds(lp, k, GLP_DB, 0.0, 1.0);
				glp_set_obj_coef(lp, k, total_invert_cost);

				// increment index
				++k;
			}
		}
		// setup the constraints
		k = 1;
		int ncons = nbookings + nvehs;
		int nelems = nbookings*nvehs*2;
        int *ia = new int[nelems + 1];
        int  *ja = new int[nelems + 1]; // +1 because glpk starts indexing at 1 (why? I don't know)
        double *ar = new double[nelems + 1];


		glp_add_rows(lp, ncons);
		for (int i=1; i<=nvehs; ++i) {
			std::stringstream ss;
			ss << "veh " << i;
			const std::string& tmp = ss.str();
			const char* cstr = tmp.c_str();
			glp_set_row_name(lp, i, cstr);
			glp_set_row_bnds(lp, i, GLP_DB, 0.0, 1.0);

			for (int j=1; j<=nbookings; ++j) {
				ia[k] = i;
				ja[k] = (i-1)*nbookings + j;
				ar[k] = 1.0;
				++k;
			}
		}

		for (int j=1; j<=nbookings; ++j) {
			std::stringstream ss;
			ss << "booking " << j;
			const std::string& tmp = ss.str();
			const char* cstr = tmp.c_str();
			glp_set_row_name(lp, nvehs+j, cstr);
			glp_set_row_bnds(lp, nvehs+j, GLP_DB, 0.0, 1.0);

			for (int i=1; i<=nvehs; ++i) {
				ia[k] = nvehs+j;
				ja[k] = (i-1)*nbookings + j;
				ar[k] = 1.0;
				++k;
			}
		}

		// if (verbose_) std::cout << k << " " << ncons << std::endl;

		// load the matrix
		// if (verbose_) std::cout << "Loading the matrix" << std::endl;
		glp_load_matrix(lp, k-1, ia, ja, ar);

		// solve the problem
		// if (verbose_) std::cout << "Solving the problem" << std::endl;

		// integer program
		/*
		glp_iocp parm;
		glp_init_iocp(&parm);
		parm.presolve = GLP_ON;
		glp_intopt(lp, &parm);
		*/
		// linear program
        if (!verbose_) glp_term_out(GLP_OFF); // suppress terminal output
		glp_simplex(lp, nullptr);

		// print out the objective value
		//double z = glp_mip_obj_val(lp);
		//if (verbose_) std::cout << z << std::endl;

		// dispatch the vehicles
		for (int k=1; k<=nvars; ++k) {
			//int opt_x = glp_mip_col_val(lp, k); //to get result for integer program
			int opt_x = glp_get_col_prim(lp,k);
			if(opt_x > 0){

				// vehicle is assigned to this booking
				auto ids = index_to_ids[k];
				int bid = ids.first;
				int veh_id = ids.second;

				bookings_queue_[bid].veh_id = veh_id;
				amod::ReturnCode rc = sim_->serviceBooking(world_state, bookings_queue_[bid]);
				if (rc!= amod::SUCCESS) {
					if (verbose_) std::cout << amod::kErrorStrings[rc] << std::endl;
                    Event ev(amod::EVENT_BOOKING_CANNOT_BE_SERVICED, --event_id_, "BookingDiscarded", world_state->getCurrentTime(), {bid, SERVICE_BOOKING_FAILURE});
                    world_state->addEvent(ev);   
				} else {
					if (verbose_) std::cout << "Assigned " << veh_id << " to booking " << bid << std::endl;
					// mark the car as no longer available
					available_vehs_.erase(veh_id);

					// change station ownership of vehicle
                    if (stations_.size() > 0) {
                        int st_id = veh_id_to_station_id_[veh_id]; //old station
                        stations_[st_id].removeVehicleId(veh_id);
                        int new_st_id = getClosestStationId( bookings_queue_[bid].destination ); //the station at the destination
                        stations_[new_st_id].addVehicleId(veh_id);
                        veh_id_to_station_id_[veh_id] = new_st_id;

                        // remove this customer from the station queue
                        stations_[st_id].removeCustomerId(bookings_queue_[bid].cust_id);
                    }

                    // issue a booking serviced event
                    Event ev(amod::EVENT_BOOKING_SERVICED, --event_id_, "BookingServiced", world_state->getCurrentTime(), {bid});
                    world_state->addEvent(ev);   
                    
				}

				// erase the booking
				bookings_queue_.erase(bid);
			}
		}

		//double x1 = glp_mip_col_val(mip, 1);
        // housekeeping; clear up all the dynamically allocated memory
        glp_delete_prob(lp);
        glp_free_env();
        
        delete [] ia;
        delete [] ja;
        delete [] ar;
        
        return amod::SUCCESS;
    }
    
    amod::ReturnCode ManagerMatchRebalance::solveMatchingGreedy(amod::World *world_state) {
        
        if (!world_state) {
            throw std::runtime_error("solveMatching: world_state is nullptr!");
        }
        
        if (available_vehs_.size() == 0) return amod::SUCCESS; // no vehicles to distribute
        if (bookings_queue_.size() == 0) return amod::SUCCESS; // no bookings to service
        
        
        // create variables for solving lp
        long nbookings = bookings_queue_.size();
        long nvehs = available_vehs_.size();
        
        // for each booking, find closest vehicle
        std::vector<int> to_erase;
        for (auto bitr = bookings_queue_.begin(); bitr != bookings_queue_.end(); ++bitr) {
            double min_dist_cost = std::numeric_limits<int>::max();
            Vehicle *closest_veh = nullptr;
            Location *closest_loc = nullptr;
            Customer *cust = world_state->getCustomerPtr(bitr->second.cust_id);
            
            // either go through available vehicles or node locations (whichever is smaller)
            if (available_vehs_.size() < world_state->getNumLocations()) {
                //if (verbose_) std::cout << "Looping through vehicles" << std::endl;
                for (auto vitr = available_vehs_.begin(); vitr != available_vehs_.end(); ++vitr){
                    // get cost
                    Vehicle *veh = world_state->getVehiclePtr(*vitr);
                    double dist_cost = -1;
                    if (veh->getLocationId() && cust->getLocationId()) {
                        dist_cost = sim_->getDrivingDistance(veh->getLocationId(), cust->getLocationId());
                    } else {
                        dist_cost = sim_->getDrivingDistance(veh->getPosition(), cust->getPosition());
                    }
                    if (dist_cost >= 0 && min_dist_cost > dist_cost) {
                        closest_veh = veh;
                        min_dist_cost = dist_cost;
                    }
                }
            } else { 
                //if (verbose_) std::cout << "Looping through locations" << std::endl;
                // check for other locations
                std::unordered_map<int, Location>::const_iterator lbitr, leitr;
                world_state->getLocations(&lbitr, &leitr); 
                for (auto itr = lbitr; itr != leitr; ++itr) {
                    auto *l = &(itr->second);
                    if (l->getNumVehicles() > 0) {
                        
                        double dist_cost = -1;
                        if (cust->getLocationId()) {
                            dist_cost = sim_->getDrivingDistance(l->getId(), cust->getLocationId());
                        } else {
                            dist_cost = sim_->getDrivingDistance(l->getPosition(), cust->getPosition());
                        }
                            
                        if (dist_cost >=0 && min_dist_cost > dist_cost) {
                            closest_loc = world_state->getLocationPtr(l->getId());
                            min_dist_cost = dist_cost;
                        }                   
                    }
                }
                
                if (closest_loc != nullptr) {
                    //get a vehicle
                    std::unordered_set<int>::const_iterator vbitr, veitr;
                    closest_loc->getVehicleIds(&vbitr, &veitr);
                    if (vbitr != veitr) {
                        closest_veh = world_state->getVehiclePtr(*vbitr);
//                         if (verbose_) {
//                             std::cout << closest_veh->getPosition().x << " " << closest_veh->getPosition().y << " : " <<
//                             closest_loc->getPosition().x << " " << closest_loc->getPosition().y << std::endl;
//                         }
                    }
                }
            }
            if (closest_veh != nullptr) {
                // assign vehicle to booking
                int veh_id = closest_veh->getId();
                int bid = bitr->second.id;
                bookings_queue_[bid].veh_id = veh_id;
                amod::ReturnCode rc = sim_->serviceBooking(world_state, bookings_queue_[bid]);
                if (rc!= amod::SUCCESS) {
                    if (verbose_) std::cout << amod::kErrorStrings[rc] << std::endl;
                } else {
                    if (verbose_) std::cout << "Assigned " << veh_id << " to booking " << bid << std::endl;
                    // mark the car as no longer available
                    available_vehs_.erase(veh_id);
                    
                    // change station ownership of vehicle
                    if (stations_.size() > 0) {
                        int st_id = veh_id_to_station_id_[veh_id]; //old station
                        stations_[st_id].removeVehicleId(veh_id);
                        int new_st_id = getClosestStationId( bookings_queue_[bid].destination ); //the station at the destination
                        stations_[new_st_id].addVehicleId(veh_id);
                        veh_id_to_station_id_[veh_id] = new_st_id;
                        
                        // remove this customer from the station queue
                        stations_[st_id].removeCustomerId(bookings_queue_[bid].cust_id);
                    }
                    
                }
                
                // issue a booking serviced event
                Event ev(amod::EVENT_BOOKING_SERVICED, --event_id_, "BookingServiced", world_state->getCurrentTime(), {bid});
                world_state->addEvent(ev);   
                
                // mark booking to be erased
                to_erase.emplace_back(bid);
            }
            
        }
        
        if (verbose_) std::cout << "Before: " << bookings_queue_.size() << " ";
        
        
        for (auto itr=to_erase.begin(); itr!= to_erase.end(); ++itr) {
            bookings_queue_.erase(*itr);
        }
        
        if (verbose_) std::cout << "After: " << bookings_queue_.size() << std::endl;
        
        return amod::SUCCESS;
    }

    amod::ReturnCode ManagerMatchRebalance::solveRebalancing(amod::World *world_state) {
    	if (!world_state) {
    		throw std::runtime_error("solveMatching: world_state is nullptr!");
    	}

    	if (available_vehs_.size() == 0) {
            if (verbose_) std::cout << "No available vehicles to rebalance." << std::endl;
            return amod::SUCCESS; // no vehicles to rebalance
        }
        if (stations_.size() == 0) {
            if (verbose_) std::cout << "No stations loaded." << std::endl;
            return amod::SUCCESS; // nothing to rebalance
        }
    	// create variables for solving lp
    	int nvehs = available_vehs_.size();
    	int nstations = stations_.size();
        int nstations_underserved = 0;
    	int nvars = nstations*nstations; // how many to send from one station to another
        
        int vi_total = available_vehs_.size();
        int cex_total = 0;
        std::unordered_map<int, int> cex;
        std::unordered_map<int, std::set<int>> vi; // free vehicles at this station

    	// set up the problem
        glp_prob *lp;
        lp = glp_create_prob();
        glp_set_prob_name(lp, "rebalancing");
        glp_set_obj_dir(lp, GLP_MIN);
        glp_add_cols(lp, nvars);


        // add the structural variables (decision variables)
		std::unordered_map<int, std::pair<int,int>> index_to_ids;
        std::unordered_map<std::pair<int,int>, int> ids_to_index;

		int k = 1;
		for (auto sitr = stations_.begin(); sitr != stations_.end(); ++sitr){
			for (auto sitr2 = stations_.begin(); sitr2 != stations_.end(); ++sitr2) {

				// store the indices to make lookups easier (can be optimized in future iterations)
				index_to_ids[k] = {sitr->first, sitr2->first};
                ids_to_index[std::make_pair(sitr->first, sitr2->first)] = k;

				// get cost
				double cost = sim_->getDrivingDistance(sitr->second.getPosition(),
						sitr2->second.getPosition() );
                
                if (cost == -1) {
                    // no route possible
                    cost = 1e11; //some large number
                };

				// add this variable to the solver
				std::stringstream ss;
				ss << "x " << sitr->first << " " << sitr2->first;
				const std::string& tmp = ss.str();
				const char* cstr = tmp.c_str();
				glp_set_col_name(lp, k, cstr);

				glp_set_col_bnds(lp, k, GLP_LO, 0.0, 0.0); // set lower bound of zero, no upperbound
				glp_set_obj_coef(lp, k, cost);

				// increment index
				++k;
			}
            
            // compute variables for the lp

			// use current demand
			// int cexi = sitr->second.getNumCustomers() - sitr->second.getNumVehicles();

			// use predicted demand
			int stid =  sitr->second.getId();
            auto curr_time = world_state->getCurrentTime();
            auto pred = dem_est_->predict(stid, *world_state, curr_time);
			int mean_pred = ceil(pred.first);
			/*int mean_pred = ceil(std::max(
					(double) dem_est_->predict(sitr->second.getId(), *world_state, world_state->getCurrentTime()).first,
					(double) sitr->second.getNumCustomers()));
			*/
            if (verbose_) std::cout << "Mean prediction: " << mean_pred;
			int cexi = mean_pred - sitr->second.getNumVehicles();
            if (verbose_) std::cout << "cexi: " << cexi;
            if (verbose_) std::cout << "vehs: " << sitr->second.getNumVehicles();
            
			cex[sitr->first] = cexi; // excess customers at this station
            cex_total += cexi; // total number of excess customers

            if (cexi > 0) {
                nstations_underserved++;
            }
            
            if (verbose_) std::cout << "cex[" << sitr->first << "]: " << cex[sitr->first] << std::endl;
            
		}
        

        // set up available vehicles at each station
        for (auto vitr = available_vehs_.begin(); vitr != available_vehs_.end(); ++vitr) {
            // get which station this vehicle belongs
            int sid = veh_id_to_station_id_[*vitr];
            vi[sid].insert(*vitr);
        }
        

		// set up constraints
        int *ia;
        int  *ja; // +1 because glpk starts indexing at 1 (why? I don't know)
        double *ar;
        if (cex_total <= 0) {
            // should be possible to satisfy all customers by rebalancing
            int ncons = nstations*2;
            int nelems = nstations*((nstations - 1)*2) + nstations*(nstations-1);
            ia = new int[nelems + 1];
            ja = new int[nelems + 1]; // +1 because glpk starts indexing at 1 (why? I don't know)
            ar = new double[nelems + 1];
            
            glp_add_rows(lp, ncons);
            int k = 1;
            int i = 1;
            
            // constraint for net flow to match (or exceed) excess customers
            for (auto sitr = stations_.begin(); sitr!= stations_.end(); ++sitr) {
                std::stringstream ss;
                ss << "st " << sitr->second.getId();
                const std::string& tmp = ss.str();
                const char* cstr = tmp.c_str();
                glp_set_row_name(lp, i, cstr);
                glp_set_row_bnds(lp, i, GLP_LO, cex[sitr->second.getId()], 0.0);
                
               
                for (auto sitr2 = stations_.begin(); sitr2 != stations_.end(); ++sitr2) {
                    if (sitr2->first == sitr->first) continue;
                    // from i to j
                    ia[k] = i;
                    int st_source = sitr->second.getId();
                    int st_dest   = sitr2->second.getId();
                    ja[k] = ids_to_index[{st_source, st_dest}];
                    ar[k] = -1.0;
                    ++k;
                    
                    // from j to i
                    ia[k] = i;
                    ja[k] = ids_to_index[{st_dest, st_source}];
                    ar[k] = 1.0;
                    ++k;
                }
                ++i; // increment i
            }
            
            // constraint to make sure stations don't send more vehicles than they have
            for (auto sitr = stations_.begin(); sitr!= stations_.end(); ++sitr) {
                std::stringstream ss;
                ss << "st " << sitr->second.getId() << " veh constraint";
                const std::string& tmp = ss.str();
                const char* cstr = tmp.c_str();
                glp_set_row_name(lp, i, cstr);
                //if (verbose_) std::cout << "vi[" << sitr->first << "]: " <<  vi[sitr->second.getId()].size() << std::endl;
                glp_set_row_bnds(lp, i, GLP_UP, 0.0, vi[sitr->second.getId()].size());
                
                for (auto sitr2 = stations_.begin(); sitr2 != stations_.end(); ++sitr2) {
                    if (sitr2->first == sitr->first) continue;
                    
                    // from i to j
                    ia[k] = i;
                    int st_source = sitr->second.getId();
                    int st_dest   = sitr2->second.getId();
                    ja[k] = ids_to_index[{st_source, st_dest}];
                    ar[k] = 1.0;
                    ++k;
                }
                ++i; // increment i
            }
            
            glp_load_matrix(lp, nelems, ia, ja, ar);
            
        } else {
            // cannot satisfy all customers, rebalance to obtain even distribution
            
            // should be possible to satisfy all customers by rebalancing
            int ncons = nstations*3;
            int nelems = nstations*((nstations-1)*2) + 2*nstations*(nstations-1) ;
            ia = new int[nelems + 1];
            ja = new int[nelems + 1]; // +1 because glpk starts indexing at 1 (why? I don't know)
            ar = new double[nelems + 1];
            
            glp_add_rows(lp, ncons);
            int k = 1;
            int i = 1;
            
            // if (verbose_) std::cout << "Even distribution: " <<  floor(vi_total/nstations_underserved) << std::endl;
            // constraint for net flow to match (or exceed) excess customers
            for (auto sitr = stations_.begin(); sitr!= stations_.end(); ++sitr) {
                std::stringstream ss;
                ss << "st " << sitr->second.getId();
                const std::string& tmp = ss.str();
                const char* cstr = tmp.c_str();
                glp_set_row_name(lp, i, cstr);
                glp_set_row_bnds(lp, i, GLP_LO,
                                 std::min((double) cex[sitr->second.getId()] ,
                                          (double) floor(vi_total/nstations_underserved)), 0.0);
                
                
                for (auto sitr2 = stations_.begin(); sitr2 != stations_.end(); ++sitr2) {
                    if (sitr2->first == sitr->first) continue;

                    // from i to j
                    ia[k] = i;
                    int st_source = sitr->second.getId();
                    int st_dest   = sitr2->second.getId();
                    ja[k] = ids_to_index[{st_source, st_dest}];
                    ar[k] =  -1.0;
                    ++k;
                    
                    // from j to i
                    ia[k] = i;
                    ja[k] = ids_to_index[{st_dest, st_source}];
                    ar[k] =  1.0;
                    ++k;
                }
                ++i; // increment i
            }
            
            // constraint to make sure stations don't send more vehicles than they have
            for (auto sitr = stations_.begin(); sitr!= stations_.end(); ++sitr) {
                std::stringstream ss;
                ss << "st " << sitr->second.getId() << " veh constraint";
                const std::string& tmp = ss.str();
                const char* cstr = tmp.c_str();
                glp_set_row_name(lp, i, cstr);
                glp_set_row_bnds(lp, i, GLP_UP, 0.0, vi[sitr->second.getId()].size());
                
                for (auto sitr2 = stations_.begin(); sitr2 != stations_.end(); ++sitr2) {
                    // from i to j
                    if (sitr2->first == sitr->first) continue;

                    ia[k] = i;
                    int st_source = sitr->second.getId();
                    int st_dest   = sitr2->second.getId();
                    ja[k] = ids_to_index[{st_source, st_dest}];
                    ar[k] =  1.0;
                    ++k;
                }
                ++i; // increment i
            }
            
            // constraint for stations to send as many vehicles as possible
            for (auto sitr = stations_.begin(); sitr!= stations_.end(); ++sitr) {
                std::stringstream ss;
                ss << "st " << sitr->second.getId() << " send all constraint";
                const std::string& tmp = ss.str();
                const char* cstr = tmp.c_str();
                glp_set_row_name(lp, i, cstr);
                double constr = std::min( (double) vi[sitr->first].size(), (double) std::max(0, -cex[sitr->first] ));
                glp_set_row_bnds(lp, i, GLP_LO, constr, 0.0);
                
                for (auto sitr2 = stations_.begin(); sitr2 != stations_.end(); ++sitr2) {
                    if (sitr2->first == sitr->first) continue;

                    // from i to j
                    ia[k] = i;
                    int st_source = sitr->second.getId();
                    int st_dest   = sitr2->second.getId();
                    ja[k] = ids_to_index[{st_source, st_dest}];
                    ar[k] =  1.0;
                    ++k;
                }
                ++i; // increment i
            }
            
            glp_load_matrix(lp, nelems, ia, ja, ar);
        }
        
        // solve the lp
        if (!verbose_) glp_term_out(GLP_OFF); // suppress terminal output
        glp_simplex(lp, nullptr);
        
        
        // redispatch based on lp solution
        for (int k=1; k<=nvars; k++) {
            // get the value
            int to_dispatch = floor(glp_get_col_prim(lp,k));
            //if (verbose_) std::cout << k << ": " << to_dispatch << std::endl;
            if (to_dispatch > 0) {
                int st_source = index_to_ids[k].first;
                int st_dest = index_to_ids[k].second;
                
                // dispatch to_dispatch vehicles form station st_source to st_dest
                amod::ReturnCode rc = interStationDispatch(st_source, st_dest, to_dispatch, world_state, vi);
                
                Event ev(amod::EVENT_REBALANCE, --event_id_, 
                      "Rebalancing", world_state->getCurrentTime(), 
                       {st_source, st_dest, to_dispatch});
                world_state->addEvent(ev);
                
                if (rc != amod::SUCCESS) {
                    if (verbose_) std::cout << amod::kErrorStrings[rc] << std::endl;
                    
                    // housekeeping 
                    glp_delete_prob(lp);
                    glp_free_env();
                    
                    delete [] ia;
                    delete [] ja;
                    delete [] ar;

                    
                    // be stringent and throw an exception: this shouldn't happen
                    throw std::runtime_error("solveRebalancing: interStationDispatch failed.");
                }
            }
        }
        
        // housekeeping
        glp_delete_prob(lp);
        glp_free_env();
        
        delete [] ia;
        delete [] ja;
        delete [] ar;

    	return amod::SUCCESS;
    }


    amod::ReturnCode ManagerMatchRebalance::interStationDispatch(int st_source, int st_dest,
                                                                 int to_dispatch,
                                                                 amod::World *world_state,
                                                                 std::unordered_map<int, std::set<int>> &vi) {
        // check that to_dispatch is positive
        if (to_dispatch <= 0) return amod::FAILED;
        
        // check that st_source and st_dest are valid
        auto itr_source = stations_.find(st_source);
        auto itr_dest = stations_.find(st_dest);
        
        if (itr_source == stations_.end() || itr_dest == stations_.end() ) {
            return amod::INVALID_STATION_ID;
        }
        
        // dispatch vehicles
        auto itr = vi[st_source].begin();
        for (int i=0; i<to_dispatch; i++) {
            // find a free vehicle at station st_source
            int veh_id = *itr;
            
            // send it to station st_dest
            if (verbose_) std::cout << "Rebalancing " << veh_id << " from " << st_source << " to " << st_dest << std::endl;
            auto rc = sim_->dispatchVehicle(world_state, veh_id , itr_dest->second.getPosition(),
                                            VehicleStatus::MOVING_TO_REBALANCE, VehicleStatus::FREE);
            
            // if dispatch is success
            if (rc != amod::SUCCESS) {
                if (verbose_) std::cout << kErrorStrings[rc] << std::endl;
            }; // return with error code
            
            // change station ownership of vehicle
            stations_[st_source].removeVehicleId(veh_id);
            stations_[st_dest].addVehicleId(veh_id);
            veh_id_to_station_id_[veh_id] = st_dest;
            available_vehs_.erase(veh_id);
            
            // mark vehicle as no longer available for dispatch
            vi[st_source].erase(veh_id);
            
            // increment iterator
            itr = vi[st_source].begin();
        }
        
        return amod::SUCCESS;
    }
    

    int ManagerMatchRebalance::getClosestStationId(const amod::Position &pos) const {
    	return stations_tree_.findNN({pos.x, pos.y}).getId();
    }


}
