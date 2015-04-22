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

        distance_cost_factor_ = 1;
        waiting_time_cost_factor_ = 1;
        output_move_events_ = true;
        bookings_itr_ = bookings_.begin();
    
		matching_interval_ = 60; //every 60 seconds
		next_matching_time_ = matching_interval_;
        
        #ifdef USE_GUROBI
        gurobi_env_ = new GRBEnv();
        #else
        
        #endif
        return;
    }
    
    ManagerMatchRebalance::~ManagerMatchRebalance() {
        if (fout_.is_open()) fout_.close();
        
        #ifdef USE_GUROBI
        delete gurobi_env_;
        #else
        
        #endif

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
    
    amod::ReturnCode ManagerMatchRebalance::setOutputFile(std::string filename, bool output_move_events) {
    	fout_.open(filename.c_str());
        if (!fout_.is_open()) {
            return amod::FAILED;
        }
        output_move_events_ = output_move_events;
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
        if (fout_.is_open()) fout_.precision(10);
        // respond to events
        for (auto e:events) {
            

        	if (fout_.is_open()) {
                if ((output_move_events_ && e.type == EVENT_MOVE) || (e.type != EVENT_MOVE)) {
                    fout_ << e.t << " Event " << e.id << " " << e.type << " " << e.name << " Entities: ";
                    for (auto ent: e.entity_ids) {
                        fout_ << ent << ",";
                    }
                    fout_ << " ";
                }
        	}
            
            if (e.type == EVENT_MOVE || e.type == EVENT_ARRIVAL || e.type == EVENT_PICKUP || e.type == EVENT_DROPOFF) {
                amod::Vehicle veh = world_state->getVehicle(e.entity_ids[0]);
                
                
                if (fout_.is_open()) {
                    if ((output_move_events_ && e.type == EVENT_MOVE) || (e.type != EVENT_MOVE)) {
                        fout_ << veh.getPosition().x << " " << veh.getPosition().y;
                    }
                }

                // make this vehicle available again
                if (veh.getStatus() == VehicleStatus::FREE || veh.getStatus() == VehicleStatus::PARKED) {
                	available_vehs_.insert(e.entity_ids[0]);
                }

            }
            
            // teleportation event
            if (e.type == EVENT_TELEPORT || e.type == EVENT_TELEPORT_ARRIVAL) {
                amod::Customer cust = world_state->getCustomer(e.entity_ids[0]);
                if (fout_.is_open()) fout_ << cust.getPosition().x << " " << cust.getPosition().y;
            }

            // output the location sizes
            if (e.type == EVENT_LOCATION_CUSTS_SIZE_CHANGE ||
            		e.type == EVENT_LOCATION_VEHS_SIZE_CHANGE) {
                amod::Location * ploc = world_state->getLocationPtr(e.entity_ids[0]);
                int curr_size = (e.type == EVENT_LOCATION_VEHS_SIZE_CHANGE)? ploc->getNumVehicles(): ploc->getNumCustomers();
                if (fout_.is_open()) fout_ << curr_size << " " << ploc->getPosition().x << " " << ploc->getPosition().y;
            }
            
            if ((output_move_events_ && e.type == EVENT_MOVE) || (e.type != EVENT_MOVE)) {
                if (fout_.is_open()) fout_ << std::endl;
            }

        }
        // clear events
        world_state->clearEvents();
        
        // dispatch bookings by solving the matching problem
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
            //std::cout << world_state->getCurrentTime() << ": Before Queue Size : " << bookings_queue_.size() << std::endl;
        	//std::cout << world_state->getCurrentTime() << ": Available Vehicles: " << available_vehs_.size() << std::endl;
        	amod::ReturnCode rc = solveMatching(world_state);
        	//std::cout << world_state->getCurrentTime() << ": After Queue Size  : " << bookings_queue_.size() << std::endl;
        	//std::cout << world_state->getCurrentTime() << ": Available Vehicles: " << available_vehs_.size() << std::endl;
            
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
        std::ifstream in(filename.c_str());
        if (!in.good()) {
            std::cout << "Cannot read: " << filename << std::endl;
            return amod::ERROR_READING_BOOKINGS_FILE;
        }
        
        while (in.good()) {
            Booking b;
            in >> b.id >> b.booking_time >> b.cust_id >> b.destination.x >> b.destination.y >> b.travel_mode;
            if (b.id && in.good()) bookings_.emplace(b.booking_time, b); //only positive booking ids allowed
        }
        
        /*
        for (auto itr = bookings_.begin(); itr != bookings_.end(); itr++) {
            auto &b = itr->second;
            std::cout << b.id << ": " << b.booking_time << " " << b.cust_id << " " << b.travel_mode << std::endl;
        }
        */
        
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
    

#ifdef USE_GUROBI
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

    		// optimization function
    		GRBLinExpr obj;

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
					//matching_var[i][j].set(GRB_DoubleAttr_Obj, total_invert_cost);
					//if (total_invert_cost > 0)
					obj += total_invert_cost*matching_var[i][j]; // set optimization function
					matching_var[i][j].set(GRB_StringAttr_VarName, vname.str());

    			}
    		}

    		// Update model to integrate new variables
    		matching_model.update();

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

    		//matching_model.update();


    		// Maximize the inverted costs
    		matching_model.setObjective(obj, GRB_MAXIMIZE); //

			// Solve
			matching_model.optimize();


	    	// perform dispatches
			std::cout << "\nOBJECTIVE: " << matching_model.get(GRB_DoubleAttr_ObjVal) << std::endl;
			std::cout << "SOLUTION:" << std::endl;

			int nbookings = bookings_queue_.size();
			int nvehs = available_vehs_.size();
			for (j=0; j < nbookings ;++j) {
				for (i=0; i < nvehs;++i){
					double opt_x = matching_var[i][j].get(GRB_DoubleAttr_X);

					if(opt_x > 0){

						// vehicle is assigned to this booking
						int bid = index_to_booking_id[j];
						int veh_id = index_to_vehicle_id[i];

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
		for (int i=0; i<available_vehs_.size(); ++i) {
			delete matching_var[i];
		}
		delete [] matching_var;


    	return amod::SUCCESS;
    }
#else
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
				double dist_cost = distance_cost_factor_*(sim_->getDrivingDistance(veh->getPosition(), cust->getPosition()));
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

		// std::cout << k << " " << ncons << std::endl;

		// load the matrix
		// std::cout << "Loading the matrix" << std::endl;
		glp_load_matrix(lp, k-1, ia, ja, ar);

		// solve the problem
		// std::cout << "Solving the problem" << std::endl;

		// integer program
		/*
		glp_iocp parm;
		glp_init_iocp(&parm);
		parm.presolve = GLP_ON;
		glp_intopt(lp, &parm);
		*/
		// linear program
        glp_term_out(GLP_OFF); // suppress terminal output
		glp_simplex(lp, nullptr);

		// print out the objective value
		//double z = glp_mip_obj_val(lp);
		//std::cout << z << std::endl;

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
					std::cout << amod::kErrorStrings[rc] << std::endl;
				} else {
					std::cout << "Assigned " << veh_id << " to booking " << bid << std::endl;
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
    


    amod::ReturnCode ManagerMatchRebalance::solveRebalancing(amod::World *world_state) {
    	if (!world_state) {
    		throw std::runtime_error("solveMatching: world_state is nullptr!");
    	}

    	if (available_vehs_.size() == 0) return amod::SUCCESS; // no vehicles to rebalance

        if (stations_.size() == 0) return amod::SUCCESS; // nothing to rebalance
        
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
			int mean_pred = ceil(dem_est_->predict(sitr->second.getId(), *world_state, world_state->getCurrentTime()).first);
			/*int mean_pred = ceil(std::max(
					(double) dem_est_->predict(sitr->second.getId(), *world_state, world_state->getCurrentTime()).first,
					(double) sitr->second.getNumCustomers()));
			*/
			int cexi = mean_pred - sitr->second.getNumVehicles();

			cex[sitr->first] = cexi; // excess customers at this station
            cex_total += cexi; // total number of excess customers

            if (cexi > 0) {
                nstations_underserved++;
            }
            
            //std::cout << "cex[" << sitr->first << "]: " << cex[sitr->first] << std::endl;
            
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
                //std::cout << "vi[" << sitr->first << "]: " <<  vi[sitr->second.getId()].size() << std::endl;
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
            
            // std::cout << "Even distribution: " <<  floor(vi_total/nstations_underserved) << std::endl;
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
        glp_term_out(GLP_OFF); // suppress terminal output
        glp_simplex(lp, nullptr);
        
        
        // redispatch based on lp solution
        for (int k=1; k<=nvars; k++) {
            // get the value
            int to_dispatch = floor(glp_get_col_prim(lp,k));
            //std::cout << k << ": " << to_dispatch << std::endl;
            if (to_dispatch > 0) {
                int st_source = index_to_ids[k].first;
                int st_dest = index_to_ids[k].second;
                
                // dispatch to_dispatch vehicles form station st_source to st_dest
                amod::ReturnCode rc = interStationDispatch(st_source, st_dest, to_dispatch, world_state, vi);
                if (rc != amod::SUCCESS) {
                    std::cout << amod::kErrorStrings[rc] << std::endl;
                    
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


#endif

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
            std::cout << "Dispatching " << veh_id << " from " << st_source << " to " << st_dest << std::endl;
            auto rc = sim_->dispatchVehicle(world_state, veh_id , itr_dest->second.getPosition(),
                                            VehicleStatus::MOVING_TO_REBALANCE, VehicleStatus::FREE);
            
            // if dispatch is success
            if (rc != amod::SUCCESS) return rc; // return with error code
            
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
