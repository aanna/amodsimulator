//
//  ManagerMatchRebalance.h
//  AMODBase
//
//  Created by Harold Soh on 29/3/15.
//  Copyright (c) 2015 Harold Soh. All rights reserved.
//

#ifndef __AMODBase__ManagerMatchRebalance__
#define __AMODBase__ManagerMatchRebalance__

#include "Types.hpp"
#include "Manager.hpp"
#include "Booking.hpp"
#include "World.hpp"
#include "Event.hpp"
#include "KDTree.hpp"

#include <map>
#include <set>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <sstream>
#include <cstdlib>

// gurobi for optimization
//#define USE_GUROBI
#ifdef USE_GUROBI
#include "gurobi_c++.h"
#else
// we use GLPK
#include "glpk.h"
#endif

namespace amod {
    
    class ManagerMatchRebalance : public Manager {
    public:
        ManagerMatchRebalance();
        virtual ~ManagerMatchRebalance();
        
        // init
        // initializes the manager with the World world_state
        // if the call is successful, it returns amod::SUCESSS. Otherwise, it returns
        // one of the amod::ReturnCode error codes.
        virtual amod::ReturnCode init(World *world_state);
        
        // update
        // updates the manager with the World world_state. This manager is a simple
        // demonstration of the manager and is a simple queue (FIFO) manager which dispatches
        // the closest FREE or PARKED vehicle. Bookings are responded to by booking time.
        // If there are no available vehicles, the booking remains in the booking queue.
        // if the call is successful, it returns amod::SUCESSS. Otherwise, it returns
        // one of the amod::ReturnCode error codes.
        virtual amod::ReturnCode update(World *world_state);
    
        // loadBookings
        // loads bookings that the manager should respond to.
        // if the call is successful, it returns amod::SUCESSS. Otherwise, it returns
        // one of the amod::ReturnCode error codes.
        virtual amod::ReturnCode loadBookings(const std::vector<Booking> &bookings);
        
        // loadBookingsFromFile
        // loads bookings from a file specified by filename that the manager should respond to.
        // if the call is successful, it returns amod::SUCESSS. Otherwise, it returns
        // one of the amod::ReturnCode error codes.
        virtual amod::ReturnCode loadBookingsFromFile(const std::string filename);
        
        // setCostFactors
        // sets the multiplicative factors for the individual matching cost components. Currently
        // there is the distance_cost_factor and the waiting_time_cost_factor
        // both factors default to 1.0
        virtual void setCostFactors(double distance_cost_factor, double waiting_time_cost_factor);

        // set and get the matching interval
        // the default matching interval is 60
        virtual void setMatchingInterval(double matching_interval);
        virtual double getMatchingInterval() const;


        // set and get rebalancing interval
        // the default rebalancing interval is 300 (every 5 minutes)
        virtual void setRebalancingInterval(double rebalancing_interval);
        virtual double getRebalancingInterval() const;

        // loadStations
        // loads the stations which are used to house vehicles
        virtual void loadStations(std::vector<amod::Location> &stations, const amod::World &world_state);


    private:
        std::multimap<double, Booking> bookings_;
        std::multimap<double, Booking>::iterator bookings_itr_;
        
        std::ofstream out;

        // matching varibles
        std::set<int> available_vehs_;
        std::map<int, Booking> bookings_queue_;
        double matching_interval_;
        double next_matching_time_;
        double distance_cost_factor_;
        double waiting_time_cost_factor_;

        // rebalancing variables
        std::map<int, amod::Location> stations_; //needs to be a map to ensure ordered lookup
        std::unordered_map<int, int> veh_id_to_station_id_;
        kdt::KDTree<amod::Location> stations_tree_; //for fast NN lookup

        double rebalancing_interval_;
        double next_rebalancing_time_;

        // Gurobi
#ifdef USE_GUROBI
        GRBEnv *gurobi_env_;
#endif
        // demo function to show how to get information from
        // if loc_id is a valid location id, we the waiting customers from that location.
        // if loc_id == 0, then we get all the waiting customers
        virtual int getNumWaitingCustomers(amod::World *world_state, int loc_id = 0);


        virtual int getClosestStationId(const amod::Position &pos) const;

        // solveMatching
        // solves the assignment problem and dispatches vehicles to serve bookings
        virtual amod::ReturnCode solveMatching(amod::World *world_state);

        // solveRebalancing
        // solves the rebalancing problem as an LP and dispatches vehicles to other stations.
        virtual amod::ReturnCode solveRebalancing(amod::World *world_state);
        
        // interStationDispatch
        // sends to_dispatch vehicles from st_source to st_dest
        virtual amod::ReturnCode interStationDispatch(int st_source, int st_dest,
                                                      int to_dispatch,
                                                      amod::World *world_state,
                                                      std::unordered_map<int, std::set<int>> &vi);

    };
}

#endif /* defined(__AMODBase__ManagerMatchRebalance__) */
