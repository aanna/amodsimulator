//
//  ManagerMatchRebalance.h
//  AMODBase
//
//  Authors: Harold, Kasia
//

#ifndef __AMODBase__ManagerMatchRebalance__
#define __AMODBase__ManagerMatchRebalance__

#include "Types.hpp"
#include "Manager.hpp"
#include "Booking.hpp"
#include "World.hpp"
#include "Event.hpp"
#include "KDTree.hpp"
#include "SimpleDemandEstimator.hpp"
#include "EmptyTrip.hpp"
#include "TripOffer.hpp"

#include <map>
#include <set>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <sstream>
#include <cstdlib>
#include <unordered_map>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/index/rtree.hpp>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

//#include "spatial_trees/packing_tree/PackingTreeAuraManager.hpp"

#include "glpk.h"


namespace amod {

typedef bg::model::point<float, 2, bg::cs::cartesian> point;
typedef bg::model::box<point> box;
typedef std::pair<box, const Entity *> value;
typedef bgi::rtree<value, bgi::linear<16> > RTree;

    
    class ManagerMatchRebalance : public Manager {
    public:
        
        enum MatchMethod { ASSIGNMENT, GREEDY};
        enum BOOKING_DISCARD_REASONS { CUSTOMER_NOT_AT_LOCATION,
            CUSTOMER_NOT_FREE,
            SERVICE_BOOKING_FAILURE,
            NO_SUITABLE_PATH,
        };
        
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
        
    	/**
    	 * loadRebalancingFromFile
    	 * loads rebalancing counts from a file specified by filename that the manager should respond to.
    	 * @param filename Rebalancing file name
    	 * @return if the call is successful, it returns amod::SUCESSS. Otherwise, it returns
    	 * one of the amod::ReturnCode error codes.
    	 */
    	virtual amod::ReturnCode loadRebalancingFromFile(const std::string& filename);


    	/**
    	 * isDemandManager
    	 * loads rebalancing counts from a file specified by filename that the manager should respond to.
    	 * @param filename Rebalancing file name
    	 * @return if the call is successful, it returns amod::SUCESSS. Otherwise, it returns
    	 * one of the amod::ReturnCode error codes.
    	 */
    	virtual amod::ReturnCode isDemandManager(bool demand_manager);

    	// how long customers can wait
    	virtual amod::ReturnCode loadMaxWaitTime(int maxWaitTime);
    	/**
    	 * updateRebalancingCounts
    	 * gets rebalancing counts that occured during the time period and place it into the rebalancing_ structure
    	 * from the last update until the current time
    	 * @param currTime current time
    	 * @return if the call is successful, it returns amod::SUCESSS. Otherwise, it returns
    	 * one of the amod::ReturnCode error codes.
    	 */

    	virtual amod::ReturnCode updateRebalancingCounts(double currTime);

        // setMatchingMethod
        // sets the matching method to use
        // either ManagerMatchRebalance::ASSIGNMENT or ManagerMatchRebalance::GREEDY
        // inline function
        virtual void setMatchMethod(ManagerMatchRebalance::MatchMethod m) {
            match_method = m;
        }
        
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

        
        // setDemandEstimator
        virtual void setDemandEstimator(amod::DemandEstimator *sde);
       

        // useCurrentQueueForEstimation
        virtual void useCurrentQueueForEstimation(bool use_queue=true) {
            use_current_queue_ = use_queue;
        }
        
        virtual bool isUseCurrentQueueForEstimation() {
            return use_current_queue_;
        }
        
    private:
        std::multimap<double, Booking> bookings_;
    	/// Container to store the rebalancing
    	// the key is the rebalancing time in secs, the value is the EmptyTrip
    	std::multimap<double, EmptyTrip> emptyTrips;
        std::multimap<double, Booking>::iterator bookings_itr_;
    	/// Iterator to the rebalancing container
    	std::multimap<double, EmptyTrip>::iterator reb_itr_;
    	// maximum waiting time for being serviced
    	int max_waiting_time;
        
        std::ifstream bfin_;
        bool use_bookings_file_;
        Booking last_booking_read_;
        
        
        int event_id_;
        std::ofstream fout_; //output file stream for logging
        bool output_move_events_;
        
        // matching variables
        // if demand manager is turned on, then customers can accept/reject the trips
        bool demand_manager;
        // matching method in case of no demand management
        MatchMethod match_method;
        std::set<int> available_vehs_;
        std::map<int, Booking> bookings_queue_;
        double matching_interval_;
        double next_matching_time_;
        double distance_cost_factor_;
        double waiting_time_cost_factor_;
        bgi::rtree<std::pair<box, int>, bgi::linear<16> > locTree_;

        // rebalancing variables
        amod::DemandEstimator *dem_est_;
        std::map<int, amod::Location> stations_; //needs to be a map to ensure ordered lookup
        std::unordered_map<int, int> veh_id_to_station_id_;
        kdt::KDTree<amod::Location> stations_tree_; //for fast NN lookup
        bool use_current_queue_;

        double rebalancing_interval_;
        double next_rebalancing_time_;

    	// is the rebalancing method offline (from file=true) on online=false
    	bool rebalancingFromFile;
    	// if offline then the rebalancing counts should be read from the following file
    	std::ifstream rebalancingFile;
    	// id of rebalancing trips
    	int reb_id;

    	// to measure program running time
    	clock_t t_1_,t_2_, t_3_;

        // demo function to show how to get information from
        // if loc_id is a valid location id, we the waiting customers from that location.
        // if loc_id == 0, then we get all the waiting customers
        virtual int getNumWaitingCustomers(amod::World *world_state, int loc_id = 0);


        virtual int getClosestStationId(const amod::Position &pos) const;

        // solveMatching
        // solves the assignment problem and dispatches vehicles to serve bookings
        virtual amod::ReturnCode solveMatching(amod::World *world_state);
        virtual amod::ReturnCode solveMatchingMinimizing(amod::World *world_state);
        
        // solves the assignment problem in a greedy FIFO manner, within a box
        virtual amod::ReturnCode solveMatchingGreedy(amod::World *world_state);

        // based on current demand, give offers to customers, who can accept or reject
        // two options given to customers are: (i) taxi ride, (ii) shared taxi ride
        virtual amod::ReturnCode solveAssortment(amod::World *world_state);

        // if a passenger is waiting for longer than 5 minutes, discard the trip
        virtual amod::ReturnCode discardTripsWithLongWaiting(amod::World *world_state);

        // find the nearest taxi to provide a private ride option for a customer
        virtual amod::ReturnCode findTheNearestTaxi(amod::World *world_state, const amod::Booking &bk,
        		bgi::rtree<std::pair<box, int>, bgi::linear<16> > vehTree, int &vehId);

        // solveRebalancing
        // solves the rebalancing problem as an LP and dispatches vehicles to other stations.
        virtual amod::ReturnCode solveRebalancing(amod::World *world_state);
        
    	/**
    	 * rebalanceOffline
    	 * the rebalancing problem is solved offline and here we only read the number of counts.
    	 * @param worldState Pointer to amod world
    	 * @return if the call is successful, it returns amod::SUCESSS. Otherwise, it returns
    	 * one of the amod::ReturnCode error codes.
    	 */
    	virtual amod::ReturnCode rebalanceOffline(amod::World *worldState);

        // interStationDispatch
        // sends to_dispatch vehicles from st_source to st_dest
        virtual amod::ReturnCode interStationDispatch(int st_source, int st_dest,
                                                      int to_dispatch,
                                                      amod::World *world_state,
                                                      std::unordered_map<int, std::set<int>> &vi);
        
        
        // updateBookingsFromFile
        // gets bookings that occured during the time period and place it into the bookings_ structure
        // from the last update until the current time
        virtual amod::ReturnCode updateBookingsFromFile(double curr_time);
        
        // isBookingValid
        // performs preliminary checks to ensure the booking is valid
        virtual bool isBookingValid(amod::World *world, const amod::Booking &bk);
    };
}

#endif /* defined(__AMODBase__ManagerMatchRebalance__) */
