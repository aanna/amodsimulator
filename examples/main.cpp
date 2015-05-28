#include <iostream>
#include <cstdlib>
#include <vector>
#include <sstream>
#include <limits.h>
#include <unordered_map>

#include "Amod.hpp"
#include "SimulatorBasic.hpp"
#include "ManagerBasic.hpp"
#include "ManagerMatchRebalance.hpp"
#include "SimpleDemandEstimator.hpp"

void basicTest(void) {
    // create a world state
    amod::World world_state;
    world_state.setCurrentTime(0);
    
    // create vehicles
    int max_x = 10000;
    int max_y = 10000;
    int num_vehs = 100;
    int num_cust = 2000;
    int num_bookings = 2000;
    int max_time = 2000;
    
    std::vector<amod::Vehicle> vehicles;
    for (int id=1; id<=num_vehs; id++) {
        amod::Vehicle veh(id); // all vehicles must have a UNIQUE id
        amod::Position pos(rand()%max_x, rand()%max_y);
        veh.setStatus(amod::VehicleStatus::FREE);
        veh.setPosition(pos);
        vehicles.push_back(veh);
    }
    
    // create customer (just one for now)
    std::vector<amod::Customer> customers;
    for (int id=1; id<=num_cust; id++) {
        int cust_id = id; // all customers must have a unique id
        std::stringstream ss;
        ss << id;
        amod::Customer cust(cust_id, ss.str(), amod::Position(rand()%max_x, rand()%max_y));
        customers.push_back(cust);
    }
    
    // Locations are places that are simulated (travel only occurs between locations
    std::vector<amod::Location> locations;
    int num_loc = 10;
    for (int id=1; id<= num_loc; ++id) {
        std::stringstream ss;
        ss << id;
        locations.emplace_back(id, ss.str(), amod::Position(rand()%max_x, rand()%max_y), INT_MAX );
    }
    
    // populate the world
    world_state.populate(locations, vehicles, customers);
    
    // create the simulator
    double resolution = 0.1;
    bool verbose = true;
    amod::SimulatorBasic sim(resolution);
    sim.setVerbose(verbose);
    
    // set simulator parameters
    // all parameters are truncated normal parameters: mean, sd, min, max
    sim.setVehicleSpeedParams(25.0, 5.0, 20.0, 30.0); // in m/s
    sim.setPickupDistributionParams(20.0, 10.0, 5.0, 50.0); // in seconds
    sim.setDropoffDistributionParams(10.0, 1.0, 5.0, 10.0); // in seconds
    
    // initialize the simulator with the world state
    sim.init(&world_state);
    
    // create bookings
    // we will load bookings from a vector
    std::vector<amod::Booking> bookings;
    for (int id=1; id <=num_bookings; id++) {
        amod::Booking booking;
        booking.id = id; // unique booking id
        booking.booking_time = rand()%((int) (max_time - 0.3*max_time)); // in seconds
        booking.cust_id = id; // which customer to pick up
        booking.veh_id = 0; // veh_id is 0 (the manager will decide this)
        booking.destination = amod::Position( rand()%max_x, rand()%max_y ); //where the customer wants to go
        bookings.push_back(booking);
    }
    // setup our manager
    amod::ManagerBasic simple_manager;
    simple_manager.init(&world_state); // initialize
    simple_manager.setSimulator(&sim); // set simulator
    simple_manager.loadBookings(bookings); // load the bookings
    
    // create another manager
    amod::ManagerMatchRebalance match_manager;
    double distance_cost_factor = 1.0;
    double waiting_cost_factor = 1.0;
    match_manager.setCostFactors(distance_cost_factor, waiting_cost_factor);
    match_manager.setMatchingInterval(30); //30 seconds
    
    match_manager.init(&world_state);
    match_manager.setSimulator(&sim); // set simulator
    match_manager.loadBookings(bookings); // load the bookings
    
    // create stations and load
    // stations are locations that can we can park vehicles
    std::vector<amod::Location> stations;
    
    match_manager.loadStations(stations, world_state); // load the stations
    
    // select which manager we want
    amod::Manager* manager = &match_manager; //simple_manager
    //amod::Manager* manager = &simple_manager;
    // loop until some future time
    while (world_state.getCurrentTime() < max_time) {
        sim.update(&world_state); // update the simulator
        amod::ReturnCode rc = manager->update(&world_state); // update the manager
        if (rc != amod::SUCCESS) {
            std::cout << "ERROR: " << world_state.getCurrentTime() << ": " << amod::kErrorStrings[rc] << std::endl;
        }
    }
    
    std::cout << "Simulation Ended" << std::endl;
    
    // checks
    // make sure the location sizes are correct
    std::unordered_map<int, amod::Location>::const_iterator bitr, eitr;
    world_state.getLocations(&bitr, &eitr);
    int total_cust = 0;
    int total_veh = 0;
    for (auto itr = bitr; itr != eitr; itr++) {
        total_cust += itr->second.getNumCustomers();
        total_veh += itr->second.getNumVehicles();
    }
    
    if (num_vehs != total_veh) {
        std::cout << "Error! Total number of vehicles before and after simulation is not the same";
        std::cout << "Before: " << num_vehs << " After: " << total_veh << std::endl;
        std::cout << "Did the simulation finish running?" << std::endl;
    } else {
        std::cout << "Number of vehicles match up." << std::endl;
    }
    
    if (num_cust != total_cust) {
        std::cout << "Error! Total number of customers before and after simulation is not the same";
        std::cout << "Before: " << num_cust << " After: " << total_cust << std::endl;
        std::cout << "Did the simulation finish running?" << std::endl;
    } else {
        std::cout << "Number of customers match up." << std::endl;
    }
    
}

void rebalanceTest() {
    // create a world state
    amod::World world_state;
    world_state.setCurrentTime(0);
    
    // create vehicles
    int num_vehs = 50;
    int num_cust = 20;
    int max_time = 5;
    
    // create two positions at (0,0) and (100, 100)
    
    // put all vehicles at (0,0)
    std::vector<amod::Vehicle> vehicles;
    for (int id=1; id<=num_vehs; id++) {
        amod::Vehicle veh(id); // all vehicles must have a UNIQUE id
        amod::Position pos(0, 0);
        veh.setStatus(amod::VehicleStatus::FREE);
        veh.setPosition(pos);
        vehicles.push_back(veh);
    }
    
    // create customers all at either (10000,10000) or (5000, 0)
    std::vector<amod::Customer> customers;
    for (int id=1; id<=num_cust; id++) {
        int cust_id = id; // all customers must have a unique id
        std::stringstream ss;
        ss << id;
        if (id%2 == 0) {
            amod::Customer cust(cust_id, ss.str(), amod::Position(10000, 10000));
            cust.setStatus(amod::CustomerStatus::FREE);
            customers.push_back(cust);
        } else {
            amod::Customer cust(cust_id, ss.str(), amod::Position(5000, 0));
            cust.setStatus(amod::CustomerStatus::FREE);
            customers.push_back(cust);
        }
    }
    
    // Locations are places that are simulated (travel only occurs between locations
    std::vector<amod::Location> locations;
    locations.emplace_back(1, "0", amod::Position(0, 0), INT_MAX );
    locations.emplace_back(2, "1", amod::Position(10000, 10000), INT_MAX );
    locations.emplace_back(3, "2", amod::Position(5000, 0), INT_MAX );
    
    
    // populate the world
    world_state.populate(locations, vehicles, customers);
    
    // create the simulator
    double resolution = 0.1;
    bool verbose = true;
    amod::SimulatorBasic sim(resolution);
    sim.setVerbose(verbose);
    // set simulator parameters
    // all parameters are truncated normal parameters: mean, sd, min, max
    sim.setVehicleSpeedParams(25.0, 5.0, 20.0, 30.0); // in m/s
    sim.setPickupDistributionParams(20.0, 10.0, 5.0, 50.0); // in seconds
    sim.setDropoffDistributionParams(10.0, 1.0, 5.0, 20.0); // in seconds
    
    // initialize the simulator with the world state
    sim.init(&world_state);
    
    // create bookings
    // we will load bookings from a vector
    std::vector<amod::Booking> bookings;
    for (int id=1; id <=num_cust; id++) {
        amod::Booking booking;
        booking.id = id; // unique booking id
        booking.booking_time = 0; // in seconds
        booking.cust_id = id; // which customer to pick up
        booking.veh_id = 0; // veh_id is 0 (the manager will decide this)
        booking.destination = amod::Position( 0, 0 ); //where the customer wants to go
        bookings.push_back(booking);
    }
    
    
    // create the maching rebalancing manager
    amod::ManagerMatchRebalance match_manager;
    double distance_cost_factor = 1.0;
    double waiting_cost_factor = 1.0;
    match_manager.setCostFactors(distance_cost_factor, waiting_cost_factor);
    match_manager.setMatchingInterval(1e10); //it never does the matching
    match_manager.setRebalancingInterval(10);
    
    match_manager.init(&world_state);
    match_manager.setSimulator(&sim); // set simulator
    match_manager.loadBookings(bookings); // load the bookings
    
    // create stations and load
    // stations are locations that can we can park vehicles
    std::vector<amod::Location> stations;
    stations = locations; // stations equivalent to locations
    match_manager.loadStations(stations, world_state); // load the stations

    
    // select which manager we want
    amod::Manager* manager = &match_manager; //simple_manager
    //amod::Manager* manager = &simple_manager;
    // loop until some future time
    while (world_state.getCurrentTime() < max_time) {
        sim.update(&world_state); // update the simulator
        amod::ReturnCode rc = manager->update(&world_state); // update the manager
        if (rc != amod::SUCCESS) {
            std::cout << "ERROR: " << world_state.getCurrentTime() << ": " << amod::kErrorStrings[rc] << std::endl;
        }
    }
    
    std::cout << "Simulation Ended" << std::endl;

}

template <typename T>
void loadEntities(std::string filename, std::vector<T> *ts, bool skip_header=false) {
	std::ifstream in(filename.c_str());
	if (!in.good()) {
		throw std::runtime_error("Cannot read locations file");
	}

	// skip header if necessary
	if (skip_header) {
        std::string header_line;
        getline(in, header_line);
	}

	while (in.good()) {
		int id;
		double x, y;
		in >> id >> x >> y;
		if (id && in.good()) {
			T t;
	        std::stringstream ss;
	        ss << id;
			t.setId(id);
			t.setPosition({x,y});
			t.setName(ss.str());
			ts->emplace_back(t);
		}
	}

	// check
	std::cout.precision(10);
	std::cout << ts->size() << std::endl;
	for (auto t : *ts) {
		std::cout << t.getId() << " " << t.getPosition().x << " " << t.getPosition().y << std::endl;
	}
}

enum ManagerType {
    SIMPLE_MANAGER,
    MATCH_MANAGER,
    MATCH_REBALANCE_MANAGER,
    MATCH_REBALANCE_PREDICT_MANAGER,
};

void starNetworkTest(ManagerType mgr_type) {
	// set parameters
    double max_time = 24*60*60*2;

	// load locations
	std::string locs_filename = "data/starnetwork_locs.txt";
	std::vector<amod::Location> locations;
	loadEntities(locs_filename, &locations);

	// load customers
	std::string custs_filename = "data/starnetwork_custs.txt";
	std::vector<amod::Customer> customers;
	loadEntities(custs_filename, &customers);
	for (auto itr=customers.begin(); itr!=customers.end(); ++itr) {
		itr->setStatus(amod::CustomerStatus::FREE);
	}

	// load vehicles
	std::string vehs_filename = "data/starnetwork_vehs.txt";
	std::vector<amod::Vehicle> vehicles;
	loadEntities(vehs_filename, &vehicles);
	for (auto itr=vehicles.begin(); itr!=vehicles.end(); ++itr) {
		itr->setStatus(amod::VehicleStatus::FREE);
	}

	// load stations
	std::string stns_filename = "data/starnetwork_stns.txt";
	std::vector<amod::Location> stations;
	loadEntities(stns_filename, &stations);

	// setup world
    amod::World world_state;
    world_state.setCurrentTime(0);
    world_state.populate(locations, vehicles, customers);

    // create the simulator
    double resolution = 1;
    bool verbose = false;
    amod::SimulatorBasic sim(resolution);
    sim.setVerbose(verbose);
    
    // set simulator parameters
    // all parameters are truncated normal parameters: mean, sd, min, max
    sim.setVehicleSpeedParams(25.0, 5.0, 20.0, 25.0); // in m/s
    sim.setPickupDistributionParams(20.0, 10.0, 20.0, 20.0); // in seconds
    sim.setDropoffDistributionParams(10.0, 1.0, 20.0, 20.0); // in seconds
    sim.setTeleportDistributionParams(10.0, 2.0, 20.0, 20.0); // in seconds

    // initialize the simulator with the world state
    sim.init(&world_state);

	// setup manager
    // setup our manager
    amod::ManagerBasic simple_manager;
    simple_manager.init(&world_state); // initialize
    simple_manager.setSimulator(&sim); // set simulator
    std::string books_filename = "data/starnetwork_all_books.txt";
    simple_manager.loadBookingsFromFile(books_filename); // load the bookings

    // setup our demand estimator
    amod::SimpleDemandEstimator sde;
    sde.loadLocations(stations);

    // setup our manager
    amod::ManagerMatchRebalance match_manager;
    double distance_cost_factor = 1.0;
    double waiting_cost_factor = 1.0;
    match_manager.setCostFactors(distance_cost_factor, waiting_cost_factor);
    
    match_manager.init(&world_state); // initialize
    match_manager.setSimulator(&sim); // set simulator
    match_manager.loadStations(stations, world_state);
    match_manager.loadBookingsFromFile(books_filename); // load the bookings
    match_manager.setMatchingInterval(5);
    match_manager.setVerbose(verbose);

    // setup out logger
    amod::Logger logger;
    
	// set the manager we want to use
    amod::Manager* manager = nullptr;
    bool output_move_events = true;
    std::string demand_filename, demand_hist_filename, demand_pred_hist_filename;
    switch (mgr_type) {
    case SIMPLE_MANAGER:
    	logger.openLogFile("spLog.txt");

    	manager = &simple_manager;
    	break;
    case MATCH_MANAGER:
        demand_filename = "data/starnetwork_all_demands.txt";
        sde.loadDemandFromFile(demand_filename);
        match_manager.setDemandEstimator(&sde); // set the demand estimator (for rebalancing)
    	logger.openLogFile("maLog.txt");
    	match_manager.setRebalancingInterval(1e10); //effectively never
    	manager = &match_manager;
    	break;

    case MATCH_REBALANCE_MANAGER:
        demand_hist_filename = "data/starnetwork_all_demands_hist.txt";
        sde.loadDemandHistFromFile(demand_hist_filename);
        match_manager.setDemandEstimator(&sde); // set the demand estimator (for rebalancing)
    	logger.openLogFile("mrLog.txt");
        match_manager.setRebalancingInterval(1*60*60); //every hour
    	manager = &match_manager;
    	break;
    case MATCH_REBALANCE_PREDICT_MANAGER:
        demand_pred_hist_filename = "data/starnetwork_all_pred_demands_hist.txt";
        sde.loadDemandHistFromFile(demand_pred_hist_filename);
        match_manager.setDemandEstimator(&sde); // set the demand estimator (for rebalancing)
    	logger.openLogFile("mrpLog.txt");
        match_manager.setRebalancingInterval(1*60*60); //every hour
    	manager = &match_manager;
        break;
    }
    
    // loop until some future time
    std::cout << "Starting Simulation" << std::endl;
    int percent_done = 0;
    while (world_state.getCurrentTime() < max_time) {
        int curr_percent_done = floor(world_state.getCurrentTime()*100/max_time);
        if (curr_percent_done > percent_done) {
            percent_done = curr_percent_done;
            std::cout << percent_done << "\% complete" << std::endl;
        }
        sim.update(&world_state); // update the simulator
        amod::ReturnCode rc = manager->update(&world_state); // update the manager
        if (rc != amod::SUCCESS) {
            std::cout << "ERROR: " << world_state.getCurrentTime() << ": " << amod::kErrorStrings[rc] << std::endl;
        }
        logger.logEvents(&world_state);
    }
    logger.closeLogFile();
    
    std::cout << "Simulation Ended" << std::endl;
   
}






void simpleDemandEstimatorTest() {
    amod::SimpleDemandEstimator sde(3600); //hourly bins
    
    // load stations
    std::string stns_filename = "data/starnetwork_stns.txt";
    std::vector<amod::Location> stations;
    loadEntities(stns_filename, &stations);
    sde.loadLocations(stations); // we only need estimated demand at stations
    
    //std::string demand_filename = "data/starnetwork_demands.txt";
    //sde.loadDemandFromFile(demand_filename);
    std::string demand_hist_filename = "data/starnetwork_all_pred_demands_hist.txt";
    sde.loadDemandHistFromFile(demand_hist_filename);
    // create an empty world state
    amod::World world;
    
    // simple output to disk
    std::ofstream fout("demandEstimatorTestResults.txt");
    for (auto s : stations) {
        for (double t = 0; t < 3*24*60*60; t += 3600) {
            auto pred = sde.predict(s.getId(), world, t);
            fout << s.getId() << " " << t << " " << pred.first << " " << pred.second << std::endl;
            std::cout << s.getId() << " " << t << " " << pred.first << " " << pred.second << std::endl;
        }
    }
    fout.close();
    
}

// ********************************************************************
// Test Using Singapore Mid Term Data
// ********************************************************************

void loadSingaporeMidTermData(std::string activities_filename,
		std::string locs_filename,
		std::string stations_filename,
		int nvehicles,
		std::vector<amod::Customer> *pcustomers,
		std::vector<amod::Vehicle> *pvehicles,
		std::vector<amod::Location> *plocations,
		std::vector<amod::Location> *pstations,
		std::vector<amod::Booking> *pbookings
		) {

	// create references to simplify syntax
	auto &customers = *pcustomers;
	auto &vehicles = *pvehicles;
	auto &locations = *plocations;
	auto &stations = *pstations;
	auto &bookings = *pbookings;

	//loadEntities(locs_filename, plocations, true); //locations

	loadEntities(stations_filename, pstations, true); //stations

	// load the activities file
	std::ifstream in(activities_filename.c_str());
	if (!in.good()) {
		throw std::runtime_error("Cannot read activities file");
	}

	// read in the travel modes
	int nmodes;
	in >> nmodes;
	if (nmodes == 0) {
		throw std::runtime_error("Num modes is zero!");
	}

	struct Mode {
		int id;
		std::string name;
	};

	std::map<int, Mode> modes;

	for (int i=0; i<nmodes; ++i) {
		Mode m;
		in >> m.id >> m.name;
		modes.insert({m.id, m});
	}

//	for (auto itr = modes.begin(); itr != modes.end(); ++itr) {
//		std::cout << itr->first << ": " << itr->second.name << std::endl;
//	}

	// read in the locations
	int nlocs;
	in >> nlocs;
	std::cout << "Loading " << nlocs << " locations" << std::endl;
	std::map<int, amod::Position> locs;

	for (int i=0; i<nlocs; ++i) {

		int id;
		amod::Position p;
		std::stringstream ss;
		in >> id >> p.x >> p.y;
		locations.emplace_back(id, ss.str(), p, 1e10);
		locs.insert({id,p});
	}

//	for (int i=0; i<nlocs; i++) {
//		std::cout << locations[i].getId() << " " << locations[i].getPosition().x << " " << locations[i].getPosition().y << std::endl;
//	}

	// read in the customers
	int ncusts;
	in >> ncusts;
	std:: cout << "Loading " << ncusts << " customers" << std::endl;
	for (int i=0; i<ncusts; ++i) {
		int id;
		int home_id;
		in >> id >> home_id;
		std::stringstream ss;
		ss << id;
		customers.emplace_back(id, ss.str(), locs[home_id]);
	}

	// read in the activities
	int nacts;
	in >> nacts;
	std:: cout << "Loading " << nacts << " activities/bookings" << std::endl;
	for (int i=0; i<nacts; ++i) {
		amod::Booking b;
		double des_arrival_time;
		int source_node_id;
		int dest_node_id;
		int mode_choice;
		in >> b.id >> b.cust_id >> b.booking_time >> source_node_id >> dest_node_id >> mode_choice >> des_arrival_time;
		if (modes[mode_choice].name == "Car" || modes[mode_choice].name == "Taxi") {
			b.travel_mode = amod::Booking::AMODTRAVEL;
		} else {
			b.travel_mode = amod::Booking::TELEPORT;
		}
		b.booking_time -= 10800; //starting time;
		b.destination = locs[dest_node_id];
		bookings.emplace_back(b);
	}

	// setup the vehicles
	int nstations = stations.size();
	int nvehsperstation = nvehicles/nstations;
	int remvehs = nvehicles%nstations;

	// loop through all stations
	int id = 1;
	for (int i=0; i<nstations; ++i) {
		for (int j=0; j<nvehsperstation; ++j) {
			std::stringstream ss;
			ss << id;
			vehicles.emplace_back(id++, ss.str(), stations[i].getPosition(), 1, amod::VehicleStatus::FREE);
		}
	}

	return;
}

void singaporeMidTermTest(ManagerType mgr_type) {

}

int main(int argc, char **argv) {
    std::cout << "AMOD Basic Test Program" << std::endl;
    // run basic test
    //basicTest();
    //rebalanceTest();
    //starNetworkTest(SIMPLE_MANAGER);
    //starNetworkTest(MATCH_MANAGER);
    //starNetworkTest(MATCH_REBALANCE_MANAGER);
    starNetworkTest(MATCH_REBALANCE_PREDICT_MANAGER);
    //simpleDemandEstimatorTest();

    //singaporeMidTermTest(SIMPLE_MANAGER);

    // return
    return 0;
}
