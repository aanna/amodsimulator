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

int main(int argc, char **argv) {
    std::cout << "AMOD Basic Test Program" << std::endl;
    
    // create a world state
    amod::World world_state;
    world_state.setCurrentTime(0);
    
    // create vehicles
    int max_x = 10000;
    int max_y = 10000;
    int num_vehs = 3000;
    int num_cust = 40000;
    int num_bookings = 20000;
    int max_time = 5000;

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
    amod::SimulatorBasic sim(resolution, verbose);

    // set simulator parameters
    // all parameters are truncated normal parameters: mean, sd, min, max
    sim.setVehicleSpeedParams(25.0, 5.0, 20.0, 30.0); // in m/s
    sim.setPickupDistributionParams(20.0, 10.0, 5.0, 50.0); // in seconds
    sim.setDropoffDistributionParams(10.0, 1.0, 0.0, 0.0); // in seconds
    
    // initialize the simulator with the world state
    sim.init(&world_state);
    
    // create bookings
    // we will load bookings from a vector
    std::vector<amod::Booking> bookings;
    for (int id=1; id <=num_bookings; id++) {
        amod::Booking booking;
        booking.id = id; // unique booking id
        booking.booking_time = rand()%((int) (max_time - 0.2*max_time)); // in seconds
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
    match_manager.setMatchingInterval(30); //5 minutes

    match_manager.init(&world_state);
    match_manager.setSimulator(&sim); // set simulator
    match_manager.loadBookings(bookings); // load the bookings

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
    } else {
        std::cout << "Number of vehicles match up." << std::endl;
        std::cout << "Did the simulation finish running?" << std::endl;
    }
    
    if (num_cust != total_cust) {
        std::cout << "Error! Total number of customers before and after simulation is not the same";
        std::cout << "Before: " << num_cust << " After: " << total_cust << std::endl;
        std::cout << "Did the simulation finish running?" << std::endl;
    } else {
        std::cout << "Number of customers match up." << std::endl;
    }
    
    // return
    return 0;
}
