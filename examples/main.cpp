#include <iostream>
#include <cstdlib>
#include <vector>

#include "Amod.hpp"
#include "SimulatorBasic.hpp"
#include "ManagerBasic.hpp"

int main(int argc, char **argv) {
    std::cout << "AMOD Basic Test Program" << std::endl;
    
    // create a world state
    amod::World world_state;
    world_state.setCurrentTime(0);
    
    // create vehicles
    std::vector<amod::Vehicle> vehicles;
    for (int id=1; id<=10; id++) {
        amod::Vehicle veh(id); // all vehicles must have a UNIQUE id
        amod::Position pos(rand()%100, rand()%100);
        veh.setPosition(pos);
        vehicles.push_back(veh);
    }
    
    // create customer (just one for now)
    std::vector<amod::Customer> customers;
    int cust_id = 1; // all customers must have a unique id
    amod::Customer cust(cust_id, "John", amod::Position(120, 120));
    customers.push_back(cust);
    
    // Locations are landmark positions
    // basic simulator does not use locations at the moment
    // so we create an empty vector
    std::vector<amod::Location> locations;


    // populate the world
    world_state.populate(locations, vehicles, customers);
    
    // create the simulator
    double resolution = 0.1;
    amod::SimulatorBasic sim(resolution);

    // set simulator parameters
    // all parameters are truncated normal parameters: mean, sd, min, max
    sim.setVehicleSpeedParams(20.0, 1.0, 15.0, 25.0); // in m/s
    sim.setPickupDistributionParams(120.0, 10.0, 30.0, 200.0); // in seconds
    sim.setDropoffDistributionParams(10.0, 1.0, 20.0, 30.0); // in seconds
    
    // initialize the simulator with the world state
    sim.init(&world_state);
    
    // create bookings
    // we will load bookings from a vector
    std::vector<amod::Booking> bookings;
    amod::Booking booking;
    
    booking.id = 1; // unique booking id
    booking.booking_time = 5.0; // in seconds
    booking.cust_id = 1; // which customer to pick up
    booking.veh_id = 0; // veh_id is 0 (the manager will decide this)
    booking.destination = amod::Position(0, 0); //where the customer wants to go
    bookings.push_back(booking);
    
    // setup our manager
    amod::ManagerBasic simple_manager;
    simple_manager.init(&world_state); // initialize
    simple_manager.setSimulator(&sim); // set simulator
    simple_manager.loadBookings(bookings); // load the bookings
    
    // loop until some future time
    while (world_state.getCurrentTime() < 5000) {
        sim.update(&world_state); // update the simulator
        simple_manager.update(&world_state); // update the manager
    }
    
    std::cout << "Simulation Ended" << std::endl;

    // return
    return 0;
}
