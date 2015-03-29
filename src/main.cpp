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
    
    // populate the world with vehicles and locations
    std::vector<amod::Vehicle> vehicles;
    std::vector<amod::Location> locations;
    std::vector<amod::Customer> customers;
    
    for (int id=1; id<=10; id++) {
        amod::Vehicle veh(id);
        amod::Position pos(rand()%100, rand()%100);
        veh.setPosition(pos);
        vehicles.push_back(veh);
    }
    
    amod::Customer cust(1, "John", amod::Position(120, 120));
    customers.push_back(cust);
    
    world_state.populate(locations, vehicles, customers);
    
    // create the simulator
    double resolution = 0.1;
    amod::SimulatorBasic sim(resolution);
    // initialize the simulator with the world state
    sim.init(&world_state);
    // set simulator parameters
    sim.setVehicleSpeedParams(20.0, 1.0, 15.0, 25.0);
    sim.setPickupDistributionParams(120.0, 10.0, 30.0, 200.0);
    sim.setDropoffDistributionParams(10.0, 1.0, 20.0, 30.0);
    
    sim.update(&world_state);
    
    
    std::vector<amod::Booking> bookings;
    amod::Booking booking;
    
    booking.id = 1;
    booking.booking_time = 5.0;
    booking.cust_id = 1;
    booking.veh_id = 0;
    booking.destination = amod::Position(0, 0);
    bookings.push_back(booking);
    
    // setup our manager
    amod::ManagerBasic simple_manager;
    simple_manager.init(&world_state);
    simple_manager.setSimulator(&sim);
    simple_manager.loadBookings(bookings);
    
    // loop until done
    for (int i=0; i<2000; i++) {
        sim.update(&world_state);
        simple_manager.update(&world_state);
    }
    

    
    // create the logger
    
    // create the manager
    
    // initialize the manager with the world state and the simulator
    
    // while true

        // ask the simulator to perform an update
        
        // ask the manager to perform an update
        
        // get the manager to print some relevant info
    
    // ask the manager to close things and end the logger
    
    // return
    return 0;
}
