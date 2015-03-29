#include <iostream>
#include <cstdlib>
#include "Amod.hpp"
#include "SimulatorBasic.hpp"

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
    amod::Position to(120, 120);
    //sim.dispatchVehicle(1, to);
    amod::Booking booking;
    booking.id = 1;
    booking.cust_id = 1;
    booking.veh_id = 1;
    booking.destination = amod::Position(0, 0);
    sim.serviceBooking(booking);
    
    for (int i=0; i<2000; i++) {

        sim.update(&world_state);
        
        // check for events
        std::vector<amod::Event> events;
        world_state.getEvents(&events);
        world_state.clearEvents();
        
        //std::cout << "Current Time: " << world_state.getCurrentTime() << std::endl;
        for (auto e:events) {
            std::cout << e.t << ": Event #" << e.id << " " << e.name << " Entities: ( ";
            for (auto ent: e.entity_ids) {
                std::cout << ent << " ";
            }
            std::cout << ")" << std::endl;
            /*
            if (e.type == amod::EVENT_ARRIVAL) {
                amod::ReturnCode rc = sim.pickupCustomer(e.entity_ids[0], 1);
                if (rc != amod::SUCCESS) {
                    std::cout << "ERROR: Cannot pickup. " << amod::kErrorStrings[rc] << std::endl;
                }
            }
            
            if (e.type == amod::EVENT_PICKUP) {
                
                amod::ReturnCode rc = sim.dropoffCustomer(e.entity_ids[0], e.entity_ids[1]);
                if (rc != amod::SUCCESS) {
                    std::cout << "ERROR: Cannot dropoff. " << amod::kErrorStrings[rc] << std::endl;
                }
            }*/
        }
        
    }
    

    
    // create the logger
    
    // create the manager
    
    // initialize the manager with the world state and the simulator
    
    // while true

    // ask the manager to perform an update
    
    // get the manager to print some relevant info
    
    // ask the manager to close things and end the logger
    
    // return
    return 0;
}
