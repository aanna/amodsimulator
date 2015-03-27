#include <iostream>
#include <cstdlib>
#include "Amod.hpp"
#include "SimulatorBasic.hpp"

int main(int argc, char **argv) {
    std::cout << "AMOD Basic Test Program" << std::endl;

    // create a world state
    amod::World world_state();

    // populate the world with vehicles and locations
    std::vector<amod::Vehicle> vehicles;
    std::vector<amod::Location> locations;
    for (int id=1; id<=10; id++) {
    	amod::Vehicle car(id);
    	amod::Position pos(rand()%100, rand()%100);
    	car.setPosition(pos);
    }

    // create the simulator
    amod::SimulatorBasic sim;

    // initialize the simulator with the world state

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
