/*
 * Manager.h
 *  Abstract base class for managers to inherit from
 *
 *  Created on: Mar 23, 2015
 *      Author: haroldsoh
 */

#ifndef Manager_H_
#define Manager_H_

#include "Types.hpp"
#include "Simulator.hpp"
#include "Logger.hpp"
#include "Booking.hpp"
#include <stdexcept>

namespace amod {

class Manager {
public:
    Manager() : sim_(nullptr), logger_(nullptr), verbose_(false) {};
    virtual ~Manager() {};

    // init
    // initializes the manager with the World world_state
    // if the call is successful, it returns amod::SUCESSS. Otherwise, it returns
    // one of the amod::ReturnCode error codes.
    virtual amod::ReturnCode init(World *world_state) = 0;
    
    // update
    // updates the manager with the World world_state
    // this should only be called after calling setSimulator
    // To keep things semantically simple, the manager should not attempt to directly modify
    // the vehicles, customers and locations directly. The simulator will modify the state
    // of the world and the manager will effect change indirectly via calls to the simulator.
    // Access should be limited to the events (i.e., reading events), responding to them and
    // clearing events that have been responded to.
    // if the call is successful, it returns amod::SUCESSS. Otherwise, it returns
    // one of the amod::ReturnCode error codes.
    virtual amod::ReturnCode update(World *world_state) = 0;
    
    // loadBookings
    // loads bookings that the manager should respond to.
    // if the call is successful, it returns amod::SUCESSS. Otherwise, it returns
    // one of the amod::ReturnCode error codes.
    virtual amod::ReturnCode loadBookings(const std::vector<Booking> &bookings) = 0;
    
    // setSimulator
    // sets the simulator that this manager will use.
    virtual void setSimulator(amod::Simulator *sim) {
        if (!sim) {
            throw std::runtime_error("Manager::setSimulator: sim is nullptr");
        }
        sim_ = sim;
    };
    
    // getSimulator
    // gets the simulator that this manager will use.
    virtual amod::Simulator* getSimulator() {
        return sim_;
    }
    
    // setLogger
    // sets the logger for this manager
    virtual void setLogger(amod::Logger *logger) {
        if (!logger) {
            throw std::runtime_error("Manager::setSimulator: logger is nullptr");
        }
        logger_ = logger;
    };
    
    // getLogger
    // returns the logger
    virtual amod::Logger* getLogger() {
        return logger_;
    }
    
    virtual void setVerbose(bool v) {
        verbose_ = v;    
    }
    
    virtual bool getVerbose() {
        return verbose_;
    }
    

protected:
    amod::Simulator* sim_;
    amod::Logger *logger_;
    
    bool verbose_;

};

} /* namespace AMOD */

#endif /* Manager_H_ */
