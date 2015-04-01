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
    Manager() : sim_(nullptr), logger_(nullptr) {};
    virtual ~Manager() {};

    virtual amod::ReturnCode init(World *world_state) = 0;
    virtual amod::ReturnCode update(World *world_state) = 0;
    virtual amod::ReturnCode loadBookings(const std::vector<Booking> &bookings) = 0;
    
    virtual void setSimulator(amod::Simulator *sim) {
        if (!sim) {
            throw std::runtime_error("Manager::setSimulator: sim is nullptr");
        }
        sim_ = sim;
    };
    
    virtual amod::Simulator* getSimulator() {
        return sim_;
    }
    
    virtual void setLogger(amod::Logger *logger) {
        if (!logger) {
            throw std::runtime_error("Manager::setSimulator: logger is nullptr");
        }
        logger_ = logger;
    };
    
    virtual amod::Logger* getLogger() {
        return logger_;
    }
    

private:
    amod::Simulator* sim_;
    amod::Logger *logger_;

};

} /* namespace AMOD */

#endif /* Manager_H_ */
