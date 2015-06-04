/*
 * Logger.h
 *
 *  Created on: Mar 23, 2015
 *      Author: haroldsoh
 */

#ifndef Logger_H_
#define Logger_H_


#include "Event.hpp"
#include "Types.hpp"
#include "World.hpp"

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <stdexcept>

namespace amod {

class Logger {
public:
	Logger();
    Logger(std::string filename);
	virtual ~Logger();
    
    virtual amod::ReturnCode openLogFile(std::string filename);
    virtual amod::ReturnCode closeLogFile();
    virtual amod::ReturnCode logEvents( amod::World *world_state, bool output_move_events = true, bool clear_events=true);
    
    virtual void setMoveEventLogInterval(double interval) {
        if (interval >= 0) {
            move_event_interval_ = interval;
        } else {
            throw std::runtime_error("Interval cannnot be negative");
        }
    }
    
    virtual double getMoveEventLogInterval() { return move_event_interval_; };
    
private:
    std::ofstream fout_;
    double move_event_interval_;
    double next_move_event_log_time_;
};

} /* namespace AMOD */

#endif /* Logger_H_ */
