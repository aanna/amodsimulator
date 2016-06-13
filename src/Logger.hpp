/*
 * Logger.h
 *
 *  Created on: Mar 23, 2015
 *      Author: haroldsoh, Kasia
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
        	moveEvntInterval = interval;
        } else {
            throw std::runtime_error("Interval cannnot be negative");
        }
    }
    
    virtual double getMoveEventLogInterval() { return moveEvntInterval; };
    
private:
    /// file where the logging is done
    std::ofstream logFile;

    /// move event interval
    double moveEvntInterval;

    /// next move event log time
    double nextMoveEvntLogTime;
};

} /* namespace AMOD */

#endif /* Logger_H_ */
