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

namespace amod {

class Logger {
public:
	Logger();
    Logger(std::string filename);
	virtual ~Logger();
    
    virtual amod::ReturnCode openLogFile(std::string filename);
    virtual amod::ReturnCode closeLogFile();
    virtual amod::ReturnCode logEvents( amod::World *world_state, bool output_move_events = true, bool clear_events=true);
    
private:
    std::ofstream fout_;
};

} /* namespace AMOD */

#endif /* Logger_H_ */
