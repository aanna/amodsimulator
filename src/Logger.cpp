/*
 * Logger.cpp
 *
 *  Created on: Mar 23, 2015
 *      Author: haroldsoh
 */

#include "Logger.hpp"

namespace amod {

Logger::Logger() {
	// TODO Auto-generated constructor stub
}

Logger::Logger(std::string filename) {
    openLogFile(filename);
}

Logger::~Logger() {
    if (fout_.is_open()) {
        fout_.close();
    }
}

amod::ReturnCode Logger::openLogFile(std::string filename) {
    fout_.open(filename.c_str());
    if (!fout_) return amod::CANNOT_OPEN_LOGFILE;
    fout_.precision(10);
    return amod::SUCCESS;
}

amod::ReturnCode Logger::closeLogFile() {
    fout_.close();
    return amod::SUCCESS;
}

amod::ReturnCode Logger::logEvents(amod::World *world_state, bool output_move_events, bool clear_events) {
    
    // first version, log events differently depending on type of events
    // future version would streamline this
    std::vector<Event> events;
    world_state->getEvents(&events);
    // respond to events
    for (auto e:events) {
        if (fout_.is_open()) {
            if ((output_move_events && e.type == EVENT_MOVE) || (e.type != EVENT_MOVE)) {
                fout_ << e.t << " Event " << e.id << " " << e.type << " " << e.name << " Entities: ";
                for (auto ent: e.entity_ids) {
                    fout_ << ent << ",";
                }
                fout_ << " ";
            }
        }
        
        if (e.type == EVENT_MOVE || e.type == EVENT_ARRIVAL || e.type == EVENT_PICKUP || e.type == EVENT_DROPOFF) {
            amod::Vehicle veh = world_state->getVehicle(e.entity_ids[0]);
            
            if (fout_.is_open()) {
                if ((output_move_events && e.type == EVENT_MOVE) || (e.type != EVENT_MOVE)) {
                    fout_ << veh.getPosition().x << " " << veh.getPosition().y << " " << veh.getStatus();
                }
            }
        }
        
        // teleportation event
        if (e.type == EVENT_TELEPORT || e.type == EVENT_TELEPORT_ARRIVAL) {
            amod::Customer cust = world_state->getCustomer(e.entity_ids[0]);
            if (fout_.is_open()) fout_ << cust.getPosition().x << " " << cust.getPosition().y << " " << cust.getStatus();
        }

        // output the location sizes
        if (e.type == EVENT_LOCATION_CUSTS_SIZE_CHANGE ||
                e.type == EVENT_LOCATION_VEHS_SIZE_CHANGE) {
            amod::Location * ploc = world_state->getLocationPtr(e.entity_ids[0]);
            int curr_size = (e.type == EVENT_LOCATION_VEHS_SIZE_CHANGE)? ploc->getNumVehicles(): ploc->getNumCustomers();
            if (fout_.is_open()) fout_ << ploc->getPosition().x << " " << ploc->getPosition().y << " " << curr_size;
        }
        
        if (e.type == EVENT_DISPATCH) {
             amod::Vehicle veh = world_state->getVehicle(e.entity_ids[0]);
            if (fout_.is_open()) fout_ << veh.getStatus();
        }
        
        if ((output_move_events && e.type == EVENT_MOVE) || (e.type != EVENT_MOVE)) {
            if (fout_.is_open()) fout_ << std::endl;
        }

    }
    // clear events
    if (clear_events) world_state->clearEvents();
    
    return amod::SUCCESS;
}
    


} /* namespace AMOD */
