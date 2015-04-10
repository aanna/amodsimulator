//
//  ManagerBasic.h
//  AMODBase
//
//  Created by Harold Soh on 29/3/15.
//  Copyright (c) 2015 Harold Soh. All rights reserved.
//

#ifndef __AMODBase__ManagerBasic__
#define __AMODBase__ManagerBasic__

#include "Types.hpp"
#include "Manager.hpp"
#include "Booking.hpp"
#include "World.hpp"
#include "Event.hpp"

#include <map>
#include <iostream>
#include <fstream>

namespace amod {
    class ManagerBasic : public Manager {
    public:
        ManagerBasic();
        virtual ~ManagerBasic();
        
        // init
        // initializes the manager with the World world_state
        // if the call is successful, it returns amod::SUCESSS. Otherwise, it returns
        // one of the amod::ReturnCode error codes.
        virtual amod::ReturnCode init(World *world_state);
        
        // update
        // updates the manager with the World world_state. This manager is a simple
        // demonstration of the manager and is a simple queue (FIFO) manager which dispatches
        // the closest FREE or PARKED vehicle. Bookings are responded to by booking time.
        // If there are no available vehicles, the booking remains in the booking queue.
        // if the call is successful, it returns amod::SUCESSS. Otherwise, it returns
        // one of the amod::ReturnCode error codes.
        virtual amod::ReturnCode update(World *world_state);
    
        // loadBookings
        // loads bookings that the manager should respond to.
        // if the call is successful, it returns amod::SUCESSS. Otherwise, it returns
        // one of the amod::ReturnCode error codes.
        virtual amod::ReturnCode loadBookings(const std::vector<Booking> &bookings);
        
        // loadBookingsFromFile
        // loads bookings from a file specified by filename that the manager should respond to.
        // if the call is successful, it returns amod::SUCESSS. Otherwise, it returns
        // one of the amod::ReturnCode error codes.
        virtual amod::ReturnCode loadBookingsFromFile(const std::string filename);
        
    private:
        std::multimap<double, Booking> bookings_;
        
        std::ofstream out;
        int num_avail_veh_; // tracks number of available vehicles
    };
}

#endif /* defined(__AMODBase__ManagerBasic__) */
