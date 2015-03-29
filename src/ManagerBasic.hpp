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
        
        virtual amod::ReturnCode init(World *world_state);
        virtual amod::ReturnCode update(World *world_state);
    
        virtual amod::ReturnCode loadBookings(const std::vector<Booking> &bookings);
        virtual amod::ReturnCode loadBookingsFromFile(const std::string filename);
        
    private:
        std::multimap<double, Booking> bookings_;
        
    };
}

#endif /* defined(__AMODBase__ManagerBasic__) */
