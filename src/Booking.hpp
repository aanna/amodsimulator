/*
 * Booking.h
 *
 *  Created on: Mar 23, 2015
 *      Author: haroldsoh
 */

#ifndef Booking_H_
#define Booking_H_

#include "Types.hpp"

namespace amod {
    
    struct Booking {
    public:
        Booking(int id = 0, int vehicle_id = 0, int customer_id = 0, Position dest_position = Position()): veh_id(vehicle_id),
        cust_id(customer_id), destination(dest_position) {};
        virtual ~Booking() {};
        
        int id;
        int veh_id;
        int cust_id;
        
        double booking_time;
        double dispatch_time;
        double pickup_time;
        double dropoff_time;
        
        Position destination;
        
    };
    
} /* namespace AMOD */

#endif /* Booking_H_ */
