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
        
        int id;         // id of booking (valid bookings have > 0)
        int veh_id;     // veh_id (valid veh_ids > 0)
        int cust_id;    // cust_id (valid cust_ids > 0)
        
        Position destination;   // destination position
        double booking_time;    // booking time (in seconds)
        
        // the following are mainly for logging purposes (optional)
        double dispatch_time;   // dispatch time (in seconds)
        double pickup_time;     // pickup time (in seconds)
        double dropoff_time;    // dropoff time (in seconds)
        
    };
    
} /* namespace AMOD */

#endif /* Booking_H_ */
