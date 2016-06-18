/*
 * TripOffer.hpp
 *
 *  Created on: 17 Jun, 2016
 *      Author: kasia
 */

#ifndef TRIPOFFER_HPP_
#define TRIPOFFER_HPP_


namespace amod {

struct TripOffer {
public:

	/**
	 * Constructor
	 */

	TripOffer(
			int offer_id_ = 0,
			std::string mode_ = "amod",
			int booking_id_ = 0,
			double price_ = 0,
			int waitTime_ = 0,
			int arrivalTime_ = 0,
			int veh_id_ = 0):
				offer_id(offer_id_),
				mode(mode_),
				booking_id(booking_id_),
				price(price_),
				waitTime(waitTime_),
				arrivalTime(arrivalTime_),
				vehId(veh_id_){}

	/**
	 * Destructor
	 */
	virtual ~TripOffer() {}

	// offer id
	int offer_id;

	/// which mode is offered (amod or shared amod s-amod)
	std::string mode;

	// booking id
	int booking_id;

	/// how much does the offer cost (estimated)
	// price = dist in meters * price 0.22 cents per 400m
	double price;

	/// waiting time to be picked up (estimated and not longer than)
	int waitTime;

	// estimated arrival time in destination
	int arrivalTime;

	// vehicle id (vehicle offered to serve the trip)
	int vehId;

};
}


#endif /* TRIPOFFER_HPP_ */
