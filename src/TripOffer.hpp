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
			std::string mode_ = "amod",
			double price_ = 0,
			int waitTime_ = 0,
			int arrivalTime_ = 0,
			int id_ = 0):
				mode(mode_),
				price(price_),
				waitTime(waitTime_),
				arrivalTime(arrivalTime_),
				vehId(id_){}

	/**
	 * Destructor
	 */
	virtual ~TripOffer() {}

	/// which mode is offered (amod or shared amod s-amod)
	std::string mode;

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
