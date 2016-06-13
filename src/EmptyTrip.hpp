/*
 * EmptyTrip.hpp
 *
 *  Created on: 4 Apr, 2016
 *      Author: kasia
 */

/*
 * EmptyBookings.hpp
 *
 *  Created on: 4 Apr, 2016
 *      Author: kasia
 */


#include "Types.hpp"
#include <istream>


namespace amod {

struct EmptyTrip {
public:

	/**
	 * Constructor
	 * @param from - from node id
	 * @param to - to node id
	 * @param rebTime - when the vehicles should be sent
	 * @param count - how many vehicles should be sent
	 */
	EmptyTrip(
			int from_ = 0,
			int to_ = 0,
			int rebTime = 0,
			int id_ = 0):
				from(from_),
				to(to_),
				rebalancingTime(rebTime),
				tripId(id_){}

	/**
	 * Destructor
	 */
	virtual ~EmptyTrip() {}

	/// Source node id (should be changed to Positions)
	int from;

	/// Desitination node id
	int to;

	// rebalancing time in seconds
	int rebalancingTime;

	// id of the trip
	int tripId;

};
}
