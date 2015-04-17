/*
 * Event.hpp
 *
 *  Created on: Mar 27, 2015
 *      Author: haroldsoh
 */

#ifndef EVENT_HPP_
#define EVENT_HPP_

namespace amod {

enum EventType {
	EVENT_DISPATCH,
	EVENT_MOVE,
	EVENT_ARRIVAL,
	EVENT_PICKUP,
	EVENT_DROPOFF,
	EVENT_LOCATION_VEHS_SIZE_CHANGE,
	EVENT_LOCATION_CUSTS_SIZE_CHANGE,
	EVENT_BOOKING_CANNOT_BE_SERVICED,
	EVENT_TELEPORT,
	EVENT_TELEPORT_ARRIVAL,
};

struct Event {
	Event(EventType event_type, long long event_id, std::string event_name, double event_time, std::vector<int> ent_ids) :
		type(event_type), id(event_id), name(event_name), t(event_time), entity_ids(ent_ids) {};
	Event(): id(0) {};
	~Event() {};

	EventType type;
	long long id; //specific event identifier
	std::string name;
	double t;
	std::vector<int> entity_ids;

};

} /* namespace amod */

#endif /* EVENT_HPP_ */
