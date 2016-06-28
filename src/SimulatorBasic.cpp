/*
 * SimulatorBasic.cpp
 *
 *  Created on: Mar 27, 2015
 *      Author: haroldsoh
 */

#include "SimulatorBasic.hpp"

namespace amod {

SimulatorBasic::SimulatorBasic(double resolution):
        																				resolution_(resolution), event_id_(0),
        																				using_locations_(false)
{
	// just return
}

SimulatorBasic::~SimulatorBasic() {
	// just return
}

amod::ReturnCode  SimulatorBasic::init(amod::World *world_state) {

	// initialize the locations
	if (world_state->getNumLocations() > 0) {
		std::vector<Location> locs;
		world_state->getLocations(&locs);
		loc_tree_.build(locs);
		using_locations_ = true;

		// set the customer positions to be in valid locations
		std::unordered_map<int, Customer>::const_iterator bitr, eitr;
		world_state->getCustomers(&bitr, &eitr);
		for (auto itr = bitr; itr!=eitr; ++itr) {

			Customer * pcust = world_state->getCustomerPtr(itr->first);
			Location loc = loc_tree_.findNN({pcust->getPosition().x, pcust->getPosition().y});
			pcust->setPosition(loc.getPosition());
			pcust->setLocationId(loc.getId());

			// add this customer to the location
			Location *ploc = world_state->getLocationPtr(loc.getId());
			ploc->addCustomerId(pcust->getId());
		}

		// set the vehicle positions to be in valid locations
		std::unordered_map<int, Vehicle>::const_iterator vbitr, veitr;
		world_state->getVehicles(&vbitr, &veitr);
		for (auto itr = vbitr; itr!=veitr; ++itr) {

			Vehicle * pveh = world_state->getVehiclePtr(itr->first);
			Location loc = loc_tree_.findNN({pveh->getPosition().x, pveh->getPosition().y});
			pveh->setPosition(loc.getPosition());
			pveh->setLocationId(loc.getId());

			// add this vehicle to the location
			Location *ploc = world_state->getLocationPtr(loc.getId());
			ploc->addVehicleId(pveh->getId());
		}
	}

	// copy over the state
	state_ = *world_state;
	return amod::SUCCESS;
}

amod::ReturnCode  SimulatorBasic::update(amod::World *world_state) {
	// updates the world state
	state_.setCurrentTime( world_state->getCurrentTime() + resolution_ );

	// simulate the vehicles
	simulateVehicles(world_state);

	// simulate the pickups
	simulatePickups(world_state);

	// simulate the dropoffs
	simulateDropoffs(world_state);

	// simulate the teleports
	simulateTeleports(world_state);

	// simulate the customers (this is currently a placeholder for future expansion)
	simulateCustomers(world_state);

	world_state->setCurrentTime( state_.getCurrentTime() );

	return amod::SUCCESS;
}

amod::ReturnCode SimulatorBasic::dispatchVehicle(amod::World *world_state,
		int veh_id,
		const amod::Position &to,
		amod::VehicleStatus start_status,
		amod::VehicleStatus end_status
) {
	return dispatchVehicle(world_state, veh_id, to, start_status, end_status, 0);
}

amod::ReturnCode SimulatorBasic::pickupCustomer(amod::World *world_state,
		int veh_id, int cust_id,
		amod::VehicleStatus start_status,
		amod::VehicleStatus end_status
) {
	return pickupCustomer(world_state, veh_id, cust_id, start_status, end_status, 0);
}

amod::ReturnCode SimulatorBasic::dropoffCustomer(amod::World *world_state,
		int veh_id, int cust_id,
		amod::VehicleStatus start_status,
		amod::VehicleStatus end_status
) {
	return dropoffCustomer(world_state, veh_id, cust_id, start_status, end_status, 0);
}


// internal helper functions

amod::ReturnCode SimulatorBasic::dispatchVehicle(amod::World *world_state,
		int veh_id,
		const amod::Position &to,
		amod::VehicleStatus start_status,
		amod::VehicleStatus end_status,
		int booking_id) {
	// check if vehicle already exists in the dispatch map
	auto it = dispatches_.find(veh_id);
	if (it != dispatches_.end()) {
		if (getVerbose()) std::cout << "Vehicle ID found (it is currently being dispatched!): " << veh_id << std::endl;
		return amod::VEHICLE_CANNOT_BE_DISPATCHED;
	}

	// create a new dispatch
	Vehicle veh = state_.getVehicle(veh_id);
	if (!veh.getId()) {
		if (getVerbose()) std::cout << "Can't get vehicle from world_state" << std::endl;
		return amod::CANNOT_GET_VEHICLE;
	}

	Dispatch dp;
	dp.booking_id = booking_id;
	dp.veh_id = veh_id;
	dp.from = veh.getPosition();
	if (using_locations_) {
		Location des = loc_tree_.findNN({to.x, to.y});
		dp.to = des.getPosition(); // find the closest location to be the destination position
		dp.to_loc_id = des.getId();
	} else {
		dp.to = to;
	}
	if (using_locations_) {
		Location des = loc_tree_.findNN({dp.from.x, dp.from.y});
		dp.from = des.getPosition(); // find the closest location to be the destination position
		dp.from_loc_id = des.getId();
	}

	dp.curr = dp.from;
	double dx = dp.to.x - dp.from.x;
	double dy = dp.to.y - dp.from.y;
	double rd = sqrt( dx*dx + dy*dy);
	if (rd == 0) {
		dp.grad = Position(1.0, 1.0); // just travel someplace (same location, will arrive at next timestep)
	} else {
		dp.grad = Position( dx/rd , dy/rd); // travel in the direction of the destination
	}
	dp.veh_end_status = end_status;

	//        if (dp.from == dp.to) {
	//            return amod::SOURCE_EQUALS_DESTINATION;
	//        }
	//

	// add it to the dispatch qmap
	dispatches_[veh_id] = dp;

	// update the vehicle status
	veh.setStatus(start_status);
	world_state->setVehicle(veh);
	state_.setVehicle(veh);
	if (booking_id) {
		//get the booking customer
		Customer cust = world_state->getCustomer(bookings_[booking_id].cust_id);

		if (!cust.isInVehicle()) {
			//if (getVerbose()) std::cout << "Set customer waiting for pickup" << std::endl;
			cust.setStatus(CustomerStatus::WAITING_FOR_PICKUP);
			world_state->setCustomer(cust);
			state_.setCustomer(cust);
		} else {
			//if (getVerbose()) std::cout << "Set customer in vehicle" << std::endl;
			cust.setStatus(CustomerStatus::IN_VEHICLE);
			world_state->setCustomer(cust);
			state_.setCustomer(cust);
		}
	}

	// location specific changes
	if (using_locations_) {
		Location * ploc = world_state->getLocationPtr(dp.from_loc_id);
		ploc->removeVehicleId(veh.getId());
		veh.setLocationId(0);
		world_state->setVehicle(veh);

		// trigger event
		Event ev(amod::EVENT_LOCATION_VEHS_SIZE_CHANGE, ++event_id_,
				"LocationVehSizeChange", state_.getCurrentTime(),
				{dp.from_loc_id});
		world_state->addEvent(ev);

		Customer cust;
		if (booking_id) {
			cust = world_state->getCustomer(bookings_[booking_id].cust_id);

			if (cust.isInVehicle()) {
				ploc->removeCustomerId(cust.getId());
				cust.setLocationId(0);
				world_state->setCustomer(cust);
				// trigger event
				Event ev(amod::EVENT_LOCATION_CUSTS_SIZE_CHANGE, ++event_id_,
						"LocationCustSizeChange", state_.getCurrentTime(),
						{dp.from_loc_id});
				world_state->addEvent(ev);

			}
		}

		// update internal state
		ploc = state_.getLocationPtr(dp.from_loc_id);
		ploc->removeVehicleId(veh.getId());
		if (booking_id && cust.isInVehicle()) {
			ploc->removeCustomerId(cust.getId());
		}
	}


	// create a dispatch event
	std::vector<int> entity_ids = {veh_id, booking_id};
	Event ev(amod::EVENT_DISPATCH, ++event_id_, "VehicleDispatch", state_.getCurrentTime(), entity_ids);
	world_state->addEvent(ev);

	return amod::SUCCESS;
}

amod::ReturnCode SimulatorBasic::dispatchSharedVehicle(amod::World *world_state,
		int vehId, int booking1_id, int booking2_id,
		const amod::Position &firstPickup, const amod::Position &secondPickup,
		const amod::Position &firstDropoff, const amod::Position &secondDropoff,
		amod::VehicleStatus start_status,
		amod::VehicleStatus end_status) {


	// check if vehicle already exists in the dispatch map
	auto it = dispatches_.find(vehId);
	if (it != dispatches_.end()) {
		if (getVerbose()) std::cout << "Vehicle ID found (it is currently being dispatched!): " << vehId << std::endl;
		return amod::VEHICLE_CANNOT_BE_DISPATCHED;
	}

	// create a new dispatch
	Vehicle veh = state_.getVehicle(vehId);
	if (!veh.getId()) {
		if (getVerbose()) std::cout << "Can't get vehicle from world_state" << std::endl;
		return amod::CANNOT_GET_VEHICLE;
	}

	Dispatch dp;
	dp.booking_id = booking1_id;
	dp.veh_id = vehId;
	dp.from = veh.getPosition();

	return amod::SUCCESS;
}

amod::ReturnCode SimulatorBasic::pickupCustomer(amod::World *world_state,
		int veh_id, int cust_id,
		amod::VehicleStatus start_status,
		amod::VehicleStatus end_status,
		int booking_id) {
	// check that vehicle is at the same location as the customer
	Vehicle veh = state_.getVehicle(veh_id);
	if (!veh.getId()) {
		return amod::CANNOT_GET_VEHICLE;
	}

	Customer cust = state_.getCustomer(cust_id);
	if (!cust_id) {
		return amod::CANNOT_GET_CUSTOMER;
	}

	//        if (getDistance(veh.getPosition(), cust.getPosition()) > 1e-3 ) {
	//            return amod::VEHICLE_NOT_AT_CUSTOMER_LOCATION;
	//        }

	// add a pickup to simulate
	double pickup_time = state_.getCurrentTime() + genRandTruncNormal(pickup_params_);
	//if (getVerbose()) std::cout << "Future Pickup time : " << pickup_time << std::endl;

	int loc_id = 0;
	if (using_locations_) {
		// get the pickup location
		Location pickup_loc = loc_tree_.findNN({cust.getPosition().x, cust.getPosition().y});
		loc_id = pickup_loc.getId();
	}
	Pickup p{booking_id, veh_id, cust_id, loc_id, pickup_time, end_status};
	pickups_.emplace(pickup_time, p);

	// set the vehicle's start status
	veh.setStatus(start_status);
	cust.setStatus(CustomerStatus::WAITING_FOR_PICKUP);
	world_state->setVehicle(veh);
	world_state->setCustomer(cust);

	state_.setVehicle(veh);
	state_.setCustomer(cust);

	return amod::SUCCESS;
}

amod::ReturnCode SimulatorBasic::dropoffCustomer(amod::World *world_state,
		int veh_id, int cust_id,
		amod::VehicleStatus start_status,
		amod::VehicleStatus end_status,
		int booking_id) {
	// check that vehicle is at the same location as the customer
	Vehicle veh = world_state->getVehicle(veh_id);
	if (!veh.getId()) {
		return amod::CANNOT_GET_VEHICLE;
	}

	if (veh.getCustomerId() != cust_id) {
		return amod::VEHICLE_DOES_NOT_HAVE_CUSTOMER;
	}

	Customer cust = state_.getCustomer(cust_id);
	if (!cust_id) {
		return amod::CANNOT_GET_CUSTOMER;
	}

	//        if (getDistance(veh.getPosition(), cust.getPosition()) > 1e-3 ) {
	//            return amod::VEHICLE_NOT_AT_CUSTOMER_LOCATION;
	//        }

	// add a dropoff to simulate
	double dropoff_time = state_.getCurrentTime() + genRandTruncNormal(dropoff_params_);

	int loc_id = 0;
	if (using_locations_) {
		// get the dropoff location
		Location dropoff_loc = loc_tree_.findNN({cust.getPosition().x, cust.getPosition().y});
		loc_id = dropoff_loc.getId();
	}

	//if (getVerbose()) std::cout << "Future Dropoff time : " << dropoff_time << std::endl;
	Dropoff doff{booking_id, veh_id, cust_id, loc_id, dropoff_time, end_status};
	dropoffs_.emplace(dropoff_time, doff);

	// sets the status of the vehicle
	veh.setStatus(start_status);
	world_state->setVehicle(veh);
	cust.setStatus(CustomerStatus::WAITING_FOR_DROPOFF);
	world_state->setCustomer(cust);

	state_.setVehicle(veh);
	state_.setCustomer(cust);

	// return success
	return amod::SUCCESS;
}

amod::ReturnCode SimulatorBasic::teleportCustomer(amod::World *world_state,
		int cust_id,
		const amod::Position &to,
		amod::CustomerStatus cust_start_status,
		amod::CustomerStatus cust_end_status
) {
	Customer *cust = state_.getCustomerPtr(cust_id);
	if (!cust) {
		return amod::CANNOT_GET_CUSTOMER;
	}

	if (cust->getStatus() != CustomerStatus::FREE) {
		return amod::CUSTOMER_IS_NOT_FREE;
	}

	// set a teleporation arrival time
	double teleport_time = state_.getCurrentTime() + genRandTruncNormal(teleport_params_);

	int from_loc_id = 0;

	if (using_locations_) {
		// get the teleport location
		Location teleport_loc = loc_tree_.findNN({cust->getPosition().x, cust->getPosition().y});
		from_loc_id = teleport_loc.getId();
	}

	int to_loc_id = 0;
	if (using_locations_) {
		// get the teleport location
		Location teleport_loc = loc_tree_.findNN({to.x, to.y});
		to_loc_id = teleport_loc.getId();
	}



	Teleport tport{cust_id, to_loc_id, teleport_time, cust_end_status};

	teleports_.emplace(teleport_time, tport);

	// sets the status of the vehicle
	cust->setStatus(cust_start_status);
	cust->setLocationId(0);

	// create a teleportation event
	std::vector<int> entity_ids = {cust_id, from_loc_id};
	Event ev(amod::EVENT_TELEPORT, ++event_id_, "CustomerTeleport", state_.getCurrentTime(), entity_ids);
	world_state->addEvent(ev);

	// adjust locations
	// location specific changes

	if (using_locations_) {
		Location * ploc = world_state->getLocationPtr(from_loc_id);

		ploc->removeCustomerId(cust_id);
		// trigger event
		Event ev(amod::EVENT_LOCATION_CUSTS_SIZE_CHANGE, ++event_id_,
				"LocationCustSizeChange", state_.getCurrentTime(),
				{from_loc_id});
		world_state->addEvent(ev);

		// update internal state;
		ploc = state_.getLocationPtr(from_loc_id);
		ploc->removeCustomerId(cust_id);
	}

	// set the internal state
	state_.getCustomerPtr(cust_id)->setStatus(cust_start_status);



	return amod::SUCCESS;
}


void SimulatorBasic::setCustomerStatus(amod::World *world_state, int cust_id, CustomerStatus status) {
	Customer cust = world_state->getCustomer(cust_id);
	cust.setStatus(status);
	world_state->setCustomer(cust);
}


amod::ReturnCode SimulatorBasic::serviceBooking(amod::World *world_state, const amod::Booking &booking) {
	// add booking to internal structure
	bookings_[booking.id] = booking;

	amod::ReturnCode rc;

	// make sure the customer is valid
	amod::Customer cust = world_state->getCustomer(booking.cust_id);
	if (!(cust.getStatus() == amod::CustomerStatus::FREE || cust.getStatus() == amod::CustomerStatus::WAITING_FOR_ASSIGNMENT)) {
		rc = amod::ReturnCode::CUSTOMER_IS_NOT_FREE;
	} else {

		// dispatch the vehicle to the customer's position
		amod::Position cust_pos = cust.getPosition();

		double dist_to_dropoff = getDrivingDistance(cust_pos, booking.destination);
		if (dist_to_dropoff < 0) {
			rc = amod::NO_PATH_TO_DESTINATION;
		} else {

			// dispatch the vehicle
			rc = dispatchVehicle(world_state, booking.veh_id, cust_pos,
					amod::VehicleStatus::MOVING_TO_PICKUP, amod::VehicleStatus::HIRED, booking.id);
		}
	}

	// if the return code was not successful, then we raise an event that the booking could not be serviced
	if (rc!= amod::SUCCESS) {
		// raise an event that this booking was dropped
		std::vector<int> entities = {booking.id, booking.cust_id};
		amod::Event ev(amod::EVENT_BOOKING_CANNOT_BE_SERVICED, ++event_id_, "BookingDropped", world_state->getCurrentTime(), entities);
		world_state->addEvent(ev);
	}
	return rc;
}


amod::ReturnCode SimulatorBasic::serviceSharedBookings(amod::World *world_state, const amod::Booking &booking1st,
		const amod::Booking &booking2nd, int &vehId1, int &vehId2) {

	amod::ReturnCode rc = amod::FAILED;

	// make sure the customer is valid
	amod::Customer *cust1 = world_state->getCustomerPtr(booking1st.cust_id);
	amod::Customer *cust2 = world_state->getCustomerPtr(booking2nd.cust_id);

	if (!(cust1->getStatus() == amod::CustomerStatus::FREE || cust1->getStatus() == amod::CustomerStatus::WAITING_FOR_ASSIGNMENT)
			|| !(cust2->getStatus() == amod::CustomerStatus::FREE || cust2->getStatus() == amod::CustomerStatus::WAITING_FOR_ASSIGNMENT)) {
		rc = amod::ReturnCode::CUSTOMER_IS_NOT_FREE;
	} else {

		// dispatch the vehicle to the customers' position
		Vehicle *veh1 = world_state->getVehiclePtr(vehId1);
		Vehicle *veh2 = world_state->getVehiclePtr(vehId2);
		amod::Position cust1_pos = cust1->getPosition();
		amod::Position cust2_pos = cust2->getPosition();
		amod::Position cust1_dropOff = booking1st.destination;
		amod::Position cust2_dropOff = booking2nd.destination;

		// schedule the trip order
		if (vehId1 == vehId2) {
			vehId2 = -1;
			double P1P2D1D2 = getDrivingDistance(veh1->getPosition(), cust1->getPosition()) +
					getDrivingDistance(cust1->getPosition(), cust2->getPosition()) +
					getDrivingDistance(cust2->getPosition(), booking1st.destination) +
					getDrivingDistance(booking1st.destination, booking2nd.destination);

			double P1P2D2D1 = getDrivingDistance(veh1->getPosition(), cust1->getPosition()) +
					getDrivingDistance(cust1->getPosition(), cust2->getPosition()) +
					getDrivingDistance(cust2->getPosition(), booking2nd.destination) +
					getDrivingDistance(booking2nd.destination, booking1st.destination);

			double P2P1D1D2 = getDrivingDistance(veh1->getPosition(), cust2->getPosition()) +
					getDrivingDistance(cust2->getPosition(), cust1->getPosition()) +
					getDrivingDistance(cust1->getPosition(), booking1st.destination) +
					getDrivingDistance(booking1st.destination, booking2nd.destination);

			double P2P1D2D1 = getDrivingDistance(veh1->getPosition(), cust2->getPosition()) +
					getDrivingDistance(cust2->getPosition(), cust1->getPosition()) +
					getDrivingDistance(cust1->getPosition(), booking2nd.destination) +
					getDrivingDistance(booking2nd.destination, booking1st.destination);

			//if (dist_to_dropoff < 0) {
			//rc = amod::NO_PATH_TO_DESTINATION;
			//}

			double shortestTrip = std::min(std::min(P1P2D1D2, P1P2D2D1), std::min(P2P1D1D2, P2P1D2D1));

			if (shortestTrip == P1P2D1D2) {
				// pick up cust1, later to cust2, drop off 1, drop off 2
				rc = dispatchSharedVehicle(world_state, vehId1, booking1st.id, booking2nd.id,
						cust1_pos, cust2_pos, cust1_dropOff, cust2_dropOff,
						amod::VehicleStatus::MOVING_TO_FIRST_PICKUP, amod::VehicleStatus::HIRED);

			} else if (shortestTrip == P1P2D2D1) {
				// pick up cust1, later to cust2, drop off 2, drop off 1
				rc = dispatchSharedVehicle(world_state, vehId1, booking1st.id, booking2nd.id,
						cust1_pos, cust2_pos, cust2_dropOff, cust1_dropOff,
						amod::VehicleStatus::MOVING_TO_FIRST_PICKUP, amod::VehicleStatus::HIRED);

			} else if (shortestTrip == P2P1D1D2) {
				// pick up cust2, later to cust1, drop off 1, drop off 2
				rc = dispatchSharedVehicle(world_state, vehId1, booking1st.id, booking2nd.id,
						cust2_pos, cust1_pos, cust1_dropOff, cust2_dropOff,
						amod::VehicleStatus::MOVING_TO_FIRST_PICKUP, amod::VehicleStatus::HIRED);

			} else if (shortestTrip == P2P1D2D1) {
				// pick up cust2, later to cust1, drop off 2, drop off 1
				rc = dispatchSharedVehicle(world_state, vehId1, booking1st.id, booking2nd.id,
						cust2_pos, cust1_pos, cust2_dropOff, cust1_dropOff,
						amod::VehicleStatus::MOVING_TO_FIRST_PICKUP, amod::VehicleStatus::HIRED);

			}
		} else {
			// find which vehicle is better to send
			// do above for veh 1 and for veh 2 separately
			double P1P2D1D2 = getDrivingDistance(veh1->getPosition(), cust1->getPosition()) +
					getDrivingDistance(cust1->getPosition(), cust2->getPosition()) +
					getDrivingDistance(cust2->getPosition(), booking1st.destination) +
					getDrivingDistance(booking1st.destination, booking2nd.destination);

			double P1P2D2D1 = getDrivingDistance(veh1->getPosition(), cust1->getPosition()) +
					getDrivingDistance(cust1->getPosition(), cust2->getPosition()) +
					getDrivingDistance(cust2->getPosition(), booking2nd.destination) +
					getDrivingDistance(booking2nd.destination, booking1st.destination);

			double P2P1D1D2 = getDrivingDistance(veh1->getPosition(), cust2->getPosition()) +
					getDrivingDistance(cust2->getPosition(), cust1->getPosition()) +
					getDrivingDistance(cust1->getPosition(), booking1st.destination) +
					getDrivingDistance(booking1st.destination, booking2nd.destination);

			double P2P1D2D1 = getDrivingDistance(veh1->getPosition(), cust2->getPosition()) +
					getDrivingDistance(cust2->getPosition(), cust1->getPosition()) +
					getDrivingDistance(cust1->getPosition(), booking2nd.destination) +
					getDrivingDistance(booking2nd.destination, booking1st.destination);

			double shortestTripVehA = std::min(std::min(P1P2D1D2, P1P2D2D1), std::min(P2P1D1D2, P2P1D2D1));

			// second vehicle
			double P1P2D1D2_ = getDrivingDistance(veh2->getPosition(), cust1->getPosition()) +
					getDrivingDistance(cust1->getPosition(), cust2->getPosition()) +
					getDrivingDistance(cust2->getPosition(), booking1st.destination) +
					getDrivingDistance(booking1st.destination, booking2nd.destination);

			double P1P2D2D1_ = getDrivingDistance(veh2->getPosition(), cust1->getPosition()) +
					getDrivingDistance(cust1->getPosition(), cust2->getPosition()) +
					getDrivingDistance(cust2->getPosition(), booking2nd.destination) +
					getDrivingDistance(booking2nd.destination, booking1st.destination);

			double P2P1D1D2_ = getDrivingDistance(veh2->getPosition(), cust2->getPosition()) +
					getDrivingDistance(cust2->getPosition(), cust1->getPosition()) +
					getDrivingDistance(cust1->getPosition(), booking1st.destination) +
					getDrivingDistance(booking1st.destination, booking2nd.destination);

			double P2P1D2D1_ = getDrivingDistance(veh2->getPosition(), cust2->getPosition()) +
					getDrivingDistance(cust2->getPosition(), cust1->getPosition()) +
					getDrivingDistance(cust1->getPosition(), booking2nd.destination) +
					getDrivingDistance(booking2nd.destination, booking1st.destination);

			double shortestTripVehB = std::min(std::min(P1P2D1D2_, P1P2D2D1_), std::min(P2P1D1D2_, P2P1D2D1_));

			double shortestTrip = std::min(shortestTripVehB, shortestTripVehA);

			if (shortestTrip == shortestTripVehA) {
				vehId2 = -1;
				if (shortestTrip == P1P2D1D2) {
					// pick up cust1, later to cust2, drop off 1, drop off 2
					rc = dispatchSharedVehicle(world_state, vehId1, booking1st.id, booking2nd.id,
							cust1_pos, cust2_pos, cust1_dropOff, cust2_dropOff,
							amod::VehicleStatus::MOVING_TO_FIRST_PICKUP, amod::VehicleStatus::HIRED);

				} else if (shortestTrip == P1P2D2D1) {
					// pick up cust1, later to cust2, drop off 2, drop off 1
					rc = dispatchSharedVehicle(world_state, vehId1, booking1st.id, booking2nd.id,
							cust1_pos, cust2_pos, cust2_dropOff, cust1_dropOff,
							amod::VehicleStatus::MOVING_TO_FIRST_PICKUP, amod::VehicleStatus::HIRED);

				} else if (shortestTrip == P2P1D1D2) {
					// pick up cust2, later to cust1, drop off 1, drop off 2
					rc = dispatchSharedVehicle(world_state, vehId1, booking1st.id, booking2nd.id,
							cust2_pos, cust1_pos, cust1_dropOff, cust2_dropOff,
							amod::VehicleStatus::MOVING_TO_FIRST_PICKUP, amod::VehicleStatus::HIRED);

				} else if (shortestTrip == P2P1D2D1) {
					// pick up cust2, later to cust1, drop off 2, drop off 1
					rc = dispatchSharedVehicle(world_state, vehId1, booking1st.id, booking2nd.id,
							cust2_pos, cust1_pos, cust2_dropOff, cust1_dropOff,
							amod::VehicleStatus::MOVING_TO_FIRST_PICKUP, amod::VehicleStatus::HIRED);

				}
			} else {
				// shortestTrip == shortestTripVehB
				vehId1 = -1;
				if (shortestTrip == P1P2D1D2_) {
					// pick up cust1, later to cust2, drop off 1, drop off 2
					rc = dispatchSharedVehicle(world_state, vehId2, booking1st.id, booking2nd.id,
							cust1_pos, cust2_pos, cust1_dropOff, cust2_dropOff,
							amod::VehicleStatus::MOVING_TO_FIRST_PICKUP, amod::VehicleStatus::HIRED);

				} else if (shortestTrip == P1P2D2D1_) {
					// pick up cust1, later to cust2, drop off 2, drop off 1
					rc = dispatchSharedVehicle(world_state, vehId2, booking1st.id, booking2nd.id,
							cust1_pos, cust2_pos, cust2_dropOff, cust1_dropOff,
							amod::VehicleStatus::MOVING_TO_FIRST_PICKUP, amod::VehicleStatus::HIRED);

				} else if (shortestTrip == P2P1D1D2_) {
					// pick up cust2, later to cust1, drop off 1, drop off 2
					rc = dispatchSharedVehicle(world_state, vehId2, booking1st.id, booking2nd.id,
							cust2_pos, cust1_pos, cust1_dropOff, cust2_dropOff,
							amod::VehicleStatus::MOVING_TO_FIRST_PICKUP, amod::VehicleStatus::HIRED);

				} else if (shortestTrip == P2P1D2D1_) {
					// pick up cust2, later to cust1, drop off 2, drop off 1
					rc = dispatchSharedVehicle(world_state, vehId2, booking1st.id, booking2nd.id,
							cust2_pos, cust1_pos, cust2_dropOff, cust1_dropOff,
							amod::VehicleStatus::MOVING_TO_FIRST_PICKUP, amod::VehicleStatus::HIRED);
				}
			}
		}
	}

	// if the return code was not successful, then we raise an event that the booking could not be serviced
	if (rc!= amod::SUCCESS) {
		// raise an event that this booking was dropped
		std::vector<int> entities = {booking1st.id, booking1st.cust_id, booking2nd.id, booking2nd.cust_id};
		amod::Event ev(amod::EVENT_SHARED_BOOKING_CANNOT_BE_SERVICED, ++event_id_, "SharedBookingDropped", world_state->getCurrentTime(), entities);
		world_state->addEvent(ev);
	}

	return amod::SUCCESS;
}

double SimulatorBasic::getDrivingDistance(const amod::Position &from, const amod::Position &to) {
	return getDistance(from, to);
}

double SimulatorBasic::getDrivingDistance(int from_loc_id, int to_loc_id) {
	if (from_loc_id && to_loc_id) {
		return getDistance(state_.getLocation(from_loc_id).getPosition(), state_.getLocation(to_loc_id).getPosition());
	}
	return -1.0;
}

double SimulatorBasic::getDistance(const amod::Position &from, const amod::Position &to) {
	return sqrt(pow( from.x - to.x ,2.0) + pow( from.y - to.y ,2.0));
}

void SimulatorBasic::simulateVehicles(amod::World *world_state) {
	// for all vehicles in dispatches_
	for (auto it = dispatches_.begin(); it != dispatches_.end(); ) {
		// move vehicle
		Vehicle veh = world_state->getVehicle(it->second.veh_id);

		double dist = genRandTruncNormal(speed_params_);

		double x_dist = dist*(it->second.grad.x)*resolution_;
		double y_dist = dist*(it->second.grad.y)*resolution_;

		it->second.curr.x += x_dist;
		it->second.curr.y += y_dist;

		veh.setPosition(it->second.curr);

		// set associated customer id and update
		int cust_id = veh.getCustomerId();
		Customer cust = Customer();
		if (cust_id) cust = world_state->getCustomer(cust_id);
		if (cust_id && cust.isInVehicle()) {
			cust.setPosition(it->second.curr);
		}

		// if vehicle (specified by the dispatch) has arrived
		if (hasArrived(it->second)) {
			// vehicle has arrived
			if (getVerbose()) std::cout << veh.getId() << " has arrived at " << it->second.to.x << " " << it->second.to.y <<
					" at time "  << world_state->getCurrentTime() << std::endl;
			it->second.curr = it->second.to;
			veh.setPosition(it->second.curr);

			// set associated customer position to the final destination
			if (cust_id && cust.isInVehicle()) {
				cust.setPosition(it->second.curr);
			}

			// trigger arrival event
			int bid = it->second.booking_id;
			std::vector<int> entities = {veh.getId()};

			if (cust_id && cust.isInVehicle()) {
				entities.push_back(cust_id);
			}

			if (bid) {
				entities.push_back(bid);
			}


			Event ev(amod::EVENT_ARRIVAL, ++event_id_, "VehicleArrival", state_.getCurrentTime(), entities);
			world_state->addEvent(ev);

			// set the vehicle status
			veh.setStatus(it->second.veh_end_status);


			// update the location to indicate the vehicle is here.
			if (!bid && ((veh.getStatus() == amod::Vehicle::FREE) ||
					(veh.getStatus() == amod::Vehicle::PARKED))
			) { //only if the vehicle is free
				int veh_id = veh.getId();
				Dispatch dp = dispatches_[veh_id];
				int loc_id = dp.to_loc_id;
				amod::Location *ploc = world_state->getLocationPtr(loc_id);
				if (ploc == nullptr) {
					if (verbose_) std::cout << "Location " << loc_id << " is nullptr!" << std::endl;
					throw std::runtime_error("AMODSimulatorSimMobility: Location Pointer is nullptr");
				}

				ploc->addVehicleId(veh_id);
				veh.setLocationId(loc_id);
				amod::Event ev(amod::EVENT_LOCATION_VEHS_SIZE_CHANGE, ++event_id_,
						"LocationVehSizeChange", state_.getCurrentTime(),
						{loc_id});
				world_state->addEvent(ev);

				// update the customer and trigger event if necessary
				if (cust.getId() && cust.isInVehicle()) {
					amod::Location *ploc = world_state->getLocationPtr(loc_id);
					ploc->addCustomerId(cust_id);
					cust.setLocationId(loc_id);
					amod::Event ev(amod::EVENT_LOCATION_CUSTS_SIZE_CHANGE, ++event_id_,
							"LocationCustSizeChange", state_.getCurrentTime(),
							{loc_id});
					world_state->addEvent(ev);
				}
			}

			// update the world state
			world_state->setVehicle(veh); //update the vehicle in the world state
			state_.setVehicle(veh); // update the internal state

			world_state->setCustomer(cust);
			state_.setCustomer(cust);

			dispatches_.erase(it++);

			// if dispatch has non-zero booking id
			if (bid) {
				if (cust.isInVehicle()) {
					//dropoff
					//if (getVerbose()) std::cout << "Car has customer - dropping off" << cust.getId() << std::endl;
					auto rc = dropoffCustomer(world_state, veh.getId(), cust.getId(), VehicleStatus::DROPPING_OFF, VehicleStatus::FREE, bid);
					if (rc != amod::SUCCESS) {
						throw std::runtime_error("Could not drop off customer");
					}
				} else {
					// pickup the customer
					//if (getVerbose()) std::cout << "Car is empty - picking up " << cust.getId() << std::endl;
					pickupCustomer(world_state, bookings_[bid].veh_id, bookings_[bid].cust_id, VehicleStatus::PICKING_UP, VehicleStatus::FREE, bid);
				}
			}

		} else {
			++it;
			//if (getVerbose()) std::cout << "Vehicle pos:" << veh.getPosition().x << " " << veh.getPosition().y << std::endl;
			std::vector<int> entities = {veh.getId()};

			// check if the customer is in the vehicle
			if (cust_id && cust.isInVehicle()) {
				entities.push_back(cust_id);
			}

			// trigger move event
			Event ev(amod::EVENT_MOVE, ++event_id_, "VehicleMoved", state_.getCurrentTime(), entities);
			world_state->addEvent(ev);

			// update the world_state and internal state
			world_state->setVehicle(veh); //update the vehicle in the world state
			state_.setVehicle(veh);
			world_state->setCustomer(cust);
			state_.setCustomer(cust);

		}
	}
}

void SimulatorBasic::simulateCustomers(amod::World *world_state) {
	// this is not necessary (at the moment) as performed using by simulateVehicles.
	return;
}

bool SimulatorBasic::hasArrived(const Dispatch &d) {
	//Position diff(d.to.x - d.curr.x, d.to.y - d.curr.y);
	//return !((sign(diff.x) == sign(d.grad.x)) && (sign(diff.x) == sign(d.grad.x)));
	double dist_to_dest = getDistance(d.from, d.to);
	double dist_to_curr = getDistance(d.from, d.curr);
	return (dist_to_curr >= dist_to_dest); // distance travelled larger or equal to distance to destination
}

void SimulatorBasic::simulatePickups(amod::World *world_state) {
	auto it = pickups_.begin();
	while (it != pickups_.end()) {
		if (it->first <= state_.getCurrentTime()) {

			if (getVerbose()) std::cout << it->second.veh_id << " has picked up " << it->second.cust_id << " at time " << it->first << std::endl;

			// create pickup event
			std::vector<int> entity_ids = {it->second.veh_id, it->second.cust_id};
			int bid = it->second.booking_id;
			if (bid) {
				entity_ids.push_back(bid);
			}
			Event ev(amod::EVENT_PICKUP, ++event_id_, "CustomerPickup", it->first, entity_ids);
			world_state->addEvent(ev);

			Vehicle veh = world_state->getVehicle(it->second.veh_id);
			veh.setCustomerId(it->second.cust_id);
			Customer cust = world_state->getCustomer(it->second.cust_id);
			cust.setAssignedVehicleId(it->second.veh_id);
			cust.setInVehicle();
			cust.setLocationId(0);

			// sets vehicle state
			veh.setStatus(it->second.veh_end_status);

			// update the world state and internal state
			world_state->setCustomer(cust);
			world_state->setVehicle(veh);
			state_.setCustomer(cust);
			state_.setVehicle(veh);


			if (bid) {
				ReturnCode rc = dispatchVehicle(world_state, veh.getId(), bookings_[bid].destination,
						VehicleStatus::MOVING_TO_DROPOFF, VehicleStatus::HIRED,
						bid);
				if (rc != amod::SUCCESS) {
					if (getVerbose()) std::cout << bookings_[bid].destination.x << " " <<
							bookings_[bid].destination.y << std::endl;
					if (getVerbose()) std::cout << kErrorStrings[rc] << std::endl;
					throw std::runtime_error("redispatch failed!");
				}
			}

			// erase item
			pickups_.erase(it);
			it = pickups_.begin();
		} else {
			break;
		}
	}
}

void SimulatorBasic::simulateDropoffs(amod::World *world_state) {
	auto it = dropoffs_.begin();
	while (it != dropoffs_.end()) {
		if (it->first <= state_.getCurrentTime()) {
			// create dropoff event
			if (verbose_) if (verbose_) std::cout  << it->second.veh_id << " has dropped off " << it->second.cust_id << " at time " << it->first << std::endl;
			int loc_id = it->second.loc_id;
			int bid = it->second.booking_id;
			std::vector<int> entity_ids = {it->second.veh_id, it->second.cust_id};
			if (bid) entity_ids.push_back(bid);
			amod::Event ev(amod::EVENT_DROPOFF, ++event_id_, "CustomerDropoff", it->first, entity_ids);
			world_state->addEvent(ev);

			amod::Vehicle *veh = world_state->getVehiclePtr(it->second.veh_id);
			veh->clearCustomerId();
			veh->setStatus(it->second.veh_end_status);
			veh->setLocationId(it->second.loc_id);

			amod::Customer *cust = world_state->getCustomerPtr(it->second.cust_id);
			cust->clearAssignedVehicleId();
			cust->setStatus(amod::CustomerStatus::FREE);
			cust->setLocationId(it->second.loc_id);

			// if is part of a booking, clear it since the vehicle has fropped off the custmer

			if (bid) {
				bookings_.erase(bid);
			}

			// erase item
			dropoffs_.erase(it);
			it = dropoffs_.begin();

			// update locations
			// update the location to indicate the vehicle is here.
			{

				amod::Location *ploc = world_state->getLocationPtr(loc_id);
				ploc->addVehicleId(veh->getId());
				veh->setLocationId(loc_id);
				amod::Event ev(amod::EVENT_LOCATION_VEHS_SIZE_CHANGE, ++event_id_,
						"LocationVehSizeChange", state_.getCurrentTime(),
						{loc_id});
				world_state->addEvent(ev);

				// update the customer and trigger event if necessary
				if (cust && cust->isInVehicle()) {
					amod::Location *ploc = world_state->getLocationPtr(loc_id);
					ploc->addCustomerId(cust->getId());
					cust->setLocationId(loc_id);
					amod::Event ev(amod::EVENT_LOCATION_CUSTS_SIZE_CHANGE, ++event_id_,
							"LocationCustSizeChange", state_.getCurrentTime(),
							{loc_id});
					world_state->addEvent(ev);
				}
			}




		} else {
			break;
		}
	}
}


void SimulatorBasic::simulateTeleports(amod::World *world_state) {
	auto it = teleports_.begin();
	while (it != teleports_.end()) {
		if (it->first <= state_.getCurrentTime()) {
			// create teleportation arrival event
			if (getVerbose()) std::cout << it->second.cust_id << " has teleported to location " << it->second.loc_id << " at time " << it->first << std::endl;

			std::vector<int> entity_ids = {it->second.cust_id};
			Event ev(amod::EVENT_TELEPORT_ARRIVAL, ++event_id_, "CustomerTeleportArrival", it->first, entity_ids);
			world_state->addEvent(ev);

			Customer cust = world_state->getCustomer(it->second.cust_id);
			cust.setStatus(it->second.cust_end_status);
			cust.setPosition(world_state->getLocationPtr(it->second.loc_id)->getPosition());


			// update the customer and trigger event if necessary
			if (using_locations_) {
				int loc_id = it->second.loc_id;
				Location *ploc = world_state->getLocationPtr(loc_id);
				ploc->addCustomerId(it->second.cust_id);
				Event ev(amod::EVENT_LOCATION_CUSTS_SIZE_CHANGE, ++event_id_,
						"LocationCustSizeChange", state_.getCurrentTime(),
						{loc_id});
				world_state->addEvent(ev);

				// update internal state
				ploc = state_.getLocationPtr(loc_id);
				ploc->addCustomerId(it->second.cust_id);

				cust.setLocationId(loc_id);
			}
			// update the external world and internal state
			world_state->setCustomer(cust);
			state_.setCustomer(cust);

			// erase item
			teleports_.erase(it);
			it = teleports_.begin();
		} else {
			break;
		}
	}
}


void SimulatorBasic::setPickupDistributionParams(double mean, double sd, double min, double max) {
	std::normal_distribution<>::param_type par{mean, sd};
	pickup_params_.par = par;
	pickup_params_.max = max;
	pickup_params_.min = min;
}

void SimulatorBasic::setDropoffDistributionParams(double mean, double sd, double min, double max) {
	std::normal_distribution<>::param_type par{mean, sd};
	dropoff_params_.par = par;
	dropoff_params_.max = max;
	dropoff_params_.min = min;
}

void SimulatorBasic::setVehicleSpeedParams(double mean, double sd, double min, double max) {

	std::normal_distribution<>::param_type par{mean, sd};
	speed_params_.par = par;
	speed_params_.max = max;
	speed_params_.min = min;
}

void SimulatorBasic::setTeleportDistributionParams(double mean, double sd, double min, double max) {

	std::normal_distribution<>::param_type par{mean, sd};
	teleport_params_.par = par;
	teleport_params_.max = max;
	teleport_params_.min = min;
}

double SimulatorBasic::genRandTruncNormal(TruncatedNormalParams &params) {
	normal_dist.param(params.par);
	double r = normal_dist(eng);
	if (r > params.max) r = params.max;
	if (r < params.min) r = params.min;
	return r;
}


} /* namespace amod */
