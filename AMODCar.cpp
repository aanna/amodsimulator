/*
 * AMODCar.cpp
 *
 *  Created on: Mar 23, 2015
 *      Author: haroldsoh
 */

#include <entities/amodController/AMODCar.hpp>

namespace AMODBase {

AMODCar::AMODCar(int id) : status_(UNKNOWN), speed_(0) {
	// TODO Auto-generated constructor stub
	AMODObject::setID(id);
}

AMODCar::~AMODCar() {
	// TODO Auto-generated destructor stub
}

} /* namespace AMOD */
