/*
 * AMODCar.h
 *
 *  Created on: Mar 23, 2015
 *      Author: haroldsoh
 */

#ifndef AMODCAR_H_
#define AMODCAR_H_

#include "AMODTypes.hpp"
#include "AMODObject.hpp"
#include <memory>

namespace AMODBase {


class AMODCar : public AMODObject {
public:
	AMODCar(int id = 0);
	virtual ~AMODCar();

	// simple inline functions
	virtual CarStatus getStatus() { return status_;};
	virtual void setStatus(CarStatus s) { status_ = s; };

	//virtual double getSpeed() { return speed_; };

private:
	CarStatus status_;
	double speed_;
	//std::shared_ptr<AMODSimulator> sim;

};

} /* namespace AMOD */

#endif /* AMODCAR_H_ */
