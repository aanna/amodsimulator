/*
 * AMODManager.h
 *  Abstract base class for managers to inherit from
 *
 *  Created on: Mar 23, 2015
 *      Author: haroldsoh
 */

#ifndef AMODMANAGER_H_
#define AMODMANAGER_H_

#include "AMODTypes.hpp"
#include "AMODSimulator.hpp"
#include "AMODLogger.hpp"
#include <stdexcept>

namespace AMODBase {

class AMODManager {
public:
	AMODManager();
    virtual ~AMODManager() {};

    virtual void init() = 0;
    virtual AMODBase::ReturnCode update() = 0;
    
    virtual void setSimulator(AMODBase::AMODSimulator *sim) {
        if (!sim) {
            throw std::runtime_error("AMODManager::setSimulator: sim is nullptr");
        }
        sim_ = sim;
    };
    
    virtual AMODBase::AMODSimulator* getSimulator() {
        return sim_;
    }
    
    virtual void setLogger(AMODBase::AMODLogger *logger) {
        if (!logger) {
            throw std::runtime_error("AMODManager::setSimulator: logger is nullptr");
        }
        logger_ = logger;
    };
    
    virtual AMODBase::AMODLogger* getLogger() {
        return logger_;
    }
    
    
private:
    AMODBase::AMODSimulator* sim_;
    AMODBase::AMODLogger *logger_;

};

} /* namespace AMOD */

#endif /* AMODMANAGER_H_ */
