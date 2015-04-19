//
//  DemandEstimator.h
//  AMODBase
//
//  Created by Harold Soh on 18/4/15.
//  Copyright (c) 2015 Harold Soh. All rights reserved.
//

#ifndef __AMODBase__DemandEstimator__
#define __AMODBase__DemandEstimator__

#include <utility>
#include "Types.hpp"
#include "World.hpp"

namespace amod {
    class DemandEstimator {
    public:
        DemandEstimator() { } ;
        virtual ~DemandEstimator() {};
        
        // predict
        // predicts the demand (number of customer bookings) at a given position pos or location id
        // at a given time t
        // returns a std::pair with the mean prediction and the uncertainty
        virtual std::pair<double, double> predict(const amod::Position &pos, const amod::World &world_state, double t) = 0;
        virtual std::pair<double, double> predict(int loc_id, const amod::World &world_state, double t) = 0;
        
        
    };
}




#endif /* defined(__AMODBase__DemandEstimator__) */
