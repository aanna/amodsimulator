//
//  OracleDemandEstimator.h
//  AMODBase
//
//  Created by Harold Soh on 18/4/15.
//  Copyright (c) 2015 Harold Soh. All rights reserved.
//

#ifndef __AMODBase__OracleDemandEstimator__
#define __AMODBase__OracleDemandEstimator__

#include <utility>
#include "Types.hpp"
#include "DemandEstimator.hpp"

namespace amod {
    class OracleDemandEstimator : DemandEstimator {
    public:
        OracleDemandEstimator() { } ;
        virtual ~OracleDemandEstimator();
        
        // predict
        // predicts the demand (number of customer bookings) at a given position pos
        // at a given time t
        // returns a std::pair with the mean prediction and the uncertainty
        virtual std::pair<double, double> predict(const amod::Position &pos, double t);
        
        
    };
}



#endif /* defined(__AMODBase__OracleDemandEstimator__) */
