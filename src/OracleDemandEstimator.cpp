//
//  OracleDemandEstimator.cpp
//  AMODBase
//
//  Created by Harold Soh on 18/4/15.
//  Copyright (c) 2015 Harold Soh. All rights reserved.
//

#include "OracleDemandEstimator.hpp"

namespace amod {

std::pair<double, double> OracleDemandEstimator::predict(const amod::Position &pos, double t) {
    double mean = 0.0;
    double var = 0.0;
    
    return std::make_pair(mean, var);
}

}