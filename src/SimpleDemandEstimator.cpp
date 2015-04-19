//
//  SimpleDemandEstimator.cpp
//  AMODBase
//
//  Created by Harold Soh on 18/4/15.
//  Copyright (c) 2015 Harold Soh. All rights reserved.
//

#include "SimpleDemandEstimator.hpp"

namespace amod {
    
    std::pair<double, double> SimpleDemandEstimator::predict(const amod::Position &pos, const amod::World &world_state, double t)
    {

        // get pos's station
        auto loc = locs_tree_.findNN({pos.x, pos.y});
        int loc_id = loc.getId();
        
        return predict(loc_id, world_state, t);
    }
    
    std::pair<double, double> SimpleDemandEstimator::predict(int loc_id, const amod::World &world_state, double t) {
       
        double mean = daily_mean_;
        double var = daily_var_;
        
        auto itr = demands_hist_.find(loc_id);
        if (itr != demands_hist_.end()) {
            int bin = round(fmod(t, kSecondsInDay)/bin_width_);
            auto titr = itr->second.find(bin);
            if (titr != itr->second.end()) {
                mean = titr->second;
            }
        }
        
        return std::make_pair(mean, var);
    }
    
    
    
    amod::ReturnCode SimpleDemandEstimator::loadDemandFromFile(const std::string filename) {
        if (locs_tree_.size() == 0) {
            throw std::runtime_error("SimpleDemandEstimator needs locations before loading demand.");
        }
        
        
        std::ifstream in(filename.c_str());
        if (!in.good()) {
            std::cout << "Cannot read: " << filename << std::endl;
            return amod::ERROR_READING_BOOKINGS_FILE;
        }
        
        std::vector<Demand> demands;
        
        while (in.good()) {
            Demand d; 
            in >> d.id >> d.t >> d.loc_id >> d.pos.x >> d.pos.y;
            if (d.id && in.good()) demands.emplace_back(d); //only positive booking ids allowed
        }
        
        /*
         for (auto itr = bookings_.begin(); itr != bookings_.end(); itr++) {
         auto &b = itr->second;
         std::cout << b.id << ": " << b.booking_time << " " << b.cust_id << " " << b.travel_mode << std::endl;
         }
         */
        
        makeDemandHist(demands);
        return amod::SUCCESS;
    }
    
    
    void SimpleDemandEstimator::loadLocations(std::vector<amod::Location> &locations)
    {
        if (locations.size() <= 0) return;
        
        // create a tree for quick lookup of location ids
        locs_tree_.build(locations);
        
        // set up for histogram maps
        for (auto l : locations) {
            demands_hist_[l.getId()] = {};
            day_counts_[l.getId()] = {};
        }
    
        return;
    }
    
    void SimpleDemandEstimator::makeDemandHist(const std::vector<Demand> &demands) {
        day_counts_.clear();
        demands_hist_.clear();
        
        
        for (auto d : demands) {
            
            // get the source location id
            if (d.loc_id == 0) {
                d.loc_id = locs_tree_.findNN({d.pos.x, d.pos.y}).getId();
            }
            
            // update day counts
            int day = floor(d.t/kSecondsInDay);
            auto ditr = day_counts_[d.loc_id].find(day);
            if (ditr != day_counts_[d.loc_id].end()) {
                ditr->second += 1;
            } else {
                day_counts_[d.loc_id][day] = 1.0;
            }
            
            // update bin counts
            int bin = floor(fmod(d.t, kSecondsInDay)/bin_width_);
            auto itr = demands_hist_[d.loc_id].find(bin);
            if (itr != demands_hist_[d.loc_id].end()) {
                itr->second += 1;
            } else {
                demands_hist_[d.loc_id][bin] = 1.0;
            }
            
        }
        
        // divide bin counts by day to get average for each bin
        for (auto itr=demands_hist_.begin(); itr != demands_hist_.end(); ++itr) {
            int loc_id = itr->first;
            for (auto ditr=demands_hist_[loc_id].begin(); ditr != demands_hist_[loc_id].end(); ++ditr) {
                ditr->second /= day_counts_[loc_id][ditr->first];
            }
        }
        
        return;
    }
}