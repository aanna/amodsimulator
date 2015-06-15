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
        
        auto itr = bookings_hist_.find(loc_id);
        if (itr != bookings_hist_.end()) {
            int bin = round(fmod(t, kSecondsInDay)/bin_width_);
            auto titr = itr->second.find(bin);
            if (titr != itr->second.end()) {
                mean = titr->second;
            }
        }
        
        return std::make_pair(mean, var);
    }
    
    
    
    amod::ReturnCode SimpleDemandEstimator::loadBookingsFromFile(const std::string filename) {
        if (locs_tree_.size() == 0) {
            throw std::runtime_error("SimpleDemandEstimator needs locations before loading bookings.");
        }
        
        std::ifstream in(filename.c_str());
        if (!in.good()) {
            std::cout << "Cannot read: " << filename << std::endl;
            return amod::ERROR_READING_BOOKINGS_FILE;
        }
        
        std::vector<Booking> bookings;
        
        while (in.good()) {
            Booking b;
            in >> b.id >> b.booking_time >> b.cust_id >> b.source.x >> b.source.y >> b.destination.x >> b.destination.y >> b.travel_mode;
            if (b.id && (b.travel_mode == amod::Booking::AMODTRAVEL) && in.good()) bookings.emplace_back(b); 
        }
        
        makeBookingsHist(bookings);
        
        // its silly but we need to the following or it doesn't work properly in the clang compiler
        // TODO: further testing required.
        /*for (auto itr = bookings.begin(); itr != bookings.end(); itr++) {
            auto &b = *itr;
            std::cout << b.id << std::endl;
        }*/
        
        return amod::SUCCESS;
    }
    
    amod::ReturnCode SimpleDemandEstimator::loadBookings(const std::vector<Booking> &bookings) {

        makeBookingsHist(bookings);

        return amod::SUCCESS;
    }
    
    
    amod::ReturnCode SimpleDemandEstimator::loadBookingsHistFromFile(const std::string filename) {
    	if (locs_tree_.size() == 0) {
			throw std::runtime_error("SimpleDemandEstimator needs locations before loading demand.");
		}

		std::ifstream in(filename.c_str());
		if (!in.good()) {
			std::cout << "Cannot read: " << filename << std::endl;
			return amod::ERROR_READING_DEMAND_HIST_FILE;
		}

		bookings_hist_.clear();
		int nstations;
		in >> nstations >> bin_width_;
		int nbins = kSecondsInDay/bin_width_;

		if (nstations != locs_tree_.size()) {
			std::cout << nstations << " " << bin_width_ << std::endl;
            std::cout << "Locations tree size: " << locs_tree_.size() << std::endl;
			throw std::runtime_error("SimpleDemandEstimator: locations tree size does not match number of stations in bookings histogram file");
		}

		// read in just the mean predictions for now
		// TODO: Read in predicted variances and use that
		for (int s=0; s<nstations; ++s) {
			int station_id;
			double bin_data;
			in >> station_id;
			for (int bin=0; bin<nbins; bin++) {
				in >> bin_data;
	            auto itr = bookings_hist_[station_id].find(bin);
	            if (itr != bookings_hist_[station_id].end()) {
	                itr->second = bin_data;
	            } else {
	                bookings_hist_[station_id][bin] = bin_data;
	            }
			}
		}


		return amod::SUCCESS;
    }

    
    void SimpleDemandEstimator::loadLocations(std::vector<amod::Location> &locations)
    {
        if (locations.size() <= 0) return;
        
        // create a tree for quick lookup of location ids
        locs_tree_.build(locations);
        
        // set up for histogram maps
        for (auto l : locations) {
            bookings_hist_[l.getId()] = {};
            day_counts_[l.getId()] = {};
        }
    
        return;
    }
    
    void SimpleDemandEstimator::makeBookingsHist(const std::vector<Booking> &bookings) {
        day_counts_.clear();
        bookings_hist_.clear();
        

        for (auto b : bookings) {
            
            // get the source location id
            int from_id = locs_tree_.findNN({b.source.x,  b.source.y}).getId();

            // update day counts
            int day = floor(b.booking_time/kSecondsInDay);
            int bin = floor(fmod(b.booking_time, kSecondsInDay)/bin_width_);
            
            auto ditr = day_counts_[from_id].find(bin);
            if (ditr != day_counts_[from_id].end()) {
                ditr->second.insert(day);
            } else {
                day_counts_[from_id][bin] = {day};
            }
            
            // update bin counts
            auto itr = bookings_hist_[from_id].find(bin);
            if (itr != bookings_hist_[from_id].end()) {
                itr->second += 1;
            } else {
                bookings_hist_[from_id][bin] = 1.0;
            }
            
        }
        
        // divide bin counts by day to get average for each bin
        for (auto itr=bookings_hist_.begin(); itr != bookings_hist_.end(); ++itr) {
            int loc_id = itr->first;
            for (auto ditr=bookings_hist_[loc_id].begin(); ditr != bookings_hist_[loc_id].end(); ++ditr) {
                ditr->second /= day_counts_[loc_id][ditr->first].size();
                //std::cout << ditr->second << std::endl;
            }
        }
        
        return;
    }
}
