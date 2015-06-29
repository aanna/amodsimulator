#include <iostream>
#include <cstdlib>
#include <vector>
#include <sstream>
#include <limits.h>
#include <unordered_map>


#include "Amod.hpp"
#include "SimulatorBasic.hpp"
#include "ManagerBasic.hpp"
#include "ManagerMatchRebalance.hpp"
#include "SimpleDemandEstimator.hpp"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>

namespace pt = boost::property_tree; // for reading in our configuration file

class AMODECBDCaseStudy {
    
public:
    AMODECBDCaseStudy(): sim_(nullptr),
        manager_(nullptr),
        de_(nullptr),
        output_move_events_(true) {
        return;
    };
    
    ~AMODECBDCaseStudy() {
        delete sim_;
        delete manager_;
        delete de_;
    }
    
    bool init(std::string amod_config_filename)
    {
        bool init_good = true;
        // load configuration file 
        acfg_.loadFile(amod_config_filename.c_str()); //perhaps we can specify this in the simmobility XML
        std::string default_string = "";    
        // init random number generator
        srand(0);

        // =============================================================
        // Initialize the world
        // load the stations
        std::string stn_cfg_filename = acfg_.get("amod.station_cfg_filename", default_string);
        std::cout << "Initializing stations using: " << stn_cfg_filename << std::endl;
        std::vector<amod::Location> stations;
        loadLocationsFile(stn_cfg_filename, &stations);
        
        // create vehicles
        int nvehs =  acfg_.get("amod.num_vehicles", 0);
        std::cout << "Initializing " << nvehs << " vehicles "<< std::endl;
        std::vector<amod::Vehicle> vehicles;
        initVehicles(stations, &vehicles, nvehs);
        
        // create customers
        std::string customer_cfg_filename = acfg_.get("amod.customer_cfg_filename", default_string);
        std::cout << "Initializing customers using: " << customer_cfg_filename << std::endl;
        std::vector<amod::Customer> customers;
        loadCustomerFile(customer_cfg_filename, &customers);

        // Locations are nodes (we leave this empty because the simulator
        // will populate the locations
        std::vector<amod::Location> locations;
        std::string location_cfg_filename = acfg_.get("amod.location_cfg_filename", default_string);
        std::cout << "Initializing locations using: " << location_cfg_filename << std::endl;
        loadLocationsFile(location_cfg_filename, &locations);
        

        // =============================================================
        // create a new simulator 
        bool verbose = true;
        double resolution  = acfg_.get("amod.simulator_params.resolution", 0.1);
        std::cout << "Setting simulation resolution at: " << resolution << std::endl;
        
        sim_ = new amod::SimulatorBasic(resolution);
        double start_time  = acfg_.get("amod.simulator_params.start_time", 0.0);
        max_time_  = acfg_.get("amod.simulator_params.max_time", 3600.0);

        // setup distributions for simulation
        double pickup_mean, pickup_std, pickup_max, pickup_min;
        pickup_mean = acfg_.get("amod.simulator_params.pickup_distribution.mean", 60.0);
        pickup_std = acfg_.get("amod.simulator_params.pickup_distribution.std", 20.0);
        pickup_min = acfg_.get("amod.simulator_params.pickup_distribution.min", 10.0);
        pickup_max = acfg_.get("amod.simulator_params.pickup_distribution.max", 200.0);
        sim_->setPickupDistributionParams(pickup_mean, pickup_std, pickup_min, pickup_max); // in seconds
        
        double teleport_mean, teleport_std, teleport_max, teleport_min;
        teleport_mean = acfg_.get("amod.simulator_params.teleport_distribution.mean", 60.0);
        teleport_std = acfg_.get("amod.simulator_params.teleport_distribution.std", 20.0);
        teleport_min = acfg_.get("amod.simulator_params.teleport_distribution.min", 10.0);
        teleport_max = acfg_.get("amod.simulator_params.teleport_distribution.max", 200.0);
        sim_->setTeleportDistributionParams(teleport_mean, teleport_std, teleport_min, teleport_max); // in seconds
        
        double dropoff_mean, dropoff_std, dropoff_max, dropoff_min;
        dropoff_mean = acfg_.get("amod.simulator_params.dropoff_distribution.mean", 60.0);
        dropoff_std = acfg_.get("amod.simulator_params.dropoff_distribution.std", 20.0);
        dropoff_min = acfg_.get("amod.simulator_params.dropoff_distribution.min", 10.0);
        dropoff_max = acfg_.get("amod.simulator_params.dropoff_distribution.max", 200.0);
        sim_->setDropoffDistributionParams(dropoff_mean, dropoff_std, dropoff_min, dropoff_max); // in seconds
        
        double speed_mean, speed_std, speed_max, speed_min;
        speed_mean = acfg_.get("amod.simulator_params.speed_distribution.mean", 60.0);
        speed_std = acfg_.get("amod.simulator_params.speed_distribution.std", 20.0);
        speed_min = acfg_.get("amod.simulator_params.speed_distribution.min", 10.0);
        speed_max = acfg_.get("amod.simulator_params.speed_distribution.max", 200.0);       
        sim_->setVehicleSpeedParams(speed_mean, speed_std, speed_min, speed_max); // in m/s
        
        // populate the world
        world_.populate(locations, vehicles, customers);
        world_.setCurrentTime(start_time);
        sim_->init(&world_);


        // =============================================================
        // setup the matching manager
        amod::ManagerMatchRebalance *match_manager= new amod::ManagerMatchRebalance();
        
        // are we doing greedy or assignment matching?
        std::string match_method_str = acfg_.get("amod.matching_algorithm", default_string);
        std::transform(match_method_str.begin(), match_method_str.end(), match_method_str.begin(), ::toupper);
        if (match_method_str == "GREEDY") {
            std::cout << "Using Greedy Matching" << std::endl;
            match_manager->setMatchMethod(amod::ManagerMatchRebalance::GREEDY); 
            double match_interval = acfg_.get("amod.assignment_params.matching_interval", 1.0);
            match_manager->setMatchingInterval(match_interval); 
        } else if (match_method_str == "ASSIGNMENT") {
            std::cout << "Using Assignment Matching" << std::endl;
            match_manager->setMatchMethod(amod::ManagerMatchRebalance::ASSIGNMENT);
            double distance_cost_factor = acfg_.get("amod.assignment_params.distance_cost_factor", 1.0);
            double waiting_cost_factor = acfg_.get("amod.assignment_params.waiting_cost_factor", 1.0);
            match_manager->setCostFactors(distance_cost_factor, waiting_cost_factor);
            double match_interval = acfg_.get("amod.assignment_params.matching_interval", 1.0);
            match_manager->setMatchingInterval(match_interval); 
        } else {
            std::cout << "ERROR! No such matching" << std::endl;
            throw std::runtime_error("No such assignment method supported! Check your amod_config xml file");
        }
        
        // initialize the manager and load the bookings
        match_manager->init(&world_);
        match_manager->setSimulator(sim_); // set simulator
        std::string bookings_filename = acfg_.get("amod.bookings_filename", default_string);
        std::cout << "Loading Bookings from " << bookings_filename << std::endl;
        if (match_manager->loadBookingsFromFile(bookings_filename) == amod::ERROR_READING_BOOKINGS_FILE) {
            throw std::runtime_error("AMODController: Cannot read bookings file");
        }
        //atch_manager->loadBookings(bookings); // load the bookings

        // set the demand estimator for demand estimation (used for rebalancing)
        amod::SimpleDemandEstimator *sde = new amod::SimpleDemandEstimator();
        sde->loadLocations(stations);
        
        std::string demand_est_method_str = acfg_.get("amod.rebalancing_params.demand_estimation_method", default_string);
        std::transform(demand_est_method_str.begin(), demand_est_method_str.end(), demand_est_method_str.begin(), ::toupper);
        if (demand_est_method_str == "ORACLE") {
            std::cout << "Demand Oracle is Loading Bookings from " << bookings_filename << std::endl;
            sde->loadBookingsFromFile(bookings_filename); 
        } else if (demand_est_method_str == "FILE") {
            std::string demand_hist_filename = acfg_.get("amod.rebalancing_params.demand_estimation_file", default_string);
            std::cout << "Demand Prediction is loading bookings histogram from " << demand_hist_filename << " ... ";
            sde->loadBookingsHistFromFile(demand_hist_filename);
            std::cout << "Done!" << std::endl;
        } else {
            std::cout << "No such demand estimation method" << std::endl;
            throw std::runtime_error("No such assignment method supported! Check your amod_config xml file");
        }
        
        double rebalancing_interval = acfg_.get("amod.rebalancing_params.rebalancing_interval", 1800.0);
        std::cout << "Setting Rebalancing Interval: " << rebalancing_interval << std::endl;
        match_manager->setDemandEstimator(sde); // set the demand estimator (for rebalancing)
        match_manager->loadStations(stations, world_);
        match_manager->setRebalancingInterval(rebalancing_interval); 
        
        std::string verbose_str = acfg_.get("amod.verbose", default_string);
        std::transform(verbose_str.begin(), verbose_str.end(), verbose_str.begin(), ::toupper);
        if (verbose_str == "TRUE") {
            std::cout << "Manager is VERBOSE" << std::endl;
            match_manager->setVerbose(true);
            
            std::cout << "Simulator is VERBOSE" << std::endl;
            sim_->setVerbose(true);
            
        } else {
            std::cout << "Manager is QUIET" << std::endl;
            match_manager->setVerbose(false);
            
            std::cout << "Simulator is QUIET" << std::endl;
            sim_->setVerbose(false);
            
        }
        // select which manager we want
        manager_ = match_manager; //simple_manager
        
        // =============================================================
        // setup the logger
        std::string log_filename = acfg_.get("amod.log_filename", std::string("amodlog.txt"));
        logger_.openLogFile(log_filename);
        
        int move_log_interval = acfg_.get("amod.move_event_log_interval", 60.0);
        logger_.setMoveEventLogInterval(move_log_interval); //output car movements every 30 seconds
        
        return init_good;    
    }
    
    
    // runs the case study for one tick 
    amod::ReturnCode update()
    {

        double current_time = world_.getCurrentTime();
        if (current_time > max_time_) {
            return amod::FAILED;
        } 
        
        sim_->update(&world_);
        manager_->update(&world_);
        logger_.logEvents(&world_, output_move_events_); //writes to disk and erases

        return amod::SUCCESS;
    }
    
    double getCurrentTime() {
        return world_.getCurrentTime();
    }
    
    double getMaxTime() {
        return max_time_;
    }
    
private:
    
    struct AmodConfig {
    public:
        AmodConfig () {};
        ~AmodConfig () {};

        void loadFile(std::string config_filename) {
            pt::read_xml(config_filename, tree);
        }

        template <typename T> 
        T get(std::string param_name, T def_val) {
            return tree.get(param_name, def_val);
        };

        pt::ptree tree;
    };
    
    amod::SimulatorBasic *sim_;
    amod::Manager *manager_;
    amod::Logger logger_;
    amod::DemandEstimator *de_;
    bool output_move_events_;

    // configuration file
    AmodConfig acfg_;
    
    // temporary test variables
    amod::World world_;
    double max_time_;


    // loads the vehicles. Format expected is lines with the values id, x, y
    void loadLocationsFile(std::string filename, std::vector<amod::Location> *locs) {
        std::ifstream fin(filename.c_str());
        if (!fin.good()) {
            throw std::runtime_error("Cannot load from locations file");
        } 
        
        // load the file
        int k = 0;
        while (fin.good()) {
            int id, capacity;
            double x, y;
            id = 0;
            capacity = 0; //we ignore capacity for now
            fin >> id >> x >> y; 
            if (id != 0) {
                std::stringstream ss;
                ss << id;
                locs->emplace_back( id, ss.str(), amod::Position(x,y), capacity);  
            }
        }
        
        std::cout << locs->size() << " stations loaded" << std::endl;
    }

    void initVehicles(const std::vector<amod::Location> & locs, std::vector<amod::Vehicle> *vehs, int nvehs) {
        
        int k=0;
        int nstations = locs.size();
        if (nstations == 0 || nvehs == 0) return;
        
        int even_nvehs = nvehs/nstations;
        for (auto itr=locs.begin(); itr!=locs.end(); ++itr) {
            for (int i=0; i< even_nvehs; ++i) {
                // create a new vehicle
                std::stringstream ss;
                ss << ++k;
                vehs->emplace_back(k, ss.str(), itr->getPosition(), 1, amod::VehicleStatus::FREE);
            }
        }
        
        int nextra_vehs = nvehs - even_nvehs*nstations;
        for (auto itr=locs.begin(); itr!=locs.end(); ++itr) {
            if (nextra_vehs > 0) {
                // create a new vehicle
                std::stringstream ss;
                ss << ++k;
                vehs->emplace_back(k, ss.str(), itr->getPosition(), 1, amod::VehicleStatus::FREE);
                nextra_vehs--;
            } else {
                break;
            }
        }
        
        std::cout << vehs->size() << " vehicles created" << std::endl;
    }

    // loads the customers
    void loadCustomerFile(std::string filename, std::vector<amod::Customer> *custs) {
        std::ifstream fin(filename.c_str());
        if (!fin.good()) {
            throw std::runtime_error("Cannot load from customer config file");
        } 
        
        // load the file
        while (fin.good()) {
            int id = 0;
            double x, y;
            
            fin >> id >> x >> y;
            if (id != 0) {
                // create a new customer
                std::stringstream ss;
                ss << id;
                custs->emplace_back(id, ss.str(), amod::Position(x,y));
            }
        }
        
        std::cout << custs->size() << " customers loaded" << std::endl;
    }
    
};


int main(int argc, char **argv) {
    std::cout << "AMOD ECBD Case Study Simulation" << std::endl;
    
    // Set up the simulation 
    std::cout << "Setting up simulation" << std::endl;
    
    AMODECBDCaseStudy case_study;
    std::string amod_config_filename = argv[1]; 
    case_study.init(amod_config_filename);

    int perc_done = 0;
    while (case_study.update()) {
        // output displays here
        double current_time = case_study.getCurrentTime();
        double max_time = case_study.getMaxTime();
        int curr_perc = (int) floor(current_time*100.0/max_time);
        if (curr_perc > perc_done) {
            perc_done = curr_perc;
            std::cout << "Percentage Done: [" << perc_done << "%]" << std::endl;
        }
    }
    
    std::cout << "Simulation Done!" << std::endl;
    
    return 0;
}
