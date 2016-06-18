#include <iostream>
#include <cstdlib>
#include <vector>
#include <sstream>
#include <limits.h>
#include <unordered_map>


#include "../src/Amod.hpp"
#include "../src/SimulatorBasic.hpp"
#include "../src/ManagerBasic.hpp"
#include "../src/ManagerMatchRebalance.hpp"
#include "../src/SimpleDemandEstimator.hpp"

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

	bool init(std::string amod_config_filename) {
		bool init_good = true;
		// load configuration file
		std::cout << "Using configuration file: " << amod_config_filename << std::endl;
		std::string defaultString = "";
		/// init random number generator
		srand(0);
		amodConfig.loadFile(amod_config_filename.c_str()); //perhaps we can specify this in the simmobility XML

		// =============================================================
		// Initialize the world
		std::string stnCfgFileName = amodConfig.get("amod.station_cfg_filename", defaultString);
		std::cout << "Initializing stations using: " << stnCfgFileName << std::endl;
		std::vector<amod::Location> stations;
		loadStationsFile(stnCfgFileName, &stations);

		/// create vehicles
		std::vector<amod::Vehicle> vehicles;

		// initialize vehicles based on uniform distribution or from file
		bool veh_uniform_distr = amodConfig.get("amod.uniform_distr", true);
		int nvehs =  amodConfig.get("amod.num_vehicles", 0);

		if (veh_uniform_distr) {
			// vehicles distributed evenly between stations
			std::cout << "Initializing " << nvehs << " vehicles and distribute them evenly across stations."<< std::endl;
			initVehicles(stations, &vehicles, nvehs);
		} else {
			// vehicles loaded from file
			std::string vehCfgFileName = amodConfig.get("amod.veh_cfg_filename", defaultString);
			std::cout << "Initializing " << nvehs << " vehicles from file: " << vehCfgFileName << std::endl;
			initVehiclesFromFile(vehCfgFileName, stations, &vehicles);
		}

		/// create customers
		std::string custConfigFileName = amodConfig.get("amod.customer_cfg_filename", defaultString);
		std::cout << "Initializing customers using: " << custConfigFileName << std::endl;
		std::vector<amod::Customer> customers;
		loadCustomerFile(custConfigFileName, &customers);

		/// Locations are nodes (we leave this empty because the simulator
		/// will populate the locations
		std::vector<amod::Location> locations;
		std::string location_cfg_filename = amodConfig.get("amod.location_cfg_filename", defaultString);
		std::cout << "Initializing locations using: " << location_cfg_filename << std::endl;
		loadLocationsFile(location_cfg_filename, &locations);

		/// populate the world
		world_.populate(locations, vehicles, customers);

		// =============================================================
		// create a new simulator
		double resolution  = amodConfig.get("amod.simulator_params.resolution", 1.0);
		std::cout << "Setting simulation resolution at: " << resolution << std::endl;

		sim_ = new amod::SimulatorBasic(resolution);
		double start_time  = amodConfig.get("amod.simulator_params.start_time", 0.0);
		max_time_  = amodConfig.get("amod.simulator_params.max_time", 3600.0);

		// setup distributions for simulation
		double pickup_mean, pickup_std, pickup_max, pickup_min;
		pickup_mean = amodConfig.get("amod.simulator_params.pickup_distribution.mean", 60.0);
		pickup_std = amodConfig.get("amod.simulator_params.pickup_distribution.std", 20.0);
		pickup_min = amodConfig.get("amod.simulator_params.pickup_distribution.min", 10.0);
		pickup_max = amodConfig.get("amod.simulator_params.pickup_distribution.max", 200.0);
		sim_->setPickupDistributionParams(pickup_mean, pickup_std, pickup_min, pickup_max); // in seconds

		double teleport_mean, teleport_std, teleport_max, teleport_min;
		teleport_mean = amodConfig.get("amod.simulator_params.teleport_distribution.mean", 60.0);
		teleport_std = amodConfig.get("amod.simulator_params.teleport_distribution.std", 20.0);
		teleport_min = amodConfig.get("amod.simulator_params.teleport_distribution.min", 10.0);
		teleport_max = amodConfig.get("amod.simulator_params.teleport_distribution.max", 200.0);
		sim_->setTeleportDistributionParams(teleport_mean, teleport_std, teleport_min, teleport_max); // in seconds

		double dropoff_mean, dropoff_std, dropoff_max, dropoff_min;
		dropoff_mean = amodConfig.get("amod.simulator_params.dropoff_distribution.mean", 60.0);
		dropoff_std = amodConfig.get("amod.simulator_params.dropoff_distribution.std", 20.0);
		dropoff_min = amodConfig.get("amod.simulator_params.dropoff_distribution.min", 10.0);
		dropoff_max = amodConfig.get("amod.simulator_params.dropoff_distribution.max", 200.0);
		sim_->setDropoffDistributionParams(dropoff_mean, dropoff_std, dropoff_min, dropoff_max); // in seconds

		double speed_mean, speed_std, speed_max, speed_min;
		speed_mean = amodConfig.get("amod.simulator_params.speed_distribution.mean", 60.0);
		speed_std = amodConfig.get("amod.simulator_params.speed_distribution.std", 20.0);
		speed_min = amodConfig.get("amod.simulator_params.speed_distribution.min", 10.0);
		speed_max = amodConfig.get("amod.simulator_params.speed_distribution.max", 200.0);
		sim_->setVehicleSpeedParams(speed_mean, speed_std, speed_min, speed_max); // in m/s

		// populate the world
		world_.setCurrentTime(start_time);
		sim_->init(&world_);


		/// =============================================================
		/// setup the matching manager
		amod::ManagerMatchRebalance *matchManager= new amod::ManagerMatchRebalance();

		/// are we doing demand management?
		bool demandManagment = amodConfig.get("amod.demand_manager", false);
		matchManager->isDemandManager(demandManagment);

		// what is the maximum waiting time customers accept?
		int maxWaitingTime = amodConfig.get("amod.customer_param.max_wait_time", 60); // in seconds
		matchManager->loadMaxWaitTime(maxWaitingTime);

		// load dynamic pricing parameters
		std::vector<double> dynPriceFactors;
		double highDemSurStart = amodConfig.get("amod.dynamic_pricing.availability_percent", 1.0);
		dynPriceFactors.push_back(highDemSurStart);
		double sharedRideDiscount = amodConfig.get("amod.dynamic_pricing.shared_ride_discount_factor", 1.0);
		dynPriceFactors.push_back(sharedRideDiscount);
		double sharedWaitTimeIncrease = amodConfig.get("amod.dynamic_pricing.shared_ride_wait_time_increase", 1.0);
		dynPriceFactors.push_back(sharedWaitTimeIncrease);
		double sharedRideArrTimeIncr = amodConfig.get("amod.dynamic_pricing.shared_ride_arrival_time_increase", 1.0);;
		dynPriceFactors.push_back(sharedRideArrTimeIncr);

		matchManager->loadDynamicPriceAndAssortmentParams(dynPriceFactors);

		/// are we doing greedy or assignment matching?
		std::string matchManagerStr = amodConfig.get("amod.matching_algorithm", defaultString);
		std::transform(matchManagerStr.begin(), matchManagerStr.end(), matchManagerStr.begin(), ::toupper);
		if (matchManagerStr == "GREEDY") {
			std::cout << "Using Greedy Matching" << std::endl;
			matchManager->setMatchMethod(amod::ManagerMatchRebalance::GREEDY);
			double matchInterval = amodConfig.get("amod.assignment_params.matching_interval", 1.0);
			matchManager->setMatchingInterval(matchInterval);
		} else if (matchManagerStr == "ASSIGNMENT") {
			std::cout << "Using Assignment Matching" << std::endl;
			matchManager->setMatchMethod(amod::ManagerMatchRebalance::ASSIGNMENT);
			double distanceCostFactor = amodConfig.get("amod.assignment_params.distance_cost_factor", 1.0);
			double waitingCostFactor = amodConfig.get("amod.assignment_params.waiting_cost_factor", 1.0);
			matchManager->setCostFactors(distanceCostFactor, waitingCostFactor);
			double matchInterval = amodConfig.get("amod.assignment_params.matching_interval", 1.0);
			matchManager->setMatchingInterval(matchInterval);
		} else {
			std::cout << "ERROR! No such matching" << std::endl;
			throw std::runtime_error("No such assignment method supported! Check your amod_config xml file");
		}

		/// initialize the manager and load the bookings
		matchManager->init(&world_);
		matchManager->setSimulator(sim_); /// set simulator
		std::string bookingsFileName = amodConfig.get("amod.bookings_filename", defaultString);
		std::cout << "Loading Bookings from " << bookingsFileName << std::endl;
		if (matchManager->loadBookingsFromFile(bookingsFileName) == amod::ERROR_READING_BOOKINGS_FILE) {
			throw std::runtime_error("AMODController: Cannot read bookings file");
		}

		/// set the demand estimator for demand estimation (used for rebalancing)
		amod::SimpleDemandEstimator *sde = new amod::SimpleDemandEstimator();
		sde->loadLocations(stations);

		std::string demandEstMethodStr = amodConfig.get("amod.rebalancing_params.demand_estimation_method", defaultString);
		std::transform(demandEstMethodStr.begin(), demandEstMethodStr.end(), demandEstMethodStr.begin(), ::toupper);
		matchManager->useCurrentQueueForEstimation(false);
		if (demandEstMethodStr == "ORACLE") {
			std::cout << "Demand Oracle is Loading Bookings from " << bookingsFileName << std::endl;
			sde->loadBookingsFromFile(bookingsFileName);
		} else if (demandEstMethodStr == "PREDICTIVE") {
			std::string demand_hist_filename = amodConfig.get("amod.rebalancing_params.demand_estimation_file", defaultString);
			std::cout << "Demand Prediction is loading bookings histogram from " << demand_hist_filename << " ... ";
			sde->loadBookingsHistFromFile(demand_hist_filename);
			std::cout << "Done!" << std::endl;
		} else if (demandEstMethodStr == "QUEUE") {
			std::cout << "Rebalancing using current queue only" << std::endl;
			matchManager->useCurrentQueueForEstimation(true);
		}
		else if (demandEstMethodStr == "FILE") {
			std::cout << "Rebalancing using offline solution" << std::endl;
			std::string rebalancing_f = amodConfig.get("amod.rebalancing_params.rebalancing_file", defaultString);
			matchManager->loadRebalancingFromFile(rebalancing_f);
		}
		else {
			std::cout << "No such demand estimation method" << std::endl;
			throw std::runtime_error("No such rebalancing method supported! Check your amod_config xml file");
		}

		double rebalancingInterval = amodConfig.get("amod.rebalancing_params.rebalancing_interval", 1800.0);
		std::cout << "Setting Rebalancing Interval: " << rebalancingInterval << std::endl;
		matchManager->setDemandEstimator(sde); /// set the demand estimator (for rebalancing)
		matchManager->loadStations(stations, world_);
		matchManager->setRebalancingInterval(rebalancingInterval);

		std::string verboseStr = amodConfig.get("amod.verbose", defaultString);
		std::transform(verboseStr.begin(), verboseStr.end(), verboseStr.begin(), ::toupper);
		if (verboseStr == "TRUE") {
			std::cout << "Manager is VERBOSE" << std::endl;
			matchManager->setVerbose(true);

			std::cout << "Simulator is VERBOSE" << std::endl;
			sim_->setVerbose(true);

		} else {
			std::cout << "Manager is QUIET" << std::endl;
			matchManager->setVerbose(false);

			std::cout << "Simulator is QUIET" << std::endl;
			sim_->setVerbose(false);

		}
		/// select which manager we want
		manager_ = matchManager; //simple_manager

		// =============================================================
		// setup the logger
		std::string log_filename = amodConfig.get("amod.log_filename", std::string("amodlog.txt"));
		logger_.openLogFile(log_filename);

		int move_log_interval = amodConfig.get("amod.move_event_log_interval", 60.0);
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
	AmodConfig amodConfig;

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

	// loads the stations. Format expected is lines with the values id, x, y
	void loadStationsFile(std::string filename, std::vector<amod::Location> *locs) {
		std::ifstream fin(filename.c_str());
		if (!fin.good()) {
			throw std::runtime_error("Cannot load from station config file");
		}

		// load the file
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

	void initVehiclesFromFile(std::string vehCfgFileName, const std::vector<amod::Location> & stations, std::vector<amod::Vehicle> *vehs) {
		std::ifstream fin(vehCfgFileName.c_str());
		if (!fin.good()) {
			throw std::runtime_error("Cannot load from vehCfgFileName config file");
		}

		int k=0; // vehicle id
		/// load the file
		while (fin.good()) {

			int nstations = stations.size();
			if (nstations == 0) return;

			long int st_id;
			int num_veh;

			fin >> st_id >> num_veh;
			if( fin.eof() ) break;

			for (int i=0; i< num_veh; ++i) {
				// create a new vehicle
				std::stringstream ss;
				ss << ++k;
				for (auto itr=stations.begin(); itr!=stations.end(); ++itr) {
					if (itr->getId() == st_id) {

						vehs->emplace_back(k, ss.str(), itr->getPosition(), 1, amod::VehicleStatus::FREE);
					}
				}
			}
		}
		std::cout << vehs->size() << " vehicles created from the file." << std::endl;
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
