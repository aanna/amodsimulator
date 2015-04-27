#include <iostream>
#include <sstream>
#include <unordered_map>
#include <map>
#include <stdexcept>
#include <fstream>
#include <cmath>
#include <utility>

struct Position{
    double x, y;
};

struct Activity {
    int act_id;
    int cust_id;
    double start_time;
    int source_node_id;
    int dest_node_id;
    int mode_choice;
    double desired_arrival_time;
};

struct Customer {
    int cust_id;
    int home_node_id;
};

void readNodesFile(std::string filename, std::unordered_map<int, Position> *pnodes) {
    auto &nodes = *pnodes;
    std::ifstream in(filename.c_str());
    if (!in.good()) {
        std::cout << "Cannot read in " << filename << std::endl;
        throw std::runtime_error("Cannot read in file");
    } else {
        // skip first line
        std::string header_line;
        getline(in, header_line);
    }



    while (in.good()) {
        Position p;
        int id;
        std::string name;
        in >> id >> name >> p.x >> p.y;
        if (id && in.good()) {
            //std::cout << id << std::endl;
            nodes[id] = p;
        }
    }

    // std::cout.precision(10);
    // for (auto itr=nodes.begin(); itr!=nodes.end(); ++itr) {
    //     std::cout << itr->first << ": " << itr->second.x << ", " << 
    //         itr->second.y << std::endl;
    // }

    return;
}

double convertTime(double smtime) {
    double hr = floor(smtime);
    double mins = smtime - hr;
    if (mins < 0.30) {
        // first half hour
        return hr*60*60;
    } else {
        // second half hour
        return hr*60*60 + 30*60;
    }
}

void readActivityFile(std::string filename, 
    std::multimap<double, Activity> *pactivities, 
    std::unordered_map<std::string, Customer> *pcustomers,
    std::unordered_map<std::string, int> *pmode_choices) {
    auto &acts = *pactivities;
    auto &custs = *pcustomers;
    auto &modes = *pmode_choices;

    std::ifstream in(filename.c_str());
    if (!in.good()) {
        std::cout << "Cannot read in " << filename << std::endl;
        throw std::runtime_error("Cannot read in file");
    } else {
        // skip first line
        std::string header_line;
        getline(in, header_line);
    }

    int act_id = 1; //the booking id
    int cust_id = 1;
    int mode_id = 1;
    while (in.good()) {
        std::string sid;
        double start_time;
        int source_node_id;
        int dest_node_id;
        std::string smode;
        double desired_arrival_time;

        in >> sid >> start_time >> source_node_id >> dest_node_id >> smode >> desired_arrival_time;
        
        if (sid != "" && in.good()) {
            //std::cout << sid << std::endl;

            // create a new activity
            Activity act;
            act.act_id = act_id++;
            act.source_node_id = source_node_id;
            act.dest_node_id = dest_node_id;

            // set start time
            act.start_time = convertTime(start_time) + rand()%30;


            // convert desired end time
            if (start_time == desired_arrival_time) {
                act.desired_arrival_time = act.start_time + rand()%30; 
            } else {
                act.desired_arrival_time = convertTime(desired_arrival_time) + rand()%30; 
            }
      
            // convert mode choice
            auto mitr = modes.find(smode);
            if (mitr == modes.end()) {
                act.mode_choice = mode_id++;
                modes[smode] = act.mode_choice;
            } else {
                act.mode_choice = mitr->second;
            }

            // try to locate this customer id
            auto itr = custs.find(sid);
            if (itr == custs.end()) {
                // add this customer
                Customer c;
                c.cust_id = cust_id++;
                c.home_node_id = source_node_id; //always assume the first trip a person makes is from the home
                custs[sid] = c;
                act.cust_id = c.cust_id;
            } else {
                // get the customer id;
                act.cust_id = itr->second.cust_id;
            }

            // add to our map
            acts.insert( std::pair<double, Activity>(act.start_time, act) );
        }
    }    

}


void outputActivities(std::string filename, std::multimap<double, Activity> *pactivities) 
{
    auto &acts = *pactivities;
    std::ofstream out(filename.c_str());
    if (!out.good()) {
        std::cout << "Cannot open output file" << filename << std::endl;
        throw std::runtime_error("Cannot open output file");
    }
    out.precision(10);
    for (auto itr=acts.begin(); itr!=acts.end(); ++itr) {
        Activity act = itr->second;
        out << act.act_id
            << " " << act.cust_id
            << " " << act.start_time
            << " " << act.source_node_id
            << " " << act.dest_node_id
            << " " << act.mode_choice
            << " " << act.desired_arrival_time
            << std::endl;
    }

    out.close();

}

void outputDataFile(std::string filename, 
    std::unordered_map<int, Position> *pnodes,
    std::multimap<double, Activity> *pactivities, 
    std::unordered_map<std::string, Customer> *pcustomers,
    std::unordered_map<std::string, int> *pmode_choices) 
{
    auto &nodes = *pnodes;
    auto &acts = *pactivities;
    auto &custs = *pcustomers;
    auto &modes = *pmode_choices;


    std::ofstream out(filename.c_str());
    if (!out.good()) {
        std::cout << "Cannot open output file" << filename << std::endl;
        throw std::runtime_error("Cannot open output file");
    }


    out.precision(10);
    
    //output modes
    out << modes.size() << std::endl;
    std::cout << "Writing " << modes.size() << " modes" << std::endl;
    for (auto mitr = modes.begin(); mitr!=modes.end(); ++mitr) {
        out << mitr->second << " " << mitr->first << std::endl;
    }

    // output nodes
    out << nodes.size() << std::endl;
    std::cout << "Writing " << nodes.size() << " nodes" << std::endl;
    for (auto nitr = nodes.begin(); nitr!=nodes.end(); ++nitr) {
        out << nitr->first << " " << nitr->second.x << " " << nitr->second.y << std::endl;
    }

    // output customers
    out << custs.size() << std::endl;
    std::cout << "Writing " << custs.size() << " customers" << std::endl;
    for (auto citr = custs.begin(); citr!=custs.end(); ++citr) {
        out << citr->second.cust_id << " " << citr->second.home_node_id << std::endl;
    }

    //output activities
    out << acts.size() << std::endl;
    std::cout << "Writing " << acts.size() << " activities" << std::endl;
    for (auto itr=acts.begin(); itr!=acts.end(); ++itr) {
        Activity act = itr->second;
        out << act.act_id
            << " " << act.cust_id
            << " " << act.start_time
            << " " << act.source_node_id
            << " " << act.dest_node_id
            << " " << act.mode_choice
            << " " << act.desired_arrival_time
            << std::endl;
    }

    out.close();

}

void outputNodes(std::string filename, 
    std::unordered_map<int, Position> *pnodes) 
{
    auto &nodes = *pnodes;

    std::ofstream out(filename.c_str());
    if (!out.good()) {
        std::cout << "Cannot open output file" << filename << std::endl;
        throw std::runtime_error("Cannot open output file");
    }


    out.precision(10);

    // output nodes
    std::cout << "Writing " << nodes.size() << " nodes" << std::endl;
    for (auto nitr = nodes.begin(); nitr!=nodes.end(); ++nitr) {
        out << nitr->first << " " << nitr->second.x << " " << nitr->second.y << std::endl;
    }
    out.close();

}



int main(int argc, char **argv) {
    if (argc != 4) {
        std::cout << "Usage: converToIds nodes_filename activity_filename output_filename" << std::endl;
        return 1;
    }

    srand(0);

    std::unordered_map<int, Position> nodes;
    readNodesFile(argv[1], &nodes);

    std::multimap<double, Activity> activities;
    std::unordered_map<std::string, Customer> customers;
    std::unordered_map<std::string, int> mode_choices;
    readActivityFile(argv[2], &activities, &customers, &mode_choices);

    for (auto mitr = mode_choices.begin(); mitr!=mode_choices.end(); ++mitr) {
        std::cout << mitr->first << ": " << mitr->second << std::endl;
    }

    //outputActivities(argv[3], &activities);
    outputNodes("nodes.txt", &nodes); //for debugging
    outputDataFile(argv[3], &nodes, &activities, &customers, &mode_choices);


    // std::ifstream in(argv[2]);
    // if (!in.good()) {
    //     std::cout << "Cannot read in " << argv[1] << std::endl;
    //     return -1;
    // }


    // //while (!in.good) {
    // for (int i = 0; i<10; ++i) 
    //     std::string sid;
    //     double t;
    //     int source_node;
    //     int dest_node;
    //     std::string smode;
    //     double desired_arrival_time;
    // }

    // // output
    // std::ofstream out(argv[3]);
    // if (!out.good()) {
    //     std::cout << "Cannot output to " << argv[2] << std::endl;
    //     return -1;
    // }


}