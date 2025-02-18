#include <chrono>
#include <cstdint>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/info/info.h>
#include <iostream>
#include <future>
#include <memory>
#include <thread>

#include <sys/types.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <memory.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <errno.h>
#include <stdlib.h>
#include <iostream>

using namespace mavsdk;
using namespace std::this_thread; // sleep_for, sleep_until
using namespace std::chrono; // nanoseconds, system_clock, seconds
using std::chrono::seconds;
using std::this_thread::sleep_for;

#define SIMULATION 1

#if SIMULATION
    #define DEST_IP "127.0.0.1"
    //#define DEST_IP "192.168.50.188"
#else
    #define DEST_IP "192.168.50.188"
    #define DEST_IP "192.168.50.195"
#endif 

#define PORT 8080

struct Waypoint {
    Offboard::PositionNedYaw position; // Position data
    double radius;                     // Radius for arrival condition

    // Constructor for Waypoint
    Waypoint(Offboard::PositionNedYaw pos, double rad) : position(pos), radius(rad) {}
};

// Struct to hold all telemetry data
struct TelemetryData {
    mavsdk::Telemetry::Position position;
    mavsdk::Telemetry::VelocityNed velocity;
    //mavsdk::Telemetry::Imu imu;
    mavsdk::Telemetry::EulerAngle euler;
};

// Class to manage and store telemetry data
class TelemetryDataCollector {
private:
    TelemetryData data;
    std::mutex data_mutex; // Mutex to ensure thread-safe access to data
public:
    // Start subscribing to telemetry data and storing it
    void start_collecting(Telemetry &telemetry) {

        telemetry.subscribe_position([this](Telemetry::Position position) {
            std::lock_guard<std::mutex> lock(data_mutex);
            data.position = position;
        });

        // Subscribe to velocity updates (in NED frame)
        telemetry.subscribe_velocity_ned([this](Telemetry::VelocityNed velocity) {
            std::lock_guard<std::mutex> lock(data_mutex);
            data.velocity = velocity;
        });

        /*telemetry.subscribe_attitude_euler([this](mavsdk::Telemetry::Imu imu) {
            std::lock_guard<std::mutex> lock(data_mutex);
            data.imu = imu;
        });*/
        
        telemetry.subscribe_attitude_euler([this](mavsdk::Telemetry::EulerAngle euler) {
            std::lock_guard<std::mutex> lock(data_mutex);
            data.euler = euler;
        });
    }

    // Function to safely get the latest telemetry data
    TelemetryData get_latest_data() {
        std::lock_guard<std::mutex> lock(data_mutex);
        return data;
    }
};

void usage(const std::string& bin_name){
    std::cerr << "Usage : " << bin_name << " <connection_url>\n"
              << "Connection URL format should be :\n"
              << " For TCP : tcp://[server_host][:server_port]\n"
              << " For UDP : udp://[bind_host][:bind_port]\n"
              << " For Serial : serial:///path/to/serial/dev[:baudrate]\n"
              << "For example, to connect to the simulator use URL: udpin://0.0.0.0:14540\n";
}

double calculate_2D_distance(double current_north, double current_east, double obj_north, double obj_east) {
    return sqrt(pow((current_north-obj_north), 2) + pow((current_east-obj_east), 2));
}

bool goto_waypoint(Offboard& offboard, Telemetry& telemetry, const Waypoint currPoint, const float vector, const float offset, const bool direction){
    /*double calculate_2D_distance = calculate_radial_distance(
        telemetry.position_velocity_ned().position.north_m,
        telemetry.position_velocity_ned().position.east_m,
        currPoint.position.north_m,
        currPoint.position.east_m
    );*/

    if(direction == true && ((vector*telemetry.position_velocity_ned().position.east_m + offset) > telemetry.position_velocity_ned().position.north_m)){
        std::cout << "Arrived at waypoint \n";
        return true;
    }
    else if(direction == false && (vector*telemetry.position_velocity_ned().position.east_m + offset) < telemetry.position_velocity_ned().position.north_m){ 
        std::cout << "Arrived at waypoint \n";
        return true;
    }
    else{
        std::cout << "Not yet at waypoint";
        return false;
    }

    /*
    if (calculate_2D_distance <= currPoint.radius) {
        std::cout << "Arrived at waypoint \n";
        return true;
    } else {
        std::cout << "Not yet at waypoint ,distance: " << calculate_2D_distance << " meters\n";
        return false;
    }*/
}

std::vector<double> findIntersection(double m1, double c1, double m2, double c2) {
    // Check if the lines are parallel
    if (m1 == m2) {
        return {};
    }

    // Calculate the intersection point
    double x = (c2 - c1) / (m1 - m2);
    double y = m1 * x + c1;

    return {x, y};
}

int main(int argc, char** argv){
    if (argc != 2) {
        usage(argv[0]);
        return 1;
    }

    Mavsdk mavsdk{Mavsdk::Configuration{Mavsdk::ComponentType::GroundStation}};
    ConnectionResult connection_result = mavsdk.add_any_connection(argv[1]);
    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << '\n';
        return 1;
    }

    std::optional<std::shared_ptr<mavsdk::System>> system = mavsdk.first_autopilot(3.0);
    if (!system) {
        std::cerr << "Timed out waiting for system\n";
        return 1;
    }

    // Instantiate plugins.
    auto telemetry = Telemetry{system.value()};
    auto action = Action{system.value()};
    auto offboard = Offboard{system.value()};

    TelemetryDataCollector data_collector;

    // We want to listen to the altitude of the drone at 1 Hz.
    const auto set_rate_result = telemetry.set_rate_position(1.0);
    if (set_rate_result != Telemetry::Result::Success) {
        std::cerr << "Setting rate failed: " << set_rate_result << '\n';
        return 1;
    }

    // Check until vehicle is ready to arm
    bool is_gyrometer_calibration_ok;
    bool is_accelerometer_calibration_ok;
    bool is_magnetometer_calibration_ok;
    bool is_local_position_ok;
    bool is_global_position_ok;
    bool is_home_position_ok;
    bool is_armable;
    while (!telemetry.health_all_ok()) {
        std::cout << "Vehicle not ready to arm. Checking individual systems:\n";

        is_gyrometer_calibration_ok = telemetry.health().is_gyrometer_calibration_ok;
        is_accelerometer_calibration_ok = telemetry.health().is_accelerometer_calibration_ok;
        is_magnetometer_calibration_ok = telemetry.health().is_magnetometer_calibration_ok;
        is_local_position_ok = telemetry.health().is_local_position_ok;
        is_global_position_ok = telemetry.health().is_global_position_ok;
        is_home_position_ok = telemetry.health().is_home_position_ok;
        is_armable = telemetry.health().is_armable;

        std::cout << "Gyrometer calibration: " << (is_gyrometer_calibration_ok ? "OK" : "FAIL") << '\n';
        std::cout << "Accelerometer calibration: " << (is_accelerometer_calibration_ok ? "OK" : "FAIL") << '\n';
        std::cout << "Magnetometer calibration: " << (is_magnetometer_calibration_ok ? "OK" : "FAIL") << '\n';
        std::cout << "Local position: " << (is_local_position_ok ? "OK" : "FAIL") << '\n';
        std::cout << "Global position: " << (is_global_position_ok ? "OK" : "FAIL") << '\n';
        std::cout << "Home position: " << (is_home_position_ok ? "OK" : "FAIL") << '\n';
        std::cout << "Armable: " << (is_armable ? "YES" : "NO") << '\n';

        std::this_thread::sleep_for(1s);
    }

    //initialize the UDP port
    int result = 0;
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr)); 

    servaddr.sin_family = AF_INET; 
    servaddr.sin_addr.s_addr = inet_addr(DEST_IP);
    servaddr.sin_port = htons(PORT); 

    //The RC should be in the same state as the code so this is enforced
    if(SIMULATION == 0){
        while(telemetry.flight_mode() != Telemetry::FlightMode::Offboard || !telemetry.armed()){
            if(telemetry.flight_mode() != Telemetry::FlightMode::Offboard && !telemetry.armed()){
                std::cout << "Vehicle is not armed nor is it in offboard mode. Please arm the vehicle and switch modes.\n";
            }
            else if (!telemetry.armed()) {
                std::cout << "Vehicle is not armed. Please arm the vehicle.\n";
            }
            else if (telemetry.flight_mode() != Telemetry::FlightMode::Offboard) {
                std::cout << "Vehicle is not in Offboard mode. Please switch to Offboard mode.\n";
            }
            offboard.set_position_ned(Offboard::PositionNedYaw {0, 0, 0, 0});
        }
    }

    const float takeoff_altitude = 15.0;
    const double radius = 15.0;
    const double height = -15.0; //NED coordinate use negative height values
    const double width = 50.0;
    const double lenght = 100.0;
    const float initial_Yaw = telemetry.attitude_euler().yaw_deg;

    std::cout << "initial angle: " << initial_Yaw << "\n";

    std::vector<Waypoint> waypoints_vec;
    //Add a path to be followed
    // waypoints are clockwise
    /*
    waypoints_vec.push_back(Waypoint(Offboard::PositionNedYaw {float(width*cos(initial_Yaw*M_PI/180)), float(lenght*sin(initial_Yaw*M_PI/180)), height, initial_Yaw}, radius));
    waypoints_vec.push_back(Waypoint(Offboard::PositionNedYaw {float(width*cos(initial_Yaw*M_PI/180 + M_PI/2)), float(lenght*sin(initial_Yaw*M_PI/180)), height, float(initial_Yaw+M_PI*3/4)}, radius));
    waypoints_vec.push_back(Waypoint(Offboard::PositionNedYaw {float(width*cos(initial_Yaw*M_PI/180 + M_PI/2)), float(lenght*sin(initial_Yaw*M_PI/180 + M_PI/2)), height, float(initial_Yaw+M_PI*3/4)}, radius));
    waypoints_vec.push_back(Waypoint(Offboard::PositionNedYaw {float(width*cos(initial_Yaw*M_PI/180)), float(lenght*sin(initial_Yaw*M_PI/180 + M_PI/2)), height, float(initial_Yaw+M_PI)}, radius));
    */

    // waypoints are counter clockwise
    waypoints_vec.push_back(Waypoint(Offboard::PositionNedYaw {float(width*cos(initial_Yaw*M_PI/180)), float(lenght*sin(initial_Yaw*M_PI/180)), height, initial_Yaw}, radius));
    waypoints_vec.push_back(Waypoint(Offboard::PositionNedYaw {-float(width*cos(initial_Yaw*M_PI/180 + M_PI/2)), float(lenght*sin(initial_Yaw*M_PI/180)), height, float(initial_Yaw+M_PI*3/4)}, radius));
    waypoints_vec.push_back(Waypoint(Offboard::PositionNedYaw {-float(width*cos(initial_Yaw*M_PI/180 + M_PI/2)), float(lenght*sin(initial_Yaw*M_PI/180 + M_PI/2)), height, float(initial_Yaw+M_PI*3/4)}, radius));
    waypoints_vec.push_back(Waypoint(Offboard::PositionNedYaw {float(width*cos(initial_Yaw*M_PI/180)), float(lenght*sin(initial_Yaw*M_PI/180 + M_PI/2)), height, float(initial_Yaw+M_PI)}, radius));
    

    TelemetryData latest_data;
    bool arrived = false;
    char* msg;
    size_t msg_length;
    // Start collecting telemetry data
    data_collector.start_collecting(telemetry);

    // Arm vehicle
    std::cout << "Arming...\n";
    const Action::Result arm_result = action.arm();
    if (arm_result != Action::Result::Success) {
        std::cerr << "Arming failed: " << arm_result << '\n';
        return 1;
    }

    // Take off to takeoff_altitude
    action.set_takeoff_altitude(takeoff_altitude);
    std::cout << "Taking off to "<< takeoff_altitude << " meters, current altitude: " << telemetry.position().relative_altitude_m << "\n";
    const Action::Result takeoff_result = action.takeoff();
    if (takeoff_result != Action::Result::Success) {
        std::cerr << "Takeoff failed: " << takeoff_result << '\n';
        return 1;
    }

    // Wait until the drone reaches takeoff_altitude meters (a != a checks if its NaN)
    while ((telemetry.position().relative_altitude_m < takeoff_altitude) || (telemetry.position().relative_altitude_m != telemetry.position().relative_altitude_m)) {
        std::cout << "Ascending to " << takeoff_altitude << " m, current altitude: " << telemetry.position().relative_altitude_m << "\n";
        sleep_for(seconds(1));
    }

    int i = 0;
    Waypoint* prevPoint = nullptr;
    Waypoint* currPoint = nullptr;
    Waypoint* nextPoint = nullptr; 
    float plane_vector;
    float plane_offset;
    float waypoint_vector;
    float waypoint_offset;

    offboard.set_position_ned(waypoints_vec[i].position);
    auto offboard_result = offboard.start();
    if (offboard_result != Offboard::Result::Success) {
        std::cerr << "Offboard start failed: " << offboard_result << '\n';
        return 1;
    }
    while(true){
        
        if (i == 0)
            prevPoint = &waypoints_vec[waypoints_vec.size()-1];
        else
            prevPoint = &waypoints_vec[i-1];

        currPoint = &waypoints_vec[i];
    
        if(i == (waypoints_vec.size()-1))
            nextPoint = &waypoints_vec[0];
        else
            nextPoint = &waypoints_vec[i+1];

        plane_vector = (prevPoint->position.north_m - nextPoint->position.north_m)/(prevPoint->position.east_m - nextPoint->position.east_m);
        waypoint_vector = -plane_vector;
        waypoint_offset = currPoint->position.north_m - currPoint->position.east_m*waypoint_vector;

        std::cout << "going to waypoint " << i << "\n";
        offboard.set_position_ned(waypoints_vec[i].position);
        while(true){
            plane_offset = telemetry.position_velocity_ned().position.north_m - telemetry.position_velocity_ned().position.east_m*plane_vector;
            std::vector<double> intersection = findIntersection(plane_vector,plane_offset, waypoint_vector, waypoint_offset);
            double dist = calculate_2D_distance(intersection[1], intersection[0], telemetry.position_velocity_ned().position.north_m, telemetry.position_velocity_ned().position.east_m);
            std::cout << "Current Distance: " << dist << "\n";
            if(dist < 2)
                break;

            //in case the operator goes to manual mode
            if(telemetry.flight_mode() != Telemetry::FlightMode::Offboard){
                while(telemetry.flight_mode() != Telemetry::FlightMode::Offboard){
                    offboard.set_position_ned(waypoints_vec[i].position);
                    std::cout << "Waiting for offboard flight mode\n";
                }
                offboard.set_position_ned(waypoints_vec[i].position);
                offboard_result = offboard.start();
                if (offboard_result != Offboard::Result::Success) {
                    std::cerr << "Offboard start failed: " << offboard_result << '\n';
                    return 1;
                }
            }

            //send data via UDP
            latest_data = data_collector.get_latest_data();
            msg_length = sizeof(latest_data);
            result = sendto(sock, &latest_data, msg_length, MSG_CONFIRM, (const struct sockaddr*)&servaddr, sizeof(servaddr));
        }
        ++i;
        if (i >= waypoints_vec.size())
            i = 0;
    }
    return EXIT_SUCCESS;
}