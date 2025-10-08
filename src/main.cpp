#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "include/helpers.h" // Helper functions including coordinate transforms and utility math functions
#include "include/spline.h" // Include the spline header for smooth path generation
#include "json.hpp"
#include "include/polynomial_solver.h" // don't need to use while using spline (better suited for parking areas scenarios)

// For convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
    // Initialize uWebSocket Hub to manage WebSocket communication to simulator
    uWS::Hub h;

    // Vectors to store map waypoints' Cartesian (x,y), Frenet s values, and unit normal vectors (dx, dy)
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

    // Path to CSV file containing pre-mapped waypoints for the track
    string map_file_ = "../data/highway_map.csv";
    // Maximum Frenet s value before track loops back to zero
    double max_s = 6945.554;

    // Initialize lane (middle lane = 1); lane width is uniform 4 meters
    int lane = 1;
    int lane_width = 4;

    // Initial target speed in meters per second; 50 MPH ~ 22.35 m/s (initialized to 0 to accelerate)
    double target_speed = 0.0;

    // Read waypoint map file line by line and extract waypoint data into vectors
    std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);
    string line;
    while (getline(in_map_, line)) {
        std::istringstream iss(line);
        double x, y;
        float s, d_x, d_y;

        // Waypoint file columns: x, y, s, d_x, d_y
        iss >> x >> y >> s >> d_x >> d_y;

        map_waypoints_x.push_back(x);
        map_waypoints_y.push_back(y);
        map_waypoints_s.push_back(s);
        map_waypoints_dx.push_back(d_x);
        map_waypoints_dy.push_back(d_y);
    }

    // Setup event handler for incoming telemetry messages from simulator
    h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
                 &map_waypoints_dx, &map_waypoints_dy, &lane, &lane_width, &target_speed]
                (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                 uWS::OpCode opCode) {
        // Maintain persistent information about ongoing lane changes
        static bool lane_change_in_progress = false;
        static int target_lane = lane; // Target lane for planned lane changes

        // Check for websocket message type and length
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {
            auto s = hasData(data); // Extract JSON data
            if (s != "") {
                auto j = json::parse(s);
                string event = j[0].get<string>();

                // Process telemetry data event
                if (event == "telemetry") {
                    // Extract car localization data (position, orientation, speed)
                    double car_x = j[1]["x"];
                    double car_y = j[1]["y"];
                    double car_s = j[1]["s"];
                    double car_d = j[1]["d"];
                    double car_yaw = j[1]["yaw"];
                    double car_speed = j[1]["speed"];

                    // Extract previously planned path to ensure smooth path continuation
                    auto previous_path_x = j[1]["previous_path_x"];
                    auto previous_path_y = j[1]["previous_path_y"];
                    double end_path_s = j[1]["end_path_s"];
                    double end_path_d = j[1]["end_path_d"];

                    // Extract sensor fusion data to track other vehicles on the road
                    auto sensor_fusion = j[1]["sensor_fusion"];

                    json msgJson;
                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

                    // Path planner parameters: how far ahead and pacing
                    double dist_inc = 0.5;
                    int num_steps = 50;                     // total points to generate for trajectory
                    int rem_path_steps = previous_path_x.size();  // Points remaining from last cycle
                    double t_step = 0.02;                   // 20ms interval per point in trajectory

                    // Initialize vectors to record minimum distance gap ahead/behind per lane (large initial value)
                    vector<double> lane_gap_ahead(3, 1e6);
                    vector<double> lane_gap_behind(3, 1e6);

                    /* 
                    Analyze sensor fusion data:
                    - Predict future car s values taking velocity and time into account
                    - Determine which lane cars belong to based on d Frenet coordinate
                    - Calculate gaps ahead and behind our car in each lane to aid lane decision-making
                    */
                    for (int i = 0; i < sensor_fusion.size(); i++) {
                        double veh_vx = sensor_fusion[i][3];
                        double veh_vy = sensor_fusion[i][4];
                        double veh_s = sensor_fusion[i][5];
                        double veh_d = sensor_fusion[i][6];
                        double veh_speed = sqrt(veh_vx * veh_vx + veh_vy * veh_vy);
                        double dt = rem_path_steps * t_step;
                        // Convert mph to m/s for speed projections
                        double veh_s_future = veh_s + veh_speed * dt * 0.447; 
                        double car_s_future = car_s + target_speed * dt * 0.447;

                        int veh_lane = -1;
                        if (veh_d >= 0 && veh_d <= 12) {
                            veh_lane = int(veh_d / 4.0); // Calculate lane index
                        }
                        if (veh_lane < 0) continue; // Skip if car not detected in any lane

                        double gap_ahead = veh_s_future - car_s_future;
                        double gap_behind = car_s_future - veh_s_future;

                        // Track minimum forward gap per lane
                        if (gap_ahead > 0 && gap_ahead < lane_gap_ahead[veh_lane]) {
                            lane_gap_ahead[veh_lane] = gap_ahead;
                        }
                        // Track minimum rear gap per lane
                        if (gap_behind > 0 && gap_behind < lane_gap_behind[veh_lane]) {
                            lane_gap_behind[veh_lane] = gap_behind;
                        }
                    }

                    // Check if car ahead is within unsafe distance in our current lane
                    bool too_close = lane_gap_ahead[lane] < 30.0;

                    /*
                    Lane change logic:
                    - If not already changing lanes and too close to car ahead:
                      * Attempt safe lane changes to left/right lane based on gaps.
                      * If no safe lane found, decelerate.
                    - If not too close, accelerate until target speed near speed limit.
                    - If lane change in progress, progressively adjust lane and
                      abort if rear gap too small (unsafe).
                    */
                    if (!lane_change_in_progress) {
                        target_lane = lane;

                        if (too_close) {
                            std::cout << "Car too close, considering lane change..." << std::endl;

                            // Check left lane safety and preference
                            if (lane > 0 && lane_gap_ahead[lane - 1] > 30.0 && lane_gap_behind[lane - 1] > 15.0) {
                                target_lane = lane - 1;
                                lane_change_in_progress = true;
                            }
                            // Check right lane safety as alternative
                            else if (lane < 2 && lane_gap_ahead[lane + 1] > 30.0 && lane_gap_behind[lane + 1] > 15.0) {
                                target_lane = lane + 1;
                                lane_change_in_progress = true;
                            }
                            else {
                                std::cout << "No safe lane change found, decelerating..." << std::endl;
                                target_speed -= 0.224; // Gradual deceleration
                            }
                        }
                        else if (target_speed < 49.5) {
                            std::cout << "Accelerating" << std::endl;
                            target_speed += 0.224; // Gradual acceleration
                        }
                        else if (target_speed - 49.5 < 0.01) {
                            // Maintain speed close to speed limit (50 mph)
                            std::cout << "Keeping constant speed at " << target_speed << " m/s" << std::endl;
                            target_speed += 0.224;
                        }
                    }
                    else {
                        // Lane change in progress: ensure no car dangerously close behind in target lane
                        if (lane_gap_behind[target_lane] < 15.0) {
                            std::cout << "Abort lane change! Car too close behind in target lane." << std::endl;
                            lane_change_in_progress = false;
                            target_lane = lane; // Revert lane change due to safety
                        }
                        else {
                            // Gradually shift current lane toward target lane, one step at a time
                            if (lane != target_lane) {
                                if (target_lane > lane) lane++;
                                else lane--;
                            }

                            // Complete lane change once current lane matches target lane
                            if (lane == target_lane) {
                                lane_change_in_progress = false;
                                std::cout << "Lane change completed. Now in lane " << lane << std::endl;
                            }
                        }
                    }

                    /*
                    Generate smooth path waypoints using spline interpolation:
                    - Use last few points from previous path or current vehicle position to ensure smooth transition.
                    - Plan three waypoints spaced 30m, 60m, and 90m ahead in Frenet coordinates with lateral offset for lane center.
                    - Transform points to local car coordinates for spline fitting.
                    - Generate points based on spline ensuring they are spaced to respect target speed.
                    - Transform points back to global coordinate frame for simulator.
                    */
                    double pos_x = car_x;
                    double pos_y = car_y;
                    double pos_angle = deg2rad(car_yaw);

                    vector<double> ptsx;
                    vector<double> ptsy;

                    if (rem_path_steps < 2) {
                        // If not enough previous points, create two points tangent to the car position and yaw
                        double pos_x_prev = pos_x - 5.0 * cos(pos_angle);
                        double pos_y_prev = pos_y - 5.0 * sin(pos_angle);
                        ptsx.push_back(pos_x_prev);
                        ptsy.push_back(pos_y_prev);
                        ptsx.push_back(pos_x);
                        ptsy.push_back(pos_y);
                    }
                    else {
                        // Use last two points from previous path as starting reference to maintain continuity
                        pos_x = previous_path_x[rem_path_steps - 1];
                        pos_y = previous_path_y[rem_path_steps - 1];
                        double pos_x_prev = previous_path_x[rem_path_steps - 2];
                        double pos_y_prev = previous_path_y[rem_path_steps - 2];
                        pos_angle = atan2(pos_y - pos_y_prev, pos_x - pos_x_prev);
                        ptsx.push_back(pos_x_prev);
                        ptsy.push_back(pos_y_prev);
                        ptsx.push_back(pos_x);
                        ptsy.push_back(pos_y);
                    }

                    // Smooth lateral displacement target to avoid abrupt lane changes
                    static double smooth_d = -1.0;
                    if (smooth_d < 0.0) smooth_d = car_d; // Initialize on first run
                    double target_d = (lane_width * 0.5) + (lane * lane_width);
                    double max_d_step_per_cycle = 0.15; // Limit lateral change per iteration for comfort and safety
                    double d_diff = target_d - smooth_d;
                    if (fabs(d_diff) > max_d_step_per_cycle)
                        smooth_d += max_d_step_per_cycle * (d_diff > 0 ? 1.0 : -1.0);
                    else
                        smooth_d = target_d;

                    // Generate three Frenet waypoints spaced at 30m intervals ahead
                    vector<double> next_wp0 = getXY(car_s + 30, smooth_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                    vector<double> next_wp1 = getXY(car_s + 60, smooth_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                    vector<double> next_wp2 = getXY(car_s + 90, smooth_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

                    ptsx.push_back(next_wp0[0]);
                    ptsx.push_back(next_wp1[0]);
                    ptsx.push_back(next_wp2[0]);
                    ptsy.push_back(next_wp0[1]);
                    ptsy.push_back(next_wp1[1]);
                    ptsy.push_back(next_wp2[1]);

                    // Transform all points to vehicle's local coordinate system for spline interpolation
                    for (int i = 0; i < ptsx.size(); i++) {
                        double shift_x = ptsx[i] - pos_x;
                        double shift_y = ptsy[i] - pos_y;
                        ptsx[i] = shift_x * cos(-pos_angle) - shift_y * sin(-pos_angle);
                        ptsy[i] = shift_x * sin(-pos_angle) + shift_y * cos(-pos_angle);
                    }

                    // Ensure spline points have strictly increasing x values to avoid spline errors
                    for (int i = 1; i < ptsx.size(); i++) {
                        if (ptsx[i] <= ptsx[i - 1]) ptsx[i] = ptsx[i - 1] + 0.0001;
                    }

                    // Create spline and set points
                    tk::spline s;
                    s.set_points(ptsx, ptsy);

                    // Start new trajectory with points from previous path for smooth transition
                    for (int i = 0; i < rem_path_steps; i++) {
                        next_x_vals.push_back(previous_path_x[i]);
                        next_y_vals.push_back(previous_path_y[i]);
                    }

                    // Define how far ahead target points will be spaced based on desired speed and time interval
                    double x_add = 0.0;
                    double target_x = 30.0;
                    double target_y = s(target_x);
                    double total_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));

                    // Fill remaining points to achieve full planned trajectory
                    for (int i = 0; i < num_steps - rem_path_steps; i++) {
                        // Calculate spacing based on desired speed
                        double N = total_dist / (target_speed * 0.447 * t_step);
                        double x_point = x_add + (target_x / N);
                        double y_point = s(x_point);
                        x_add = x_point;

                        // Transform back to global coordinates
                        double x_ref = x_point;
                        double y_ref = y_point;

                        x_point = x_ref * cos(pos_angle) - y_ref * sin(pos_angle) + pos_x;
                        y_point = x_ref * sin(pos_angle) + y_ref * cos(pos_angle) + pos_y;

                        // Append next points to trajectory lists
                        next_x_vals.push_back(x_point);
                        next_y_vals.push_back(y_point);
                    }

                    // Package the next trajectory points and send to simulator
                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;
                    auto msg = "42[\"control\"," + msgJson.dump() + "]";
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                } // end telemetry event
            } else {
                // No data: fallback to manual control mode
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });

    // Connection event: log successful WebSocket handshake
    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    // Disconnection event: cleanup and notify
    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    // Start WebSocket server on port 4567
    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }

    // Enter the event loop to process incoming messages
    h.run();
}


