#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "TrajectoryGenerator.h"
#include "cost_functions/BestCostStrategy.h"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  // generator which provide the best trajectory based on cost function
  RoadOptions road_options = {4, 3.0, 48 / 2.237, max_s, 10 / 2.237, 0.02};
  TrajectoryGenerator generator(road_options);
  BestCostStrategy best_cost_strategy(road_options);

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy, &generator, &road_options,
               &best_cost_strategy](uWS::WebSocket<uWS::SERVER> ws, char *data,
                                    size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          double prev_size = previous_path_x.size();
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          if (prev_size >= 2) {
            ref_x = previous_path_x[prev_size - 1];
            double ref_x_prev = previous_path_x[prev_size - 2];
            ref_y = previous_path_y[prev_size - 1];
            double ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
            car_speed = sqrt((ref_x - ref_x_prev) * (ref_x - ref_x_prev) +
                             (ref_y - ref_y_prev) * (ref_y - ref_y_prev)) /
                        .02;
          } else {
            end_path_s = car_s;
            end_path_d = car_d;
          }

          std::vector<VehiclePosition> other_vehicles;

          for (const auto &sensor : sensor_fusion) {
            double vx = sensor[3];
            double vy = sensor[4];
            double check_speed = sqrt(vx * vx + vy * vy);
            double v_s = sensor[5];
            v_s += road_options.point_interval * check_speed * prev_size;
            other_vehicles.emplace_back(v_s, sensor[6], check_speed);
          }

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // insert the points from previous trajectory
          for (int i = 0; i < previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // if points less than limit, generate new trajectory
          if (next_x_vals.size() < 50) {
            // init current position in frenet coordinates
            VehiclePosition current_position(end_path_s, end_path_d, car_speed);

            // jenerate the best trajectory
            const auto trajectories =
                generator.generate_trajectories({current_position});
            std::vector<double> best_cost_x;
            std::vector<double> best_cost_y;
            double best_cost = -1;
            int t_index = 0;
            for (const auto &trajectory : trajectories) {
              std::vector<double> x_vals;
              std::vector<double> y_vals;
              // provide spline for correct mapping to x,y coordinates
              tk::spline spline;
              std::vector<double> px;
              std::vector<double> py;
              // to difficult geometry when lane changing
              if (prev_size >=
                  2) //  and
                     //  road_options.getLaneNumber(trajectory.back().getD()) !=
                     //  road_options.getLaneNumber(current_position.getD()))
              {
                px.push_back(previous_path_x[prev_size - 2]);
                py.push_back(previous_path_y[prev_size - 2]);
              }

              px.push_back(ref_x);
              py.push_back(ref_y);
              auto xy =
                  getXY(trajectory.back().getS(), trajectory.back().getD(),
                        map_waypoints_s, map_waypoints_x, map_waypoints_y);
              //      px.push_back(xy[0]);
              //     py.push_back(xy[1]);

              xy =
                  getXY(trajectory.back().getS() + 30, trajectory.back().getD(),
                        map_waypoints_s, map_waypoints_x, map_waypoints_y);
              px.push_back(xy[0]);
              py.push_back(xy[1]);
              xy =
                  getXY(trajectory.back().getS() + 60, trajectory.back().getD(),
                        map_waypoints_s, map_waypoints_x, map_waypoints_y);
              px.push_back(xy[0]);
              py.push_back(xy[1]);

              for (int i = 0; i < px.size(); i++) {

                // shift car reference angle to 0 degrees
                double shift_x = px[i] - ref_x;
                double shift_y = py[i] - ref_y;

                px[i] = (shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw));
                py[i] = (shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw));
              }

              spline.set_points(px, py);

              for (const auto &vehiclePosition : trajectory) {

                double x_point = vehiclePosition.getS() - end_path_s;
                double y_point = spline(x_point);

                double x_ref = x_point;
                double y_ref = y_point;

                x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
                y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

                x_point += ref_x;
                y_point += ref_y;

                x_vals.push_back(x_point);
                y_vals.push_back(y_point);
              }

              double cost = best_cost_strategy.calculateCost(
                  current_position, trajectory, x_vals, y_vals, other_vehicles);
              /*
              {
                  std::cout<<t_index << " " << cost << std::endl;
              }*/
              t_index++;
              if (best_cost < 0 || cost < best_cost) {
                best_cost = cost;
                best_cost_x = std::move(x_vals);
                best_cost_y = std::move(y_vals);
              }
            }

            std::copy(best_cost_x.begin(), best_cost_x.end(),
                      std::back_inserter(next_x_vals));
            std::copy(best_cost_y.begin(), best_cost_y.end(),
                      std::back_inserter(next_y_vals));
          }

          json msgJson;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";
          // std::cout << msg;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        } // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}
