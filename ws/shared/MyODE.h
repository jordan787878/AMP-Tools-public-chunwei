#pragma once

#include "Eigen/Dense"
#include <iostream>
#include <cmath>
#include "MyFunctions.h"

#include <fstream>
#include <sstream>
#include <string>
#include <yaml-cpp/yaml.h>

class MyODE{
    public:
        double AU;
        double mu_sun;
        double mu;
        double d;
        double beta;
        std::vector<double> x_init_config;
        double t_end_config;

        double g0; // solar sail acceleration constant [km/s^2]
        double C1;
        double C2;
        double C3;
        double unit_time;
        double unit_length;
        double unit_vel;
        double unit_acc;

        std::vector<std::vector<double>> u_data;

        void init_params(std::string config_file){

            //clean
            u_data.clear();
            
            // Set Constants
            mu_sun = 1.327e+11;
            AU = 1.496e+8;
            
            // Load the YAML file (configuration)
            YAML::Node config = YAML::LoadFile(config_file);
            mu = config["mu"].as<double>();
            d = config["d"].as<double>();
            d = d*AU;
            beta = config["beta"].as<double>();
            t_end_config = config["t_end_config"].as<double>();
            x_init_config = config["x_init_config"].as<std::vector<double>>();

            C1 = 2.0;
            C2 = 0.0;
            C3 = 0.0;

            // Derived constants
            double N = pow(mu_sun/pow(d,3),0.5);
            unit_time = 1/N;
            unit_length = pow( mu/(N*N), 0.33333);
            unit_vel = unit_length / unit_time;
            unit_acc = unit_vel / unit_time;
            std::cout << unit_time << " " << unit_length << " " << unit_vel << " " << unit_acc << "\n";

            double sigma = 1.0;
            double g_guess = cal_g0(sigma, d);
            sigma = (beta/(2*g_guess/unit_acc));
            g0 = cal_g0(sigma, d);
        }

        Eigen::VectorXd get_dxdt(double t, Eigen::VectorXd x, const std::vector<double> u){
            double r1 = x[0]; double r2 = x[1]; double r3 = x[2];
            double v1 = x[3]; double v2 = x[4]; double v3 = x[5];

            double r = pow(r1*r1 + r2*r2 + r3*r3, 0.5);
            std::vector<double> a_SRP = get_a_SRP(u[0], u[1]);
            double ax = a_SRP[0]/unit_acc;
            double ay = a_SRP[1]/unit_acc;
            double az = a_SRP[2]/unit_acc;
            // std::cout << "a_SRP: " << ax << " " << ay << " " << az <<  "\n";

            const int size = x.size();
            Eigen::VectorXd dxdt(size);
            dxdt[0] = v1;
            dxdt[1] = v2;
            dxdt[2] = v3;
            dxdt[3] =  2*v2 + 3*r1 - r1/pow(r,3) + ax;
            dxdt[4] = -2*v1 - r2/pow(r,3) + ay;
            dxdt[5] = -r3 - r3/pow(r,3) + az;
            return dxdt;
        }

        std::vector<double> get_a_SRP(double u1, double u2){
            std::vector<double> a_SRP;
            a_SRP.push_back( g0*cos(u1)*( C1*pow(cos(u1),2) + C2*cos(u1) + C3 ) );
            a_SRP.push_back( g0*cos(u1)*( -(C1*cos(u1)+C2)*sin(u1)*sin(u2) ) );
            a_SRP.push_back( g0*cos(u1)*( -(C1*cos(u1)+C2)*sin(u1)*cos(u2) ) );
            return a_SRP;
        }

        double cal_g0(const double sigma, const double d){
        // input:
        //      sigma: area to mass ratio       [m^2/kg]
        //      d:     asteroid to sun distance [km]

            const double solarflux = 1366;                     // [W/m^2]
            const double dE = 1.496e+11 / 1000;                // earth to sun distance [km]
            const double c = 2.998e+8;                         // speed of light [m/s]
            double result = (sigma*solarflux*pow((dE/d),2)/c)/1000;
            return result;
        }

        // ODE Solver
        // Runge-Kutta solver function
        std::vector<Eigen::VectorXd> rungeKutta(double t0, Eigen::VectorXd x0, const std::vector<double> u, 
                                                double h, const double t_end){

            // std::cout << "Runge-Kutta\n";
            const int size = x0.size();
            std::vector<Eigen::VectorXd> state_traj;

            double t = t0;
            Eigen::VectorXd x(size);
            x = x0;
            state_traj.push_back(x);

            int k = 0;

            while (t < t_end) {
                if(isTerminal(x)){
                    return state_traj;
                }

                // init u at time k
                std::vector<double> uk;
                // get u control from u.csv data
                if(u_data.size() > 0){
                    if(k < u_data.size()){
                        uk = u_data[k];
                    }
                    // exceeding the u.csv data size
                    else{
                        return state_traj;
                    }
                }
                // prespecified u signal
                else{
                    uk = u;
                }

                k = k + 1;

                Eigen::VectorXd w1(size);
                Eigen::VectorXd w2(size);
                Eigen::VectorXd w3(size);
                Eigen::VectorXd w4(size);

                w1 = get_dxdt(t, x, uk);
                w2 = get_dxdt(t, x + 0.5*h*w1, uk);
                w3 = get_dxdt(t, x + 0.5*h*w2, uk);
                w4 = get_dxdt(t, x + 1.0*h*w3, uk);

                x = x + h*(w1 + 2*w2 + 2*w3 + w4)/6;
                t = t + h;
                state_traj.push_back(x);
            }

            // std::cout << "Complete\n";

            return state_traj;
        }

        bool isTerminal(Eigen::VectorXd& s){
            double km2m = 1000.0;
            double x =  abs(s[0]*unit_length*km2m);
            double y =  abs(s[1]*unit_length*km2m);
            double z =  abs(s[2]*unit_length*km2m);
            double r = pow(x*x + y*y + z*z, 0.5);
            double vx = abs(s[3]*unit_vel*km2m);
            double vy = abs(s[4]*unit_vel*km2m);
            double vz = abs(s[5]*unit_vel*km2m);
            // if( std::max({vx, vy, vz}) < 2.0 && std::max({x, y, z}) < 250.0){
            if( std::max({vx, vy, vz}) < 2.0 && r < 250.0){
                return true;
            }
            return false;
        }

        // Validation
        void validate(std::string filename, bool isUdata, const std::string ufilename){
            // To validata: make sure to change
            // x0 (initial states)
            // t_end (final simulation time)
            // beta (the non-dimensional solar sail acceleration)
            // mu (asteroid gravity constant)
            // d (asteroid to sun distance)

            Eigen::VectorXd x0(6);
            x0 << x_init_config[0], x_init_config[1], x_init_config[2], 
                  x_init_config[3], x_init_config[4], x_init_config[5];
            
            const double u1 = 0;
            const double u2 = 0;
            // (Alternatively) read u data
            if(isUdata){
                read_u_control(ufilename);
            }

            const double t_end = t_end_config;
            const double tstep_Traj = 0.0001;

            const std::vector<double> u_control = {u1, u2};
            
            std::vector<Eigen::VectorXd> x_traj;
            x_traj = rungeKutta(0.0, x0, u_control, tstep_Traj, t_end);
            log_vector(x_traj[x_traj.size()-1]);

            write_x_traj(x_traj, filename);
        }

        void read_u_control(const std::string csv_file){
            // Open the .csv file
            std::ifstream file(csv_file);

            if (!file.is_open()) {
                std::cerr << "Failed to open the .csv file." << std::endl;
            }

            std::string line;
            while (std::getline(file, line)) {
                std::vector<double> row;
                std::istringstream ss(line);
                std::string cell;

                while (std::getline(ss, cell, ',')) {
                    try {
                        double value = std::stod(cell);
                        row.push_back(value);
                    } catch (const std::invalid_argument& e) {
                        std::cerr << "Error parsing a non-numeric value: " << cell << std::endl;
                    }
                }
                u_data.push_back(row);
            }

            // Close the .csv file
            file.close();
        }
};