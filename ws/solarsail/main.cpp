#include "AMPCore.h"

#include "MyRRT.h"
#include "MyODE.h"


int main(int argc, char** argv) {
    amp::RNG::seed(amp::RNG::randiUnbounded());

    // Configuration
    std::vector<std::string> config_files;
    config_files.push_back("/Users/chko1829/src/SolarSailLanding/configs/orbit1_config.yaml");
    config_files.push_back("/Users/chko1829/src/SolarSailLanding/configs/orbit2_config.yaml");
    config_files.push_back("/Users/chko1829/src/SolarSailLanding/configs/orbit3_config.yaml");
    config_files.push_back("/Users/chko1829/src/SolarSailLanding/configs/orbit4_config.yaml");

    MyODE ode;

    // (1,2,3,4) 4 configurtions to choose
    const int config_index = 4;
    YAML::Node config = YAML::LoadFile(config_files[config_index-1]);
    // load control file path
    const std::string u_filename = config["u_filename"].as<std::string>();
    const std::string valid1_filename = config["valid1_filename"].as<std::string>();
    const std::string valid2_filename = config["valid2_filename"].as<std::string>();

    // Uncontrolled simulation
    // ode.init_params(config_files[config_index-1]);
    // ode.validate(valid1_filename, false, "");

    // // Controlled simulation
    // ode.init_params(config_files[config_index-1]);
    // ode.validate(valid2_filename, true, u_filename);

    MyRRT rrtalgo;
    rrtalgo.problem_setup(config_files[config_index-1]);
    std::vector<Eigen::VectorXd> path = rrtalgo.plan();
    write_x_traj(path, "/Users/chko1829/src/SolarSailLanding/file_dump/path.csv");

    std::cout << "[RUN COMPLETE]\n";
    return 0;
}