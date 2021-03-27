#ifndef SRC_ROCKET_MODEL_HPP
#define SRC_ROCKET_MODEL_HPP

#include <fstream>
#include "ros/package.h"

class Rocket {
public:
    float dry_mass;
    float propellant_mass;
    float Isp;
    float minTorque;
    float maxTorque;

    std::vector<float> maxThrust{0, 0, 0};
    std::vector<float> minThrust{0, 0, 0};

    float dry_CM;
    float propellant_CM;
    float total_CM; // Current Cm of rocket, in real time

    float total_length;

    std::vector<float> target_apogee = {0, 0, 0};
    std::vector<float> Cd = {0, 0, 0};
    std::vector<float> surface = {0, 0, 0};
    std::vector<float> drag_coeff = {0, 0, 0};

    std::vector<float> dry_Inertia{0, 0, 0};
    std::vector<float> total_Inertia{0, 0, 0};

    std::vector<float> J_inv{0, 0, 0};

    std::vector<float> full_thrust;
    std::vector<float> full_thrust_time;


    void update_CM(float current_prop_mass) {
        total_CM = total_length - (dry_CM * dry_mass + propellant_CM * current_prop_mass) /
                                  (dry_mass + current_prop_mass); // From aft of rocket

        float new_inertia = dry_Inertia[0] + pow(total_CM - (total_length - propellant_CM), 2) * current_prop_mass;

        total_Inertia[0] = new_inertia;
        total_Inertia[1] = new_inertia;

        J_inv[0] = total_CM / total_Inertia[0];
        J_inv[1] = total_CM / total_Inertia[1];
        J_inv[2] = 1 / total_Inertia[2];
    }


    virtual void init(ros::NodeHandle n) {
        n.getParam("/rocket/minTorque", minTorque);
        n.getParam("/rocket/maxTorque", maxTorque);
        n.getParam("/rocket/maxThrust", maxThrust);
        n.getParam("/rocket/minThrust", minThrust);
        n.getParam("/rocket/Isp", Isp);

        n.getParam("/rocket/dry_mass", dry_mass);
        n.getParam("/rocket/propellant_mass", propellant_mass);

        n.getParam("/rocket/Cd", Cd);
        n.getParam("/rocket/dry_I", dry_Inertia);

        n.getParam("/rocket/dry_CM", dry_CM);
        n.getParam("/rocket/propellant_CM", propellant_CM);

        n.getParam("/environment/apogee", target_apogee);

        std::vector<float> diameter = {0, 0, 0};
        std::vector<float> length = {0, 0, 0};

        int nStage;

        n.getParam("/rocket/diameters", diameter);
        n.getParam("/rocket/stage_z", length);
        n.getParam("/rocket/stages", nStage);

        total_length = length[nStage - 1];

        surface[0] = diameter[1] * total_length;
        surface[1] = surface[0];
        surface[2] = diameter[1] * diameter[1] / 4 * 3.14159;

        float rho_air = 1.225;
        drag_coeff[0] = 0.5 * rho_air * surface[0] * Cd[0];
        drag_coeff[1] = 0.5 * rho_air * surface[1] * Cd[1];
        drag_coeff[2] = 0.5 * rho_air * surface[2] * Cd[2];

        total_Inertia[2] = dry_Inertia[2];

        update_CM(propellant_mass);

        load_motor();
    }

    void load_motor() {
        std::string path = ros::package::getPath("basic_gnc") + "/config/motor_file.txt";

        std::string line;
        std::ifstream myfile(path);
        if (myfile.is_open()) {
            while (getline(myfile, line)) {
                int separator = line.find("\t");
                std::string time_string = line.substr(0, separator);
                std::string thrust_string = line.substr(separator + 1, line.length() - separator);

                full_thrust_time.push_back(std::stof(time_string));
                full_thrust.push_back(std::stof(thrust_string));
            }
            myfile.close();

        } else { ROS_WARN("Didn't find motor file"); }
    }

    float get_full_thrust(float time_thrust) {
        int i;
        for (i = 0; i < (int)full_thrust_time.size(); i++) {
            if (time_thrust < full_thrust_time[i]) {
                break;
            }
        }
        i--;

        float interp_thrust;
        if (time_thrust < full_thrust_time.back())
            interp_thrust = (full_thrust[i] +
                             (time_thrust - full_thrust_time[i]) / (full_thrust_time[i + 1] - full_thrust_time[i]) *
                             (full_thrust[i + 1] - full_thrust[i]));

        else
            interp_thrust = full_thrust_time.back();

        if (time_thrust < 0)
            interp_thrust = 0;

        return interp_thrust;
    }
};

#endif //SRC_ROCKET_MODEL_HPP
