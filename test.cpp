#include <fstream>
#include "sim_FLIP.h"
#include <cstdlib>
#include <iostream>



// the data for a single step from the config file
struct Config_step
    {
    bool stop = 0;
    float accelX = 0;
    float accelY = 0;
    };



inline int getBit(const bitsState& state, int index)
    {
    return state.bits[index>>5]>>(index & 31) & 1u;
    }



void printState(std::ostream& out, const bitsState& state, int i)
    {
    out << "------------------------------\n";
    if (i >= 0) out << "State: " << i << "\n";

    unsigned long index = 0;
    unsigned long bound = 0;
    // Row 0
    out << "           ";
    bound = index + 5;
    for (; index < bound; ++index)
        out << ((getBit(state, index)) ? "X" : "-") << " ";
    out << "          \n";

    // Row 1
    out << "       ";
    bound = index + 9;
    for (; index < bound; ++index)
        out << ((getBit(state, index)) ? "X" : "-") << " ";
    out << "      \n";

    // Row 2
    out << "     ";
    bound = index + 11;
    for (; index < bound; ++index)
        out << ((getBit(state, index)) ? "X" : "-") << " ";
    out << "    \n";

    // Row 3
    out << "   ";
    bound = index + 13;
    for (; index < bound; ++index)
        out << ((getBit(state, index)) ? "X" : "-") << " ";
    out << "  \n";

    // Row 4
    out << "   ";
    bound = index + 13;
    for (; index < bound; ++index)
        out << ((getBit(state, index)) ? "X" : "-") << " ";
    out << "  \n";

    // Row 5
    bound = index + 15;
    for (; index < bound; ++index)
        out << " " << ((getBit(state, index)) ? "X" : "-");
    out << "\n";

    // Row 6
    bound = index + 15;
    for (; index < bound; ++index)
        out << " " << ((getBit(state, index)) ? "X" : "-");
    out << "\n";

    // Row 7
    bound = index + 15;
    for (; index < bound; ++index)
        out << " " << ((getBit(state, index)) ? "X" : "-");
    out << "\n";

    // Row 8
    bound = index + 15;
    for (; index < bound; ++index)
        out << " " << ((getBit(state, index)) ? "X" : "-");
    out << "\n";

    // Row 9
    bound = index + 15;
    for (; index < bound; ++index)
        out << " " << ((getBit(state, index)) ? "X" : "-");
    out << "\n";

    // Row 10
    out << "   ";
    bound = index + 13;
    for (; index < bound; ++index)
        out << ((getBit(state, index)) ? "X" : "-") << " ";
    out << "  \n";

    // Row 11
    out << "   ";
    bound = index + 13;
    for (; index < bound; ++index)
        out << ((getBit(state, index)) ? "X" : "-") << " ";
    out << "  \n";

    // Row 12
    out << "     ";
    bound = index + 11;
    for (; index < bound; ++index)
        out << ((getBit(state, index)) ? "X" : "-") << " ";
    out << "    \n";

    // Row 13
    out << "       ";
    bound = index + 9;
    for (; index < bound; ++index)
        out << ((getBit(state, index)) ? "X" : "-") << " ";
    out << "      \n";

    // Row 14
    out << "           ";
    bound = index + 5;
    for (; index < bound; ++index)
        out << ((getBit(state, index)) ? "X" : "-") << " ";
    out << "          \n";
    out << "------------------------------\n\n";
    }



Config_step getConfigUpdate()
    {
    // Open the config file
    Config_step config_step;
    std::ifstream file("test_config.txt");
    if (!file.is_open())
        {
        std::cerr << "Error opening config file.\n";
        return config_step;
        }
    
    // Read the config files
    std::string line, value;
    while (file >> line >> value)
        {
        if (line == "STOP:")
            config_step.stop = (std::stoi(value) != 0);
        else if (line == "ACCEL_X:")
            config_step.accelX = std::stof(value);
        else if (line == "ACCEL_Y:")
            config_step.accelY = std::stof(value);
        }

    return config_step;
    }



int main(int argc, char** argv)
    {
    SIM sim;
    sim.updateStepSize(TIMESTEP);
    
    bitsState state;
    std::ofstream file("test_output.txt", std::ios::trunc);
    while (true) 
        {
        Config_step config_step = getConfigUpdate();
        if (config_step.stop)
            {
            std::cout << "Stopping simulation.\n";
            break;
            }
        state = sim.step(config_step.accelX, config_step.accelY); // Add current velocity? (change from gravity to velocity because gravity is an acceleration)
        file.seekp(0);
        printState(file, state, -1);
        file.flush();
        }
    file.close();
    std::cout << "Simulation finished.\n";
    }