#include "sim_FLIP.h"
#include <cstdlib>
#include <iostream>



inline int getBit(const bitsState& state, int index)
    {
    return state.bits[index>>5]>>(index & 31) & 1u;
    }



void printState(const bitsState& state, int i)
    {
    std::cout << "------------------------------\n";
    std::cout << "State: " << i << "\n";

    unsigned long index = 0;
    unsigned long bound = 0;
    // Row 0
    std::cout << "           ";
    bound = index + 5;
    for (; index < bound; ++index)
        std::cout << ((getBit(state, index)) ? "X" : "-") << " ";
    std::cout << "          \n";

    // Row 1
    std::cout << "       ";
    bound = index + 9;
    for (; index < bound; ++index)
        std::cout << ((getBit(state, index)) ? "X" : "-") << " ";
    std::cout << "      \n";

    // Row 2
    std::cout << "     ";
    bound = index + 11;
    for (; index < bound; ++index)
        std::cout << ((getBit(state, index)) ? "X" : "-") << " ";
    std::cout << "    \n";

    // Row 3
    std::cout << "   ";
    bound = index + 13;
    for (; index < bound; ++index)
        std::cout << ((getBit(state, index)) ? "X" : "-") << " ";
    std::cout << "  \n";

    // Row 4
    std::cout << "   ";
    bound = index + 13;
    for (; index < bound; ++index)
        std::cout << ((getBit(state, index)) ? "X" : "-") << " ";
    std::cout << "  \n";

    // Row 5
    bound = index + 15;
    for (; index < bound; ++index)
        std::cout << " " << ((getBit(state, index)) ? "X" : "-");
    std::cout << "\n";

    // Row 6
    bound = index + 15;
    for (; index < bound; ++index)
        std::cout << " " << ((getBit(state, index)) ? "X" : "-");
    std::cout << "\n";

    // Row 7
    bound = index + 15;
    for (; index < bound; ++index)
        std::cout << " " << ((getBit(state, index)) ? "X" : "-");
    std::cout << "\n";

    // Row 8
    bound = index + 15;
    for (; index < bound; ++index)
        std::cout << " " << ((getBit(state, index)) ? "X" : "-");
    std::cout << "\n";

    // Row 9
    bound = index + 15;
    for (; index < bound; ++index)
        std::cout << " " << ((getBit(state, index)) ? "X" : "-");
    std::cout << "\n";

    // Row 10
    std::cout << "   ";
    bound = index + 13;
    for (; index < bound; ++index)
        std::cout << ((getBit(state, index)) ? "X" : "-") << " ";
    std::cout << "  \n";

    // Row 11
    std::cout << "   ";
    bound = index + 13;
    for (; index < bound; ++index)
        std::cout << ((getBit(state, index)) ? "X" : "-") << " ";
    std::cout << "  \n";

    // Row 12
    std::cout << "     ";
    bound = index + 11;
    for (; index < bound; ++index)
        std::cout << ((getBit(state, index)) ? "X" : "-") << " ";
    std::cout << "    \n";

    // Row 13
    std::cout << "       ";
    bound = index + 9;
    for (; index < bound; ++index)
        std::cout << ((getBit(state, index)) ? "X" : "-") << " ";
    std::cout << "      \n";

    // Row 14
    std::cout << "           ";
    bound = index + 5;
    for (; index < bound; ++index)
        std::cout << ((getBit(state, index)) ? "X" : "-") << " ";
    std::cout << "          \n";
    std::cout << "------------------------------\n\n";
    }



int main(int argc, char** argv)
    {
    SIM sim;
    float timestep = TIMESTEP;
    int seconds = 5;

    if (argc >= 3)
        {
        seconds = std::atoi(argv[2]);
        timestep = 1.0f / std::atoi(argv[1]);
        sim.updateStepSize(static_cast<float>(timestep));
        }
    else if (argc == 2)
        {
        timestep = 1.0f / std::atoi(argv[1]);
        sim.updateStepSize(static_cast<float>(timestep));
        }

    
    const int iterations = seconds / timestep;
    for (int i = 0; i < iterations; ++i)
        {
        bitsState state = sim.step(0, -9.81, 0, 0);
        // Print the state
        printState(state, i);
        }
    }

