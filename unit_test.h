#include "sim_FLIP.h"
#include <vector>
#include <cassert>
#include <iostream>

class test_FLIP : public SIM
    {
    public:
        test_FLIP() : SIM() {}

        void set_verbose(bool verbose) 
            {
            this->verbose = verbose;
            }

        // Expose protected methods for testing
        using SIM::initWalls;
        using SIM::initParticles;
        using SIM::particlesToGrid;
        using SIM::normalizeGrid;
        using SIM::applyExternalForces;
        using SIM::computeDivergence;
        using SIM::solvePressure;
        using SIM::applyPressureGradient;
        using SIM::interpolateGridVelocity;
        using SIM::transferVelocityToParticles;
        using SIM::advectParticles;
        using SIM::enforceParticleBounds;
        using SIM::particlesToBitState;

        // Unit tests for protected methods
        void test_InitWalls()
            {
            initWalls();

            // Example using std::vector for a 2D array
            std::vector<std::vector<int>> expected_walls = 
                {
                {-1, -1, -1, -1, -1,  0,  0,  0,  0,  0, -1, -1, -1, -1, -1},
                {-1, -1, -1,  0,  0,  0,  0,  0,  0,  0,  0,  0, -1, -1, -1},
                {-1, -1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, -1, -1},
                {-1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, -1},
                {-1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, -1},
                { 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
                { 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
                { 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
                { 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
                { 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
                {-1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, -1},
                {-1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, -1},
                {-1, -1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, -1, -1},
                {-1, -1, -1,  0,  0,  0,  0,  0,  0,  0,  0,  0, -1, -1, -1},
                {-1, -1, -1, -1, -1,  0,  0,  0,  0,  0, -1, -1, -1, -1, -1}
                };

            for (int x = 0; x < DIAMETER; ++x)
                for (int y = 0; y < DIAMETER; ++y)
                    assert(grid.cells[x][y].weight == expected_walls[x][y]);
                    
            std::cout << "test_InitWalls passed!" << std::endl;
            }
        
        void test_initParticles()
            {
            
            }
            
        private:
            bool verbose = false;
    };