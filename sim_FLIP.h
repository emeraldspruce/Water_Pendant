#ifndef FLIP_SIMULATION_H
#define FLIP_SIMULATION_H

#include "Math.h"



static constexpr int PARTICLE_NUM      = 75;
static constexpr int DIAMETER          = 15;
static constexpr int WATER_DENSITY     = 1000;
static constexpr int SIZE_OF_BIT_STATE = 6;
static constexpr float TIMESTEP        = 3.125e-8;



struct LEDMapping
    {
    char bitStateIndex;
    long bitStateOR;
    };



static constexpr LEDMapping LEDMap[DIAMETER][DIAMETER] = 
    {
    {{0xF, 0}, {0xF, 0}, {0xF, 0}, {0xF, 0}, {0xF, 0}, {0, 1u<<0}, {0, 1u<<1}, {0, 1u<<2}, {0, 1u<<3}, {0, 1u<<4}, {0xF, 0}, {0xF, 0}, {0xF, 0}, {0xF, 0}, {0xF, 0},},
    {{0xF, 0}, {0xF, 0}, {0xF, 0}, {0, 1u<<5}, {0, 1u<<6}, {0, 1u<<7}, {0, 1u<<8}, {0, 1u<<9}, {0, 1u<<10}, {0, 1u<<11}, {0, 1u<<12}, {0, 1u<<13}, {0xF, 0}, {0xF, 0}, {0xF, 0},},
    {{0xF, 0}, {0xF, 0}, {0, 1u<<14}, {0, 1u<<15}, {0, 1u<<16}, {0, 1u<<17}, {0, 1u<<18}, {0, 1u<<19}, {0, 1u<<20}, {0, 1u<<21}, {0, 1u<<22}, {0, 1u<<23}, {0, 1u<<24}, {0xF, 0}, {0xF, 0},},
    {{0xF, 0}, {0, 1u<<25}, {0, 1u<<26}, {0, 1u<<27}, {0, 1u<<28}, {0, 1u<<29}, {0, 1u<<30}, {0, 1u<<31}, {1, 1u<<0}, {1, 1u<<1}, {1, 1u<<2}, {1, 1u<<3}, {1, 1u<<4}, {1, 1u<<5}, {0xF, 0},},
    {{0xF, 0}, {1, 1u<<6}, {1, 1u<<7}, {1, 1u<<8}, {1, 1u<<9}, {1, 1u<<10}, {1, 1u<<11}, {1, 1u<<12}, {1, 1u<<13}, {1, 1u<<14}, {1, 1u<<15}, {1, 1u<<16}, {1, 1u<<17}, {1, 1u<<18}, {0xF, 0},},
    {{1, 1u<<19}, {1, 1u<<20}, {1, 1u<<21}, {1, 1u<<22}, {1, 1u<<23}, {1, 1u<<24}, {1, 1u<<25}, {1, 1u<<26}, {1, 1u<<27}, {1, 1u<<28}, {1, 1u<<29}, {1, 1u<<30}, {1, 1u<<31}, {2, 1u<<0}, {2, 1u<<1},},
    {{2, 1u<<2}, {2, 1u<<3}, {2, 1u<<4}, {2, 1u<<5}, {2, 1u<<6}, {2, 1u<<7}, {2, 1u<<8}, {2, 1u<<9}, {2, 1u<<10}, {2, 1u<<11}, {2, 1u<<12}, {2, 1u<<13}, {2, 1u<<14}, {2, 1u<<15}, {2, 1u<<16},},
    {{2, 1u<<17}, {2, 1u<<18}, {2, 1u<<19}, {2, 1u<<20}, {2, 1u<<21}, {2, 1u<<22}, {2, 1u<<23}, {2, 1u<<24}, {2, 1u<<25}, {2, 1u<<26}, {2, 1u<<27}, {2, 1u<<28}, {2, 1u<<29}, {2, 1u<<30}, {2, 1u<<31},},
    {{3, 1u<<0}, {3, 1u<<1}, {3, 1u<<2}, {3, 1u<<3}, {3, 1u<<4}, {3, 1u<<5}, {3, 1u<<6}, {3, 1u<<7}, {3, 1u<<8}, {3, 1u<<9}, {3, 1u<<10}, {3, 1u<<11}, {3, 1u<<12}, {3, 1u<<13}, {3, 1u<<14},},
    {{3, 1u<<15}, {3, 1u<<16}, {3, 1u<<17}, {3, 1u<<18}, {3, 1u<<19}, {3, 1u<<20}, {3, 1u<<21}, {3, 1u<<22}, {3, 1u<<23}, {3, 1u<<24}, {3, 1u<<25}, {3, 1u<<26}, {3, 1u<<27}, {3, 1u<<28}, {3, 1u<<29},},
    {{0xF, 0}, {3, 1u<<30}, {3, 1u<<31}, {4, 1u<<0}, {4, 1u<<1}, {4, 1u<<2}, {4, 1u<<3}, {4, 1u<<4}, {4, 1u<<5}, {4, 1u<<6}, {4, 1u<<7}, {4, 1u<<8}, {4, 1u<<9}, {4, 1u<<10}, {0xF, 0},},
    {{0xF, 0}, {4, 1u<<11}, {4, 1u<<12}, {4, 1u<<13}, {4, 1u<<14}, {4, 1u<<15}, {4, 1u<<16}, {4, 1u<<17}, {4, 1u<<18}, {4, 1u<<19}, {4, 1u<<20}, {4, 1u<<21}, {4, 1u<<22}, {4, 1u<<23}, {0xF, 0},},
    {{0xF, 0}, {0xF, 0}, {4, 1u<<24}, {4, 1u<<25}, {4, 1u<<26}, {4, 1u<<27}, {4, 1u<<28}, {4, 1u<<29}, {4, 1u<<30}, {4, 1u<<31}, {5, 1u<<0}, {5, 1u<<1}, {5, 1u<<2}, {0xF, 0}, {0xF, 0},},
    {{0xF, 0}, {0xF, 0}, {0xF, 0}, {5, 1u<<3}, {5, 1u<<4}, {5, 1u<<5}, {5, 1u<<6}, {5, 1u<<7}, {5, 1u<<8}, {5, 1u<<9}, {5, 1u<<10}, {5, 1u<<11}, {0xF, 0}, {0xF, 0}, {0xF, 0},},
    {{0xF, 0}, {0xF, 0}, {0xF, 0}, {0xF, 0}, {0xF, 0}, {5, 1u<<12}, {5, 1u<<13}, {5, 1u<<14}, {5, 1u<<15}, {5, 1u<<16}, {0xF, 0}, {0xF, 0}, {0xF, 0}, {0xF, 0}, {0xF, 0},},
    };



// Represents a 2D vector
template <typename T>
struct Vec2 
    {
    T x, y;
    Vec2() noexcept = default;
    Vec2(T x, T y) noexcept : x(x), y(y) {}
    };

// MAYBE NOT NEEDED. CHECK THE PRESSURE SOLVER   
inline float absf(float v) noexcept { return v < 0 ? -v : v; }
template<typename T>
inline T max2(T a, T b) noexcept { return a > b ? a : b; }



// Represents the return value of the simulator where the on/off state of each LED is stored.
struct bitsState 
    {
    long bits[SIZE_OF_BIT_STATE]; // 6 * 1u<<5 = 192 bits max

    bitsState() noexcept
        {
        for (unsigned long i = 0; i < SIZE_OF_BIT_STATE; i++)
            bits[i] = 0;
        }

    bitsState(const bitsState& other) noexcept
        {
        for (unsigned long i = 0; i < SIZE_OF_BIT_STATE; i++)
            bits[i] = other.bits[i];
        }

    // Returns a 1u<<5 bit section of the state
    long operator[](unsigned long index) const
        {
        return bits[index];
        }

    // Updates the current bits to that of the other state 
    void operator=(const bitsState& other) noexcept
        {
        for (unsigned long i = 0; i < SIZE_OF_BIT_STATE; i++)
            bits[i] = other.bits[i];
        }

    // Performs a bitwise and
    bitsState operator&(const bitsState& other) const noexcept
        {
        bitsState newBits;
        for (unsigned long i = 0; i < SIZE_OF_BIT_STATE; i++) 
            newBits.bits[i] = bits[i] & other[i];
        return newBits;
        }

    // Performs a bitwise or
    bitsState operator|(const bitsState other) const noexcept
        {
        bitsState newBits;
        for (unsigned long i = 0; i < SIZE_OF_BIT_STATE; i++) 
            newBits.bits[i] = bits[i] | other[i];
        return newBits;
        }
    };



// Represents a particle in the simulation.
struct Particle // 16B
    {
    float position[2]; // 0: x-position, 1: y-position
    float velocity[2];  // 0: x-velocity, 1: y-position
    // Weight = WATER_DENSITY

    Particle() noexcept = default;

    /**
     * @param pos1 The x-position of the particle
     * @param pos2 The y-position of the particle
     * @param vel1 The x-velocity of the particle
     * @param vel2 The y-velocity of the particle
     */
    Particle(float pos1, float pos2, float vel1, float vel2) noexcept
        {
        position[0] = pos1;
        position[1] = pos2;
        velocity[0] = vel1;
        velocity[1] = vel2;
        }

    inline void setPosX(float x) noexcept
        {
        position[0] = x;
        }
    
    inline void setPosY(float y) noexcept
        {
        position[1] = y;
        }

    inline float getPosX() const noexcept
        {
        return position[0];
        }
    
    inline float getPosY() const noexcept
        {
        return position[1];
        }   
    
    inline void setVelX(float x) noexcept
        {
        velocity[0] = x;
        }

    inline void setVelY(float y) noexcept
        {
        velocity[1] = y;
        }

    inline float getVelX() const noexcept
        {
        return velocity[0];
        }

    inline float getVelY() const noexcept
        {
        return velocity[1];
        }
    };




// Represents a single cell in the simulation grid.
// Each x and y position is determined by the location in the Grid
struct GridCell { // 20B
    float velocity[2]; // 0: x-velocity, 1: y-position
    float divergence; // Divergence of the cell
    float pressure; // Pressure of the cell
    signed short weight;  // Weight for velocity normalization

    GridCell() noexcept = default;

    inline float getVelX() const noexcept
        {
        return velocity[0];
        }

    inline float getVelY() const noexcept
        {
        return velocity[1];
        }

    inline void setVelX(float x) noexcept
        {
        velocity[0] = x;
        }
    
    inline void setVelY(float y) noexcept
        {
        velocity[1] = y;
        }

    GridCell& operator=(const GridCell& other) noexcept
        {
        velocity[0] = other.velocity[0];
        velocity[1] = other.velocity[1];
        weight = other.weight;
        divergence = other.divergence;
        pressure = other.pressure;
        return *this;
        }
};



// Represents the simulation grid.
struct Grid {
    GridCell cells[DIAMETER][DIAMETER];

    Grid() noexcept = default;

    void clear() noexcept
        {
        for (unsigned short i = 0; i < DIAMETER; ++i)
            for (unsigned short j = 0; j < DIAMETER; ++j)
                {
                GridCell& cell = cells[i][j];
                if (cell.weight > -1)
                    {
                    cell.setVelX(0);
                    cell.setVelY(0);
                    cell.weight = 0;
                    }
                }
        }

    Grid& operator=(const Grid& other) noexcept
        {
        for (unsigned short i = 0; i < DIAMETER; ++i)
            for (unsigned short j = 0; j < DIAMETER; ++j)
                cells[i][j] = other.cells[i][j];
        return *this;
        }

    // Returns a single row
    GridCell* operator[](const unsigned short index) 
        {
        return cells[index];
        }
};



// Performs the FLIP fluid simulation
class SIM
    {
    public:
        SIM();
        void updateStepSize(float dt);
        bitsState step(float gravX, float gravY, float accelX, float accelY);
        void setParticlePosition(int index, float x, float y);

    private:
        float dt = TIMESTEP;
        float gravityInput[2];
        float accelInput[2];
        Grid grid;
        Grid copyGrid;
        Particle particles[PARTICLE_NUM];

        void initWalls();
        void initParticles();
        // Particle -> Grid
        void particlesToGrid();
        void normalizeGrid();
        // Update Grid
        void applyExternalForces();
        void computeDivergence();
        void solvePressure();
        void applyPressureGradient();
        // Grid -> Particle
        Vec2<float> interpolateGridVelocity( Grid& grid, float x, float y ) const;
        void transferVelocityToParticles();
        void advectParticles();
        void enforceParticleBounds();
        // Particles -> LED bit state
        bitsState particlesToBitState();
        
    };

#endif // FLIP_SIMULATION_H