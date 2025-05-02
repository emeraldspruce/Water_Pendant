#ifndef FLIP_SIMULATION_H
#define FLIP_SIMULATION_H

#include "Math.h"

static constexpr int PARTICLE_NUM      = 75;
static constexpr int DIAMETER          = 15;
static constexpr int WATER_DENSITY     = 1000;
static constexpr int SIZE_OF_BIT_STATE = 6;
static constexpr float TIMESTEP        = 3.125e-8;



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
    long bits[SIZE_OF_BIT_STATE]; // 6 * 32 = 192 bits max

    bitsState() noexcept
        {
        for (unsigned long i = 0; i < SIZE_OF_BIT_STATE; i++)
            bits[i] = 0;
        }

    // Returns a 32 bit section of the state
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
    bitsState operator&(bitsState other) const noexcept
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



enum class CellType : unsigned char
    {
    FLUID = 0,
    WALL  = 1
    };



// Represents a single cell in the simulation grid.
// Each x and y position is determined by the location in the Grid
struct GridCell { // 20B
    float velocity[2]; // 0: x-velocity, 1: y-position
    float divergence; // Divergence of the cell
    float pressure; // Pressure of the cell
    short weight;  // Weight for velocity normalization
    CellType type; // Type of cell (FLUID or WALL)

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
        type = other.type;
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
                cells[i][j].setVelX(0);
                cells[i][j].setVelY(0);
                cells[i][j].weight = 0;
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

    private:
        float dt = TIMESTEP;
        float gravityInput[2];
        float accelInput[2];
        Grid grid;
        Grid copyGrid;
        Particle particles[PARTICLE_NUM];

        void initWalls();
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