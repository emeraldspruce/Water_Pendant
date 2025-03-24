#include "Math.h"
#include "Providers"

#define PARTICLE_NUM 75 // Water particles
#define DIAMETER 15
#define WATER_DENSITY 1000
#define SIZE_OF_BIT_STATE 6



// Represents the return value of the simulator where the on/off state of each LED is stored.
struct bitsState 
    {
    long bits[SIZE_OF_BIT_STATE]; // 6 * 32 = 192 bits max

    // Returns a 32 bit section of the state
    long operator[](unsigned long index)
        {
        return bits[index];
        }

    // Updates the current bits to that of the other state 
    void operator=(bitsState other)
        {
        for (unsigned long i = 0; i < SIZE_OF_BIT_STATE; i++)
            bits[i] = other.bits[i];
        }

    // Performs a bitwise and
    bitsState operator&(bitsState other)
        {
        bitsState newBits;
        for (unsigned long i = 0; i < SIZE_OF_BIT_STATE; i++) 
            newBits.bits[i] = bits[i] & other[i];
        return newBits;
        }

    // Performs a bitwise or
    bitsState operator|(bitsState other)
        {
        bitsState newBits;
        for (unsigned long i = 0; i < SIZE_OF_BIT_STATE; i++) 
            bits[i] = bits[i] | other[i];
        return newBits;
        }
    };



// Represents a particle in the simulation.
struct Partical
    {
    short position; // bits 0-7 are x, 7-15 are y
    long velocity; // bits 0-15 are x, 16-31 are y

    Partical()
        {
        position = 0; 
        velocity = 0;
        }
    
    /**
     * @param pos1 The x component of position
     * @param pos2 The y component of position
     * @param vel1 The x component of velocity
     * @param vel2 The y component of velocity
     */
    Partical(char pos1, char pos2, short vel1, short vel2)
        {
        position = pos1 | (pos2 << 8);
        velocity = vel1 | (vel2 << 16);
        }
    };



// Represents a single cell in the simulation grid.
struct GridCell {
    long velocity; // Velocity components (x, y)
    short weight;  // Weight for velocity normalization

    GridCell() : velocity(0), weight(0) {}

    inline void velPack(short x, short y) 
        {
        this->velocity = static_cast<long>(x) | (static_cast<long>(y) << 16);
        }

    void operator=(GridCell other)
        {
        this->weight = other.weight;
        velPack((short)other.velocity, other.velocity >> 8);
        }
};



// Represents the simulation grid.
struct Grid {
    GridCell cells[DIAMETER][DIAMETER];

    void clear()
        {
        for (unsigned short i = 0; i < DIAMETER; ++i)
            for (unsigned short j = 0; j < DIAMETER; ++j)
                {
                cells[i][j].velPack(0, 0);
                cells[i][j].weight = 0;
                }
        }

    void operator=(Grid other)
        {
        for (unsigned short i = 0; i < DIAMETER; ++i)
            for (unsigned short j = 0; j < DIAMETER; ++j)
                this->cells[i][j] = other.cells[i][j];
        }
};



// Performs the FLIP fluid simulation
class FLIP
    {
    public:
        FLIP(GravityProvider *gravityProvider, VelocityProvider *velocityProvider);
        void update();

    private:
        short gravityInput[2];
        short velocityInput[2];
        GravityProvider *gravityProvider;
        VelocityProvider *velocityProvider;
        Partical particles[PARTICLE_NUM];

        void updateInputs();
        void updateVelocities();
        void updatePositions();

        void addGravity();
        void makeIncompressible();
        void advect();
    };