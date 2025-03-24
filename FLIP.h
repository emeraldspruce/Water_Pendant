#include "Math.h"
#include "Providers"

#define PARTICLE_NUM 75 // Water particles
#define DIAMETER 15
#define WATER_DENSITY 1000
#define SIZE_OF_BIT_STATE 16



// Represents the return value of the simulator where the on/off state of each LED is stored.
struct bitsState {
    long bits[SIZE_OF_BIT_STATE]; // 10 * 32 = 320 bits max

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
            bits[i] = bits[i] & other[i];
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
    short position[2];
    short velocity[2];

    Partical()
        {
        position[0] = 0;
        position[1] = 0;
        velocity[0] = 0;
        velocity[1] = 0;
        }

    Partical(long pos1, long pos2, long vel1, long vel2)
        {
        position[0] = pos1;
        position[1] = pos2;
        velocity[0] = vel1;
        velocity[1] = vel2;
        }
    };



// Representation of the Grid for storing velocities
struct Grid
    {
    
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