#ifndef FLIP_SIMULATION_H
#define FLIP_SIMULATION_H

#include "Math.h"
#include "Providers.h"

#define PARTICLE_NUM 75 // Water particles
#define DIAMETER 15
#define WATER_DENSITY 1000
#define SIZE_OF_BIT_STATE 6
#define TIMESTEP 3.125e-8



// Represents the return value of the simulator where the on/off state of each LED is stored.
struct bitsState 
    {
    long bits[SIZE_OF_BIT_STATE]; // 6 * 32 = 192 bits max

    // Returns a 32 bit section of the state
    long operator[](unsigned long index) const
        {
        return bits[index];
        }

    // Updates the current bits to that of the other state 
    void operator=(const bitsState& other)
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
            newBits.bits[i] = bits[i] | other[i];
        return newBits;
        }
    };



// Represents a particle in the simulation.
struct Particle
    {
    short position; // bits 0-7: x-position, bits 8-15: y-position
    long velocity;  // bits 0-15: x-velocity, bits 16-31: y-position

    Particle() : position(0), velocity(0) {}

    Particle(char pos1, char pos2, short vel1, short vel2)
        {
        position = static_cast<unsigned char>(pos1) | (static_cast<unsigned short>(static_cast<unsigned char>(pos2)) << 8);
        velocity = static_cast<unsigned short>(vel1) | (static_cast<unsigned long>(static_cast<unsigned short>(vel2)) << 16);
        }

    inline void setPosX(char x)
        {
        position = (position & 0xFF00) | static_cast<unsigned char>(x);
        }
    
    inline void setPosY(char y)
        {
        position = (position & 0x00FF) | (static_cast<unsigned short>(static_cast<unsigned char>(y)) << 8);
        }

    inline void setPos(char pos1, char pos2)
        {
        position = static_cast<unsigned char>(pos1) | (static_cast<unsigned short>(static_cast<unsigned char>(pos2)) << 8);
        }

    inline char getPosX() const
        {
        return static_cast<char>(position & 0xFF);
        }
    
    inline char getPosY() const
        {
        return static_cast<char>((position >> 8) & 0xFF);
        }   
    
    inline void setVelX(short vx)
        {
        velocity = (velocity & 0xFFFF0000) | static_cast<unsigned short>(vx);
        }
    
    inline void setVelY(short vy)
        {
        velocity = (velocity & 0x0000FFFF) | (static_cast<unsigned long>(static_cast<unsigned short>(vy)) << 16);
        }
    
    inline void setVel(short vel1, short vel2)
        {
        velocity = static_cast<unsigned short>(vel1) | (static_cast<unsigned long>(static_cast<unsigned short>(vel2)) << 16);
        }

    inline short getVelX() const
        {
        return static_cast<short>(velocity & 0xFFFF);
        }
    
    inline short getVelY() const
        {
        return static_cast<short>((velocity >> 16) & 0xFFFF);
        }     
    };



// Represents a single cell in the simulation grid.
struct GridCell {
    long velocity; // bits 0-15: x-velocity, bits 16-31: y-position
    short weight;  // Weight for velocity normalization

    GridCell() : velocity(0), weight(0) {}

    // Pack velocity with the X and Y components
    inline void velPack(short x, short y) 
        {
        this->velocity = (static_cast<long>(static_cast<unsigned short>(y)) << 16) 
                       | (static_cast<unsigned short>(x));
        }

    inline short getVelX() const 
        {
        return static_cast<short>(velocity);
        }

    inline short getVelY() const 
        {
        return static_cast<short>((velocity >> 16) & 0xFFFF);
        }

    inline void setVelX(short x)
        {
        velocity = (velocity & 0xFFFF0000) | static_cast<unsigned short>(x);
        }
    
    inline void setVelY(short y)
        {
        velocity = (velocity & 0x0000FFFF) | (static_cast<unsigned long>(static_cast<unsigned short>(y)) << 16);
        }

    inline void setVel(short x, short y)
        {
        velocity = static_cast<unsigned short>(x) | (static_cast<unsigned long>(static_cast<unsigned short>(y)) << 16);
        }

    GridCell& operator=(const GridCell& other) 
        {
        velocity = other.velocity;
        weight = other.weight;
        return *this;
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

    Grid& operator=(const Grid& other)
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
class FLIP
    {
    public:
        FLIP(GravityProvider *gravityProvider, VelocityProvider *velocityProvider);
        void update();

    private:
        long gravityInput;  // bits 0-15: x-component, bits 16-31: y-component
        long velocityInput; // bits 0-15: x-component, bits 16-31: y-component
        GravityProvider *gravityProvider;
        VelocityProvider *velocityProvider;
        Particle particles[PARTICLE_NUM];

        void updateInputs();
        void updateVelocities();
        void updatePositions();
        inline short getX(long packedValue);
        inline short getY(long packedValue);
    };

#endif // FLIP_SIMULATION_H