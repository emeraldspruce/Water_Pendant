#include "FLIP.h"



/**
 * @param gravityProvider The provider for the gravity vector.
 * @param velocityProvider The provider for the velocity vector.
 * 
 * @brief Constructs a new FLIP object with base initialized values.
 */
FLIP::FLIP(GravityProvider *gravityProvider, VelocityProvider *velocityProvider) : 
           gravityProvider(gravityProvider), 
           velocityProvider(velocityProvider),
           gravityInput(0),
           velocityInput(0) 
            {
            grid.clear();
            copyGrid.clear();
            }



/**
 * @brief Updates the simulation by advancing through one step.
 */
void FLIP::update()
    {
    copyGrid = grid;
    }



/**
 * @param gravInput The gravity vector to be used in the simulation.
 * @param veloInput The velocity vector to be used in the simulation.
 * 
 * @brief Updates the gravity and velocity vectors.
 */
inline void FLIP::updateInputs(long gravInput, long veloInput)
    {
    gravityInput = gravInput;
    velocityInput = veloInput;
    }



/**
 * @brief Updates the partical velocities
 */
inline void FLIP::updateVelocities()
    {
    Particle *partical = nullptr;
    for (unsigned long i = 0; i < PARTICLE_NUM; i++)
        {
        partical = &particles[i];
        partical->setVel(partical->getVelX() + Math::getX(gravityInput), 
                         partical->getVelY() + Math::getY(gravityInput));
        }
    }



/**
 * @brief Updates the partical positions
 */
inline void FLIP::updatePositions()
    {
    Particle *partical = nullptr;
    for (unsigned long i = 0; i < PARTICLE_NUM; i++)
        {
        partical = &particles[i];
        partical->setPos(partical->getPosX() + partical->getVelX(), 
                         partical->getPosY() + partical->getVelY());
        }
    }



inline void FLIP::ParticleToGrid() 
    {
    
    }