#include "FLIP.h"



/**
 * @param gravityProvider The provider for the gravity vector.
 * @param velocityProvider The provider for the velocity vector.
 * 
 * @brief Constructs a new FLIP object with base initialized values.
 */
FLIP::FLIP(GravityProvider *gravityProvider, VelocityProvider *velocityProvider) : 
           gravityProvider(gravityProvider), 
           velocityProvider(velocityProvider) 
    {
    gravityInput[0] = 0;
    gravityInput[1] = 0;
    velocityInput[0] = 0;
    velocityInput[1] = 0;
    }



/**
 * @brief Updates the simulation by advancing through one step.
 */
void FLIP::update()
    {
    updateInputs();
    updateVelocities();


    // Do this last
    updatePositions();
    // Partical positions to LED states
    }



/**
 * @brief Updates the gravity and velocity vectors.
 */
inline void FLIP::updateInputs()
    {
    long gravInput = gravityProvider->getGravity();
    long veloInput = velocityProvider->getVelocity();
    gravityInput[0] = (gravInput & 0xFFFF);
    gravityInput[1] = (gravInput >> 16);
    velocityInput[0] = (veloInput & 0xFFFF);
    velocityInput[1] = (veloInput >> 16);
    }



/**
 * @brief Updates the partical velocities
 */
inline void FLIP::updateVelocities()
    {
    Partical *partical = nullptr;
    for (unsigned long i = 0; i < PARTICLE_NUM; i++)
        {
        partical = &particles[i];
        partical->velocity[0] += velocityInput[0] + gravityInput[0];
        partical->velocity[1] += velocityInput[1] + gravityInput[1];
        }
    }



/**
 * @brief Updates the partical positions
 */
inline void FLIP::updatePositions()
    {
    Partical *partical = nullptr;
    for (unsigned long i = 0; i < PARTICLE_NUM; i++)
        {
        partical = &particles[i];
        partical->position[0] += partical->velocity[0];
        partical->position[1] += partical->velocity[1];
        }
    }