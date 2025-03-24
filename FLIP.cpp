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
           velocityInput(0) {}



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
    gravityInput = gravInput;
    velocityInput = veloInput;
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
        partical->setVelX(partical->getVelX() + getX(gravityInput));
        partical->setVelY(partical->getVelY() + getY(gravityInput));
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
        partical->setPosX(partical->getPosX() + partical->getVelX());
        partical->setPosY(partical->getPosY() + partical->getVelY());
        }
    }



/**
 * @param packedValue The long storing the X component in the first 16 bits
 * 
 * @brief Unpacks the X component of the packedValue
 * 
 * @returns The X component of the packedValue
 */
inline short FLIP::getX(long packedValue)
    {
    return static_cast<short>(packedValue);
    }



/**
 * @param packedValue The long storing the Y component in the last 16 bits
 * 
 * @brief Unpacks the Y component of the packedValue
 * 
 * @returns The Y component of the packedValue
 */
inline short FLIP::getY(long packedValue)
    {
    return static_cast<short>((packedValue >> 16) & 0xFFFF);
    }