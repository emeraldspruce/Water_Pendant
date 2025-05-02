#include "sim_FLIP.h"



SIM::SIM()
    {
    gravityInput[0] = 0;
    gravityInput[1] = -9.81;
    accelInput[0] = 0;
    accelInput[1] = 0;
    initWalls();
    }



/**
 * @param dt The time step to update the simulation
 * 
 * @brief Updates the time step of the simulation
 */
void SIM::updateStepSize(float dt)
    {
    this->dt = dt;
    }



/**
 * @param gravX The x-component of the gravity vector
 * @param gravY The y-component of the gravity vector
 * @param accelX The x-component of the acceleration vector
 * @param accelY The y-component of the acceleration vector
 * 
 * @brief Performs a single step of the simulation
 * 
 * @return The current state of the simulation as a bit state
 */
bitsState SIM::step(float gravX, float gravY, float accelX, float accelY)
    {
    // 1) Update the inputs
    gravityInput[0] = gravX;
    gravityInput[1] = gravY;
    accelInput[0] = accelX;
    accelInput[1] = accelY;

    // 2) Clear current Grids
    grid.clear();

    // 3) Transfer the particles to the grid
    particlesToGrid();
    normalizeGrid();
    copyGrid = grid; // Copy the grid to the copyGrid

    // 4) Update the grid
    applyExternalForces();
    computeDivergence();            
    solvePressure();
    applyPressureGradient();   
    
    // 5) Transfer the grid velocities to the particles
    transferVelocityToParticles();  // FLIP update using copyGrid
    advectParticles();
    enforceParticleBounds();

    // 6) Transfer the particles to the bit state
    return particlesToBitState();
    }



/**
 * @brief Constructs the walls by initializing the weight values to -1
 */
void SIM::initWalls()
    {
    // center index and radius
    constexpr float R   = DIAMETER / 2.0f;      // = 7.5
    constexpr float R2  = R * R;                // = 56.25
    const int       c   = DIAMETER / 2;         // = 7

    for (int row = 0; row < DIAMETER; ++row)
        for (int col = 0; col < DIAMETER; ++col) 
            {
            float dx = col - c;
            float dy = row - c;
            // outside the circle?
            if ((dx*dx + dy*dy) > R2)
                grid[row][col].weight = -1; // wall
            else
                grid[row][col].weight = 0;  // fluid cell
            }
    }



/**
 * @brief Adds the paricles velocity and mass to the grid by:
 */
void SIM::particlesToGrid()
    {
    for (unsigned short i = 0; i < PARTICLE_NUM; ++i)
        {
        // Get the cell coordinates
        long x = static_cast<long>(particles[i].getPosX());
        long y = static_cast<long>(particles[i].getPosY());

        // Update the grid velocities and weights
        GridCell& cell = grid[y][x];
        cell.setVelX(cell.getVelX() + particles[i].getVelX());
        cell.setVelY(cell.getVelY() + particles[i].getVelY());
        cell.weight++;
        }
    }



/**
 * @brief Normalizes the grid by dividing the velocity of each 
 * cell times the weight of a water cell by the total weight in
 * order to get the arithmatic mean of the velocity
 */
void SIM::normalizeGrid()
    {
    // Normalize the grid velocities
    for (unsigned short i = 0; i < DIAMETER; ++i)
        for (unsigned short j = 0; j < DIAMETER; ++j)
            {
            GridCell& cell = grid[i][j];
            if (cell.weight > 0) // Check if cell contains any particles
                {
                cell.setVelX(cell.getVelX() / cell.weight);
                cell.setVelY(cell.getVelY() / cell.weight);
                }
            }
    }



/**
 * @brief Adds gravity (gyro) and acceleration to the GRID
 */
void SIM::applyExternalForces()
    {
    for (unsigned short i = 0; i < DIAMETER; ++i)
        for (unsigned short j = 0; j < DIAMETER; ++j)
            {
            GridCell& cell = grid[i][j];
            if (cell.weight > 0) // Check if cell contains any particles (AND isn't a wall)
                {
                cell.setVelX(cell.getVelX() + dt * (gravityInput[0] + accelInput[0]));
                cell.setVelY(cell.getVelY() + dt * (gravityInput[1] + accelInput[1]));
                }
            }
    }



/**
 * @brief Compute the divergence of each cell such that div = ∇·u 
 */
void SIM::computeDivergence()
    {
    // Compute the middle of the grid
    for (int y = 1; y < DIAMETER - 1; ++y) 
        {
        GridCell* cellRowTop = grid[y - 1];
        GridCell* cellRowCur = grid[y];
        GridCell* cellRowBot = grid[y + 1];
        for (int x = 1; x < DIAMETER - 1; ++x) 
            {
            GridCell& cell = cellRowCur[x];

            // Skip walls
            if (cell.weight < 0)
                continue;

            // fetch neighbors
            float cellEast  = cellRowCur[x + 1].velocity[0];
            float cellWest  = cellRowCur[x - 1].velocity[0];
            float cellNorth = cellRowTop[x].velocity[1];
            float cellSouth = cellRowBot[x].velocity[1];

            cell.divergence = 0.5f * ( cellEast - cellWest +
                                       cellNorth - cellSouth );
            }
        }


    // Compute the edges of the grid
    for (long i = 5; i < 10; ++i)
        {
        float dudx = 0.0f;
        float dvdy = 0.0f;


        // Calculate the divergence for the western edge
        GridCell& cellWest = grid[i][0];
        // forward x-difference
        dudx = grid[i][1].velocity[0] - 
                       cellWest.velocity[0];
        // centered y-difference
        dvdy = 0.5f * ( grid[i + 1][0].velocity[1] -
                        grid[i - 1][0].velocity[1] );
        cellWest.divergence = dudx + dvdy;
    

        // Calculate the divergence for the eastern edge
        GridCell& cellEast = grid[i][DIAMETER - 1];
        // backward x-difference
        dudx = cellEast.velocity[0] -
               grid[i][DIAMETER - 2].velocity[0];
        // centered y-difference
        dvdy = 0.5f * ( grid[i + 1][DIAMETER - 1].velocity[1] -
                        grid[i - 1][DIAMETER - 1].velocity[1] );
        cellEast.divergence = dudx + dvdy;
        
  
        // Calculate the divergence for the northern edge
        GridCell& cellNorth = grid[0][i];
        // centered x-difference
        dudx = 0.5f * ( grid[0][i + 1].velocity[0] -
                        grid[0][i - 1].velocity[0] );
        // forward y-difference
        dvdy = grid[1][i].velocity[1] -
               cellNorth.velocity[1];
        cellNorth.divergence = dudx + dvdy;
  

        // Calculate the divergence for the southern edge
        GridCell& cellSouth = grid[DIAMETER - 1][i];
        // centered x-difference
        dudx = 0.5f * ( grid[DIAMETER - 1][i + 1].velocity[0] -
                        grid[DIAMETER - 1][i - 1].velocity[0] );
        // backward y-difference
        dvdy = cellSouth.velocity[1] -
               grid[DIAMETER - 2][i].velocity[1];
        cellSouth.divergence = dudx + dvdy;
        }
    }



/**
 * @brief Solve the pressure system to find p such that ∇²p = div
 * Using Guass-Seidel method to compute the divergence
 * 
 * @todo: TEST HOW MANY ITERATIONS ARE NEEDED ON AVERAGE.
 *     - if we frequently break early keep the early break
 *     - otherwise, remove the break check
 */
void SIM::solvePressure()
    {
    const int numIterations = DIAMETER;
    const float threshold = 1e-5f;
    
    // For every iteration run through each cell in the grid
    for (int iter = 0; iter < numIterations; ++iter)
        {
        float maxDelta = 0.0f;
        for (int y = 1; y < DIAMETER - 1; ++y)
            for (int x = 1; x < DIAMETER - 1; ++x) 
                {
                GridCell& cell = grid[y][x];
                if (cell.weight < 0) continue;  // wall
        
                // Skip neighbors that are walls (treat them as 0-pressure Dirichlet)
                float pR = grid[y][x+1].weight >= 0 ? grid[y][x+1].pressure : 0.0f;
                float pL = grid[y][x-1].weight >= 0 ? grid[y][x-1].pressure : 0.0f;
                float pU = grid[y+1][x].weight >= 0 ? grid[y+1][x].pressure : 0.0f;
                float pD = grid[y-1][x].weight >= 0 ? grid[y-1][x].pressure : 0.0f;
        
                float newP = 0.25f * (pR + pL + pU + pD - cell.divergence);
                float delta = absf(newP - cell.pressure);

                maxDelta = max2(maxDelta, delta);
                cell.pressure = newP;

                }
        if (maxDelta < threshold) break;
        }
    }



/**
 * @brief Apply pressure gradient so that we avoid compression
 *     - u_new = u_old – ∇p / density
 */
void SIM::applyPressureGradient()
    {
    const float invRho_dt = 1.0f / WATER_DENSITY * dt;

    // Compute the middle of the grid
    for (int y = 1; y < DIAMETER - 1; ++y) 
        {
        GridCell* cellRowTop = grid[y - 1];
        GridCell* cellRowCur = grid[y];
        GridCell* cellRowBot = grid[y + 1];
        for (int x = 1; x < DIAMETER - 1; ++x) 
            {
            GridCell& cell = cellRowCur[x];

            // Skip walls
            if (cell.weight < 0)
                continue;

            // fetch neighbors
            float cellEast  = cellRowCur[x + 1].pressure;
            float cellWest  = cellRowCur[x - 1].pressure;
            float cellNorth = cellRowTop[x].pressure;
            float cellSouth = cellRowBot[x].pressure;

            // centered differences, Δx=Δy=1 so divide by 2
            float gradX = 0.5f * (cellEast - cellWest);
            float gradY = 0.5f * (cellSouth - cellNorth);

            cell.velocity[0] -= invRho_dt * gradX;
            cell.velocity[1] -= invRho_dt * gradY;
            }
        }


    // Compute the edges of the grid
    for (long i = 5; i < 10; ++i)
        {
        float gradX = 0.0f;
        float gradY = 0.0f;


        // Calculate the velocity change for the western edge
        GridCell& cellWest = grid[i][0];
        // forward x-difference
        gradX = grid[i][1].pressure - 
                       cellWest.pressure;
        // centered y-difference
        gradY = 0.5f * ( grid[i + 1][0].pressure -
                        grid[i - 1][0].pressure );
        cellWest.velocity[0] -= invRho_dt * gradX;
        cellWest.velocity[1] -= invRho_dt * gradY;
    

        // Calculate the velocity change for the eastern edge
        GridCell& cellEast = grid[i][DIAMETER - 1];
        // backward x-difference
        gradX = cellEast.pressure -
               grid[i][DIAMETER - 2].pressure;
        // centered y-difference
        gradY = 0.5f * ( grid[i + 1][DIAMETER - 1].pressure -
                        grid[i - 1][DIAMETER - 1].pressure );
        cellEast.velocity[0] -= invRho_dt * gradX;
        cellEast.velocity[1] -= invRho_dt * gradY;
        
  
        // Calculate the velocity change for the northern edge
        GridCell& cellNorth = grid[0][i];
        // centered x-difference
        gradX = 0.5f * ( grid[0][i + 1].pressure -
                        grid[0][i - 1].pressure );
        // forward y-difference
        gradY = grid[1][i].pressure -
               cellNorth.pressure;
        cellNorth.velocity[0] -= invRho_dt * gradX;
        cellNorth.velocity[1] -= invRho_dt * gradY;
  

        // Calculate the velocity change for the southern edge
        GridCell& cellSouth = grid[DIAMETER - 1][i];
        // centered x-difference
        gradX = 0.5f * ( grid[DIAMETER - 1][i + 1].pressure -
                        grid[DIAMETER - 1][i - 1].pressure );
        // backward y-difference
        gradY = cellSouth.pressure -
               grid[DIAMETER - 2][i].pressure;
        cellSouth.velocity[0] -= invRho_dt * gradX;
        cellSouth.velocity[1] -= invRho_dt * gradY;
        }
    }



/**
 * @param grid The grid to interpolate from
 * @param x The x-coordinate of the particle
 * @param y The y-coordinate of the particle
 * 
 * @brief Interpolates the velocity of the grid at the given coordinates
 *     - Uses bilinear interpolation to get the velocity at the given coordinates
 * 
 * @return A vector of interpolated velocity
 */
Vec2<float> SIM::interpolateGridVelocity( Grid& grid, float x, float y ) const
    {
    // Get the cell coordinates
    int i = static_cast<int>(x);
    int j = static_cast<int>(y);

    // Get the fractional part
    float fx = x - i;
    float fy = y - j;

    // Interpolate the velocity
    Vec2<float> velocity;
    GridCell& cell00 = grid[j][i];
    GridCell& cell01 = grid[j][i + 1];
    GridCell& cell10 = grid[j + 1][i];
    GridCell& cell11 = grid[j + 1][i + 1];

    velocity.x =  (1 - fx) * (1 - fy) * cell00.getVelX() +
                  fx *       (1 - fy) * cell01.getVelX() +
                  (1 - fx) * fy *       cell10.getVelX() +
                  fx *       fy *       cell11.getVelX();

    velocity.y =  (1 - fx) * (1 - fy) * cell00.getVelY() +
                  fx *       (1 - fy) * cell01.getVelY() +
                  (1 - fx) * fy *       cell10.getVelY() +
                  fx *       fy *       cell11.getVelY();

    return velocity;
    }



/**
 * @brief Transfer the Grid velocities to the Particles
 */
void SIM::transferVelocityToParticles()
    {
    for (int i = 0; i < PARTICLE_NUM; ++i) 
        {
        Particle& particle = particles[i];

        // Sample new and old grid velocities
        Vec2<float> uNew = interpolateGridVelocity(grid, particle.position[0], particle.position[1]);
        Vec2<float> uOld = interpolateGridVelocity(copyGrid, particle.position[0], particle.position[1]);

        // Apply FLIP delta
        particle.velocity[0] += (uNew.x - uOld.x);
        particle.velocity[1] += (uNew.y - uOld.y);
        }
    }



/**
 * @brief Advect the particles to their new positions
 */
void SIM::advectParticles()
    {
    for (int i = 0; i < PARTICLE_NUM; ++i) 
        {
        Particle& particle = particles[i];
        particle.position[0] += particle.velocity[0] * dt;
        particle.position[1] += particle.velocity[1] * dt;
        }
    }



/**
 * @brief Enforce the bounds of the particles to be within the grid
 */
void SIM::enforceParticleBounds()
    {
    error
    }



/**
 * @brief Convert the particles to a bit state by mapping 
 * the ON/OFF state of each Grid cell to the corresponding LED
 */
bitsState SIM::particlesToBitState()
    {
    error
    }