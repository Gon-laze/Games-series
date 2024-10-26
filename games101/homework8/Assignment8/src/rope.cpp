#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
        if (num_nodes <= 0)     return;

        // step 1: build node(Mass)
        masses.resize(num_nodes);
        Vector2D cnt_position   = start;
        Vector2D gap_Length     = (end - start) / num_nodes;
        for (int i = 0; i < num_nodes; i++)
        {
            masses[i] = new Mass(cnt_position, node_mass, false);
            cnt_position += gap_Length;
        }
        
//        Comment-in this part when you implement the constructor
//        for (auto &i : pinned_nodes) {
//            masses[i]->pinned = true;
//        }
        for (auto &i : pinned_nodes)
           masses[i]->pinned = true;

        // step 2: build line(Spring)
        int num_lines = num_nodes - 1;
        springs.resize(num_lines);
        for (int i = 0; i < num_lines; i++)
            springs[i] = new Spring(masses[i], masses[i+1], k);
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            auto cnt_line           = s->m2->position - s->m1->position;
            auto force_b2a          = s->k * cnt_line.unit() * (cnt_line.norm() - s->rest_length);

            s->m1->forces += force_b2a;
            s->m2->forces -= force_b2a;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces += gravity * m->mass;
                auto acceleration   = m->forces / m->mass;
                
                // m->position += m->velocity * delta_t;                       // explict method
                m->velocity += acceleration * delta_t;
                // m->position += m->velocity * delta_t;                       // semi-implicit method
                // TODO (Part 2): Add global damping
                m->position += m->velocity * delta_t * (1 - 0.00005);       // semi-implicit method & damping
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            auto cnt_line           = s->m2->position - s->m1->position;
            auto force_b2a          = s->k * cnt_line.unit() * (cnt_line.norm() - s->rest_length);

            s->m1->forces += force_b2a;
            s->m2->forces -= force_b2a;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                m->forces += gravity * m->mass;
                auto acceleration   = m->forces / m->mass;
                
                // m->position = temp_position + (temp_position - m->last_position) + acceleration * delta_t * delta_t;
                // m->last_position = temp_position;
                
                // TODO (Part 4): Add global Verlet damping

                m->position = temp_position + (1-0.00005) * (temp_position - m->last_position) + acceleration * delta_t * delta_t;
                m->last_position = temp_position;

                m->forces = Vector2D(0, 0);
            }
        }
    }
}
