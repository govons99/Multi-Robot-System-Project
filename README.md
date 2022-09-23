# Multi Robot System Project

The aim of this project is analysing the hybrid consensus in multi-robot systems where the agents communicate through two different graphs, one related to the continuous dynamics and one related to the discrete dynamics. 

## Simulations

The simulations have been implemented in Matlab and several scenarios have been considered.

1. Agents modelled as simple integrators
    * Connected union communication graph
    * Not connected communication graph
2. Agents modellend as unicycles, _leaderless formation problem_
    * Pure continuous dynamics
    * Hybrid dynamics
    * Sensors with different frequency: orientation sensor working continuosly and position sensor working in a discrete way

## Code

1. _main_integrators_undirected_graphs.m_ regards the simulations when the agents are modelled as simple integrators
2. _leaderless formation problem_:
    * _main_uni_continuous.m_ regards the simulations when the agents evolve acoordingly to a pure continuous dynamics
    * _main_uni_hybrid_communication.m_ regards the simulations when the agents evolve acoordingly to an hybrid dynamics
    * _main_uni_discrete_position.m_ and _main_uni_discrete_position_ez0.m_ regards the simulations when the agents mount two sensors with different frequencies
