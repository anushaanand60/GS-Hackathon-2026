# Goldman Sachs India Hackathon 2026 (CS Track Solutions)

Welcome to my repository housing my C++20 solutions for the algorithmic and system-design challenges presented during the **Goldman Sachs India Hackathon 2026**. 

---

## Repository Architecture

The repository contains exactly 3 source files, each mapping to a specific hackathon challenge:

```
├── json_type_generator.cpp  # JSON to TypeScript Type Compiler
├── group_trip_planner.cpp   # Constraint-Driven Replanning Engine
└── multi_agent_drone.cpp    # Spatiotemporal Vehicle Routing Engine
```
---

## Challenge Breakdown & Problem Statements

### [1. JSON → TypeScript Type Generator](https://www.hackerrank.com/contests/goldman-sachs-india-hackathon-2026-cs/challenges/json-typescript-type-generator/problem)

**Core Domain:** Abstract Syntax Trees (AST), Deterministic Compilers, Type Inference

#### Full Problem Summary

The task is to build a deterministic compilation tool that ingests raw, single-line compact JSON arrays of variable data structures and outputs character-for-character accurate, strictly formatted TypeScript type declaration files (`.d.ts`). The parser must handle deep structural nesting, missing values, and type collisions without any structural ambiguity.

#### Key Constraints & Engineering Rules

* **Formatting Rigor:** Output requires exact 2-space indentation, strict Unix LF (`\n`) line endings, and case-sensitive ASCII alphabetical sorting for both interface declarations and inner property keys.
* **Type Resolution & Merging:** Primitives must be parsed out dynamically (`string`, `number`, `boolean`, `null`). A field must be tagged as optional (`?`) if it is entirely missing from at least one object in the input stream. Heterogeneous arrays must resolve into explicit union arrays (e.g., `(Address | number)[]`).
* **Name Collisions:** When two different nested paths share an identical object key name, structural conflicts are resolved using a global depth-first, alphabetical key traversal tracking system to append incremental numeric suffixes (e.g., `Address`, `Address2`).

---

### [2. Group Trip Planner](https://www.hackerrank.com/contests/goldman-sachs-india-hackathon-2026-cs/challenges/group-trip-planner/problem)

**Core Domain:** Constraint Satisfaction Problems (CSP), State Management, Real-Time Replanning

#### Full Problem Summary

An optimization and scheduling core engineered to organize a multi-day itinerary for a group of $N$ travelers over a $D$-day timeframe. The core business logic maximizes collective group satisfaction from a global pool of time-bound activities while instantly triggering global downstream recalculations whenever disruptive real-time events (weather blocks, budget cuts, traveler drops, sudden fatigue) are pushed into the stream.

#### Key Constraints & Engineering Rules

* **Fairness Bottlenecks:** Every scheduled daily itinerary must rigidly satisfy three limits. Crucially, these boundaries are dynamically restricted by the most limited *active* member of the group for that day:
* **Financial:** $\sum \text{cost}(a) \le \min(u.\text{budget})$
* **Stamina:** $\sum \text{energy}(a) \le \min(u.\text{energy})$
* **Temporal:** $\sum \text{duration}(a) \le \text{Max Allocated Hours } H$


* **Deterministic Tie-Breaking:** Ambiguous schedules must resolve through a strict tuple priority hierarchy: Maximize Satisfaction Score $\rightarrow$ Minimize Total Financial Cost $\rightarrow$ Choose the subset whose sorted Activity ID array comes first lexicographically.
* **Stateful Fail-safes:** Chosen activities are consumed permanently from the global catalog. If no combination of remaining items fits the strict fairness thresholds, the engine must fall back to a mandatory `REST` day (0 cost, 0 satisfaction points).

---

### [3. Multi-Agent Drone Routing in a Temporal Urban Grid](https://www.hackerrank.com/contests/goldman-sachs-india-hackathon-2026-cs/challenges/dronedelivery/problem)

**Core Domain:** Spatiotemporal Pathfinding, Resource Scheduling, Physics & Payload Models

#### Full Problem Summary

An orchestration routing network designed to navigate a fleet of $N$ autonomous delivery drones transporting $M$ deadline-bound packages from a central warehouse outward across a 2D city coordinate system. The objective function penalizes latency and resource consumption while navigating shifting spatiotemporal zones and physical battery depletion.

#### Key Constraints & Engineering Rules

* **Dynamic No-Fly Zones (NFZs):** Shifting rectangular or circular geometry boundaries that activate/deactivate within explicit timeframes $[T_{start}, T_{end}]$. The routing engine must mathematically calculate whether it is optimal to execute geometric detours via waypoints or stand still using a stateful `WAIT` command until an obstacle deactivates.
* **Payload-Dependent Energy Mechanics:** Drones are loaded with a max battery capacity of 500 units. Power depletion per transit leg scales linearly based on the cumulative weight of remaining packages on the frame:

$$
E_{leg} = d \times (1 + W_{payload})
$$

where:

- d = transit distance
- W_payload = cumulative remaining payload weight carried by the drone

Dropping heavier objects off first actively saves power for subsequent downstream legs.
* **Resource Charging Infrastructure:** Dedicated grid coordinates offer a recharge rate of 2 units/timestep. Crucially, stations have limited physical hardware slots; if full, drones must join a temporal waiting queue, demanding rigorous multi-agent scheduling.
* **The Objective Function:**

$$\text{Raw Score} = (\text{Successful Deliveries} \times 100) - (\text{Total Energy} \times 0.1) - (\text{Makespan} \times 0.05)$$

*Note: A single contract violation (deadline breach, negative battery, payload overflow, or stepping into an active NFZ) voids the entire manifest execution and forces a score of 0.*

---

## Build & Compilation

All files are self-contained and ready to compile using any modern compiler supporting standard **C++20** or higher.

```bash
# Compile Problem - JSON Type Generator
g++ -O3 -std=c++20 json_type_generator.cpp -o json_gen -ljsoncpp

# Compile Problem - Group Trip Planner
g++ -O3 -std=c++20 group_trip_planner.cpp -o trip_planner

# Compile Problem - Multi-Agent Drone Router
g++ -O3 -std=c++20 multi_agent_drone.cpp -o drone_router -ljsoncpp
```

*Developed independently as part of the Goldman Sachs India Hackathon 2026.*
