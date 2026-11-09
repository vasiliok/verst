# Verst: Architectural Bottlenecks & Hard Engineering Challenges

This document outlines the critical technical hurdles, complex edge cases, and potential development dead ends anticipated during the evolution of **verst** from an offline vector map viewer into a fully functional turn-by-turn navigation engine.

It is maintained as a reference for architectural decisions and risk assessment.

---

## 1. Road Network Topology & Turn Restrictions (`graph.dat`)
Building a basic graph for shortest-path calculation is straightforward; building a network suitable for real-world driving requires handling complex topological realities:
* **Turn Restrictions:** OSM turn restrictions (e.g., "no left turn", "only right") are stored as abstract relations. A naive node-edge graph cannot represent these without expanding the state space (e.g., Line Graphs or Expanded Edge Graphs), which significantly increases graph size and complexity.
* **Multi-level Interchanges & Layers:** Roads crossing in 2D space without physical connections (`bridge`, `tunnel`, `layer` tags). Failures in `osm2nav` layer parsing will lead to erratic routing behavior (e.g., suggesting a jump from an overpass to a surface street).
* **Speed Profiles & Hierarchical Weights:** Edge weights must account not only for `maxspeed` and distance, but also road classes (`motorway` vs. `residential`), traffic calming devices, and roundabouts to prevent routing heavy traffic through residential alleys.

## 2. Routing Algorithms & Mobile Performance
Running raw Dijkstra or A* algorithms over regional or national graphs directly on mobile hardware is too slow and memory-intensive:
* **Hierarchical Routing (Contraction Hierarchies / Multi-Level Dijkstra):** To calculate a 500 km route in under 50 ms on an iPhone, the routing graph must be pre-processed and "contracted" (introducing shortcut edges between major highway nodes). Implementing CH generation inside `osm2nav` is algorithmically non-trivial.
* **Dynamic Rerouting:** When a driver misses a turn, recalculating the route from scratch causes noticeable latency. Local repair algorithms or restricted sub-graph searches are required for instant recovery.

## 3. Map Matching & GPS Trace Alignment
Raw GPS data on mobile devices is noisy, prone to multipath errors, and jumps between coordinates:
* **Hidden Markov Models (HMM):** Snapping GPS points to the nearest coordinate on the road network causes vehicles to jump across parallel lanes or off bridges. A robust map matching pipeline using HMMs and Kalman filters is required to reconcile historical trajectory sequences (t-2, t-1, t0), vehicle heading, and graph topology.

## 4. Text & Icon Rendering (SDF Labels & Collision Detection)
Rendering road geometries is lightweight, but labeling a map clearly under continuous motion is a major bottleneck:
* **Dynamic Collision Detection:** Street names and POIs must not overlap, jitter during rotation, or render upside down. This requires a fast 2D spatial index (R-Tree) running every frame to evaluate bounding box visibility.
* **Text Along Curves:** River names and winding roads require text glyphs from FreeType SDF to curve smoothly along complex spline paths.

## 5. Guidance & Maneuver Generation
Converting a sequence of graph edges into human-comprehensible instructions requires deep geometric analysis:
* **Maneuver Aggregation:** Merging 10 consecutive micro-segments into a single instruction ("In 200 meters, turn right onto Rue de la Roche") requires analyzing intersection angles, ramp structures, and street name changes.
* **Roundabouts:** Dedicated state machines are needed to identify circular geometries, count exits correctly ("Take the 3rd exit"), and announce them dynamically.

## 6. Spatial Indexing & Data Management (`names.dat`)
* **Offline Geocoding / Fuzzy Search:** Searching for streets or places locally within binary files without full RAM expansion requires efficient prefix trees (Trie / Finite State Transducers) supporting fast transliteration and typo tolerance.
* **Incremental Updates:** Since OSM data updates frequently, forcing users to re-download entire regional binary maps is inefficient. Designing a patch/delta update mechanism for custom binary formats remains an open challenge.
