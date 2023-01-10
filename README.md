# RustIK

A Rust port of my IK implementation from [here](https://github.com/pickles976/LearningRobotics).

Long-term goal is to implement a hybrid solver using Genetic Algorithms and gradient-based optimization to solve pre-planned paths for robot arms to follow. 
I'd like to leverage multithreading, GPU compute, and WASM bindings to allow it to be used in the browser with my web visualizer in three.js

TODO:
- [ ] get transformation matrix functions going with nalgebra
- [ ] test transformation matrix cases
- [ ] basic visualization of transforms with three.rs
- [ ] unconstrained gradient descent solver 
- [ ] implement multithreading of gradient descent
- [ ] visualize arm

- [ ] Implement evolutionary algorithm for IK
- [ ] Implement angle constraints
- [ ] Implement obstacle avoidance
- [ ] 
- [ ] Parallelize with threads
- [ ] Hybrid solver
- [ ] GPU parallelization?
- [ ] WASM bindings?
