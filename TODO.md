
## MVP
- [x] Try out the nalgebra library
- [x] Add testing for generating transform matrices
- [x] Create function to generate vec of matrices from vecs of specs
- [x] Test function
- [x] Create forward and backward mat functions
- [x] Test forward and backward mat functions
- [x] Move origin outside function
- [x] test updated functions
- [x] Add solver
- [x] Make solver and matrix methods have a common interface
- [x] Add debug print method to solver
- [x] Organize
- [x] Test solver
- [x] Implement and test loss function
- [x] Test matrix functions against expected values from js
- [x] Implement gradient descent for Solver
- [x] Test gradient descent convergence
- [x] Add momentum and learn rate for gradient descent
- [x] Time functions
- [x] Add solve method
- [x] Fix the circular reference issue with the population/solver references
- [x] Add threading with rayon
- [x] Add GA Solver
- [x] Test GA Solver
- [x] Move Vector clone calls into IKSolverGA constructor
- [x] Determine whether to use f32 or f64
- [x] Add tests for CollisionManager arm collisions
- [x] Add ncollide CollisionManager class
- [x] Pass tests
- [x] Add tests for CollisionManager world collisions
- [x] pass tests
- [x] Add CollisionManager to GA solver


# Use in browser
- [x] Export WASM
- [x] Use in browser
- [x] Allow user to control precision of solution
- [x] Determine if WASM can store an object in the Rust end
- [x] Set up persistent state wasm
- [x] Export GA solver to wasm
- [x] Test GA solver with collision
- [x] Learn to use the Serde library
- [ ] Add serialization of colliders
- [ ] Add Rayon WASM support

https://github.com/GoogleChromeLabs/wasm-bindgen-rayon

# Constraints
- [ ] Add collision constraints to both solvers
- [ ] Add angle constraints to both solvers

# Improvements 
- [ ] Implement collision avoidance jacobian for speedup
