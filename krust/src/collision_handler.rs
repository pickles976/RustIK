extern crate nalgebra as na;
use na::{Vector3, Matrix4, Isometry3};
use ncollide3d::bounding_volume::{self, BoundingVolume};
use ncollide3d::shape::Cuboid;
pub struct CollisionHandler {

    arm_colliders: Vec<Cuboid<f64>>,
    world_colliders: Vec<Cuboid<f64>>,

}

impl CollisionHandler{

    pub fn new(arm: Vec<Cuboid<f64>>, world: Vec<Cuboid<f64>>) -> CollisionHandler {
        CollisionHandler {
            arm_colliders: arm,
            world_colliders: world,
        }
    }

    // utility functions here
    pub fn is_arm_colliding(&self, matrices: &Vec<Matrix4<f64>>) -> bool {
        // TODO:
        // Matrix4 to isometry
        // Update transform of colliders
        // test distance
        // perform collision checks
        true
    }
}