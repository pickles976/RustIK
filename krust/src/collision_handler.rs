extern crate nalgebra as na;
use std::vec;

use na::{Vector3, Matrix4, Isometry3};
use ncollide3d::query;
use ncollide3d::shape::Cuboid;
use ncollide3d::math::Vector;
use crate::matrices::transform_matrix;

pub struct CollisionHandler {

    arm_offsets: Vec<Matrix4<f32>>,
    arm_colliders: Vec<Cuboid<f32>>,

    world_offsets: Vec<Matrix4<f32>>,
    world_colliders: Vec<Cuboid<f32>>

}

impl CollisionHandler{

    pub fn new(arm: Vec<Vector3<f32>>, obstacles: Vec<Vector3<f32>>, obstacle_offsets: Vec<Matrix4<f32>>) -> CollisionHandler {
        CollisionHandler {
            arm_offsets: arm.iter().map(|v| transform_matrix(0.0, &Vector3::z_axis(), &Vector3::new(0.0, 0.0, v.z / 2.0))).collect(),
            arm_colliders: arm.iter().map(|v| Cuboid::new(vector_convert(v))).collect(),

            world_offsets: obstacle_offsets,
            world_colliders: obstacles.iter().map(|v| Cuboid::new(vector_convert(v))).collect(),
        }
    }

    pub fn is_arm_colliding_self(&self, matrices: &Vec<Matrix4<f32>>) -> bool {

        for i in 0..self.arm_colliders.len() {
            for j in i..self.arm_colliders.len() {
                if j - i > 1 {
                    let iso_i: Isometry3<f32> = nalgebra::try_convert(matrices[i] * self.arm_offsets[i]).expect("Matrix was not an isometry");
                    let iso_j: Isometry3<f32> = nalgebra::try_convert(matrices[j] * self.arm_offsets[j]).expect("Matrix was not an isometry");
                    let dist = query::distance(&iso_i, &self.arm_colliders[i], &iso_j, &self.arm_colliders[j]);
                    if dist <= 0.0
                    {
                        return true
                    }
                }
            }
        }

        false
    }

    pub fn find_arm_collisions_self(&self, matrices: &Vec<Matrix4<f32>>) -> Vec<bool> {
        
        let mut collisions: Vec<bool>  = vec![false; self.arm_colliders.len()];

        for i in 0..self.arm_colliders.len() {
            for j in i..self.arm_colliders.len() {
                if j - i > 1 {
                    let iso_i: Isometry3<f32> = nalgebra::try_convert(matrices[i] * self.arm_offsets[i]).expect("Matrix was not an isometry");
                    let iso_j: Isometry3<f32> = nalgebra::try_convert(matrices[j] * self.arm_offsets[j]).expect("Matrix was not an isometry");
                    let dist = query::distance(&iso_i, &self.arm_colliders[i], &iso_j, &self.arm_colliders[j]);
                    if dist <= 0.0
                    {
                        collisions[i] = true;
                        collisions[j] = true;
                    }
                }
            }
        }

        collisions
    }

    pub fn is_arm_colliding_world(&self, matrices: &Vec<Matrix4<f32>>) -> bool {

        for i in 0..self.arm_colliders.len() {
            for j in 0..self.world_colliders.len() {
                let iso_i: Isometry3<f32> = nalgebra::try_convert(matrices[i] * self.arm_offsets[i]).expect("Matrix was not an isometry");
                let iso_j: Isometry3<f32> = nalgebra::try_convert(self.world_offsets[j]).expect("Matrix was not an isometry");
                let dist = query::distance(&iso_i, &self.arm_colliders[i], &iso_j, &self.world_colliders[j]);
                if dist <= 0.0
                {
                    return true
                }
            }
        }

        false
    }

    pub fn find_arm_collisions_world(&self, matrices: &Vec<Matrix4<f32>>) -> Vec<bool> {
        
        let mut collisions: Vec<bool>  = vec![false; self.arm_colliders.len()];

        for i in 0..self.arm_colliders.len() {
            for j in 0..self.world_colliders.len() {
                let iso_i: Isometry3<f32> = nalgebra::try_convert(matrices[i] * self.arm_offsets[i]).expect("Matrix was not an isometry");
                let iso_j: Isometry3<f32> = nalgebra::try_convert(self.world_offsets[j]).expect("Matrix was not an isometry");
                let dist = query::distance(&iso_i, &self.arm_colliders[i], &iso_j, &self.world_colliders[j]);
                if dist <= 0.0
                {
                    collisions[i] = true;
                }
            }
        }

        collisions
    }
}

fn vector_convert(v: &Vector3<f32>) -> Vector<f32> {
    Vector::new(v.x, v.y, v.z)
}