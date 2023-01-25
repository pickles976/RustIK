extern crate nalgebra as na;
use std::vec;

use na::{Vector3, Matrix4, Isometry3};
use ncollide3d::query;
use ncollide3d::bounding_volume::{self, BoundingVolume, BoundingSphere};
use ncollide3d::shape::Cuboid;
use ncollide3d::math::Vector;
use crate::matrices::transform_matrix;

pub struct CollisionHandler {

    arm_offsets: Vec<Matrix4<f32>>,
    arm_colliders: Vec<Cuboid<f32>>,
    arm_spheres: Vec<BoundingSphere<f32>>,

    world_offsets: Vec<Matrix4<f32>>,
    world_colliders: Vec<Cuboid<f32>>,
    world_spheres: Vec<BoundingSphere<f32>>,

}

impl CollisionHandler{

    // TODO: make this less ugly
    pub fn new(arm: &Vec<Vector3<f32>>, obstacles: &Vec<Vector3<f32>>, obstacle_offsets: &Vec<Vector3<f32>>) -> CollisionHandler {

        let identity = na::one::<Isometry3<f32>>();

        let arm_colliders: Vec<Cuboid<f32>> = arm.iter().map(|v| Cuboid::new(vector_convert(v))).collect();
        let arm_spheres: Vec<BoundingSphere<f32>> = arm_colliders.iter().map(|cube| bounding_volume::bounding_sphere(cube, &identity)).collect();
        
        let world_colliders: Vec<Cuboid<f32>> = obstacles.iter().map(|v| Cuboid::new(vector_convert(v))).collect();
        let world_offsets: Vec<Matrix4<f32>> =  obstacle_offsets.iter().map(|v| transform_matrix(0.0, &Vector3::z_axis(), &v)).collect();

        CollisionHandler {
            arm_offsets: arm.iter().map(|v| transform_matrix(0.0, &Vector3::z_axis(), &Vector3::new(0.0, 0.0, v.z / 2.0))).collect(),
            arm_colliders: arm_colliders,
            arm_spheres: arm_spheres,

            world_spheres: get_bounding_spheres_world(&world_offsets, &world_colliders),
            world_offsets: world_offsets,
            world_colliders: world_colliders,
        }
    }

    pub fn is_arm_colliding_self(&self, index: usize, matrices: &Vec<Matrix4<f32>>) -> bool {

        let isometries: Vec<Isometry3<f32>> = self.get_arm_isometries(matrices);
        let mut spheres: Vec<BoundingSphere<f32>> = vec![];
        self.arm_spheres.iter().enumerate().for_each(|(i, sphere)| spheres.push(sphere.transform_by(&isometries[i])));

        // Checks collisions of the colliders before the index against the
        // colliders after the index. Since colliders in their own slice are guaranteed not to be colliding
        // [] [] [] index [] [] []
        for i in 0..index {
            for j in index..self.arm_colliders.len() {
                if j - i > 1 {
                    if spheres[i].intersects(&spheres[j]) {
                        let iso_i: Isometry3<f32> = isometries[i];
                        let iso_j: Isometry3<f32> = isometries[j];
                        let dist = query::distance(&iso_i, &self.arm_colliders[i], &iso_j, &self.arm_colliders[j]);
                        if dist <= 0.0
                        {
                            return true
                        }
                    }
                }
            }
        }

        false
    }

    pub fn is_arm_colliding_self_naive(&self, matrices: &Vec<Matrix4<f32>>) -> bool {

        let isometries: Vec<Isometry3<f32>> = self.get_arm_isometries(matrices);
        let mut spheres: Vec<BoundingSphere<f32>> = vec![];
        self.arm_spheres.iter().enumerate().for_each(|(i, sphere)| spheres.push(sphere.transform_by(&isometries[i])));

        for i in 0..self.arm_colliders.len() {
            for j in i..self.arm_colliders.len() {
                if j - i > 1 {
                    if spheres[i].intersects(&spheres[j]) {
                        let iso_i: Isometry3<f32> = isometries[i];
                        let iso_j: Isometry3<f32> = isometries[j];
                        let dist = query::distance(&iso_i, &self.arm_colliders[i], &iso_j, &self.arm_colliders[j]);
                        if dist <= 0.0
                        {
                            return true
                        }
                    }
                }
            }
        }

        false
    }

    pub fn find_arm_collisions_self(&self, matrices: &Vec<Matrix4<f32>>) -> Vec<bool> {

        let isometries: Vec<Isometry3<f32>> = self.get_arm_isometries(matrices);

        let mut spheres: Vec<BoundingSphere<f32>> = vec![];
        self.arm_spheres.iter().enumerate().for_each(|(i, sphere)| spheres.push(sphere.transform_by(&isometries[i])));
        
        let mut collisions: Vec<bool>  = vec![false; self.arm_colliders.len()];

        for i in 0..self.arm_colliders.len() {
            for j in i..self.arm_colliders.len() {
                if j - i > 1 {
                    if spheres[i].intersects(&spheres[j]) {
                        let iso_i: Isometry3<f32> = isometries[i];
                        let iso_j: Isometry3<f32> = isometries[j];
                        let dist = query::distance(&iso_i, &self.arm_colliders[i], &iso_j, &self.arm_colliders[j]);
                        if dist <= 0.0
                        {
                            collisions[i] = true;
                            collisions[j] = true;
                        }
                    }
                }
            }
        }

        collisions
    }

    pub fn is_arm_colliding_world(&self, index: usize, matrices: &Vec<Matrix4<f32>>) -> bool {

        let arm_isometries: Vec<Isometry3<f32>> = self.get_arm_isometries(matrices);
        let world_isometries: Vec<Isometry3<f32>> = self.get_world_isometries();

        let mut arm_spheres: Vec<BoundingSphere<f32>> = vec![];
        self.arm_spheres.iter().enumerate().for_each(|(i, sphere)| arm_spheres.push(sphere.transform_by(&arm_isometries[i])));

        for i in index..self.arm_colliders.len() {
            for j in 0..self.world_colliders.len() {
                if arm_spheres[i].intersects(&self.world_spheres[j]) {
                    let iso_i: Isometry3<f32> = arm_isometries[i];
                    let iso_j: Isometry3<f32> = world_isometries[j];
                    let dist = query::distance(&iso_i, &self.arm_colliders[i], &iso_j, &self.world_colliders[j]);
                    if dist <= 0.0
                    {
                        return true
                    }
                }
            }
        }

        false
    }

    pub fn is_arm_colliding_world_naive(&self, matrices: &Vec<Matrix4<f32>>) -> bool {

        let arm_isometries: Vec<Isometry3<f32>> = self.get_arm_isometries(matrices);
        let world_isometries: Vec<Isometry3<f32>> = self.get_world_isometries();

        let mut arm_spheres: Vec<BoundingSphere<f32>> = vec![];
        self.arm_spheres.iter().enumerate().for_each(|(i, sphere)| arm_spheres.push(sphere.transform_by(&arm_isometries[i])));

        for i in 0..self.arm_colliders.len() {
            for j in 0..self.world_colliders.len() {
                if arm_spheres[i].intersects(&self.world_spheres[j]) {
                    let iso_i: Isometry3<f32> = arm_isometries[i];
                    let iso_j: Isometry3<f32> = world_isometries[j];
                    let dist = query::distance(&iso_i, &self.arm_colliders[i], &iso_j, &self.world_colliders[j]);
                    if dist <= 0.0
                    {
                        return true
                    }
                }
            }
        }

        false
    }

    pub fn find_arm_collisions_world(&self, matrices: &Vec<Matrix4<f32>>) -> Vec<bool> {

        let arm_isometries: Vec<Isometry3<f32>> = self.get_arm_isometries(matrices);
        let world_isometries: Vec<Isometry3<f32>> = self.get_world_isometries();

        let mut arm_spheres: Vec<BoundingSphere<f32>> = vec![];
        self.arm_spheres.iter().enumerate().for_each(|(i, sphere)| arm_spheres.push(sphere.transform_by(&arm_isometries[i])));

        
        let mut collisions: Vec<bool>  = vec![false; self.arm_colliders.len()];

        for i in 0..self.arm_colliders.len() {
            for j in 0..self.world_colliders.len() {
                if arm_spheres[i].intersects(&self.world_spheres[j]) {
                    let iso_i: Isometry3<f32> = arm_isometries[i];
                    let iso_j: Isometry3<f32> = world_isometries[j];
                    let dist = query::distance(&iso_i, &self.arm_colliders[i], &iso_j, &self.world_colliders[j]);
                    if dist <= 0.0
                    {
                        collisions[i] = true;
                    }
                }
            }
        }

        collisions
    }

    fn get_arm_isometries(&self, matrices: &Vec<Matrix4<f32>>) -> Vec<Isometry3<f32>> {

        let mut isometries: Vec<Isometry3<f32>> = vec![];

        self.arm_offsets
        .iter()
        .enumerate()
        .for_each(
            |(i, offset)| 
            isometries.push(nalgebra::try_convert(matrices[i] * offset).expect("Matrix was not an isometry"))
        );

        isometries
    }

    fn get_world_isometries(&self) -> Vec<Isometry3<f32>> {

        self.world_offsets
        .iter()
        .map(
            |offset| 
            nalgebra::try_convert(offset.clone()).expect("Matrix was not an isometry")
        ).collect()

    }

}

impl Clone for CollisionHandler {
    fn clone(&self) -> CollisionHandler {
        CollisionHandler 
        { 
            arm_offsets: self.arm_offsets.to_vec(), 
            arm_colliders: self.arm_colliders.to_vec(), 
            arm_spheres: self.arm_spheres.to_vec(), 
            world_offsets: self.world_offsets.to_vec(), 
            world_colliders: self.world_colliders.to_vec(), 
            world_spheres: self.world_spheres.to_vec(), 
        }
    }
} 

fn vector_convert(v: &Vector3<f32>) -> Vector<f32> {
    Vector::new(v.x, v.y, v.z)
}

// Since obstacles are static we bake their Isometries into their bounding spheres
fn get_bounding_spheres_world(world_offsets: &Vec<Matrix4<f32>>, world_colliders: &Vec<Cuboid<f32>>) -> Vec<BoundingSphere<f32>> {

    let world_isometries: Vec<Isometry3<f32>> = world_offsets.iter().map(|offset| nalgebra::try_convert(offset.clone()).expect("Matrix was not an isometry")).collect();
    
    let mut world_spheres: Vec<BoundingSphere<f32>> = vec![];
    world_colliders.iter().enumerate().for_each(|(i, cube)| world_spheres.push(bounding_volume::bounding_sphere(cube, &world_isometries[i])));

    world_spheres
}