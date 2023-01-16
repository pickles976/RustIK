extern crate nalgebra as na;
use na::Vector3;
use std::{f32::consts::PI};

pub mod matrices;
pub mod solver;
pub mod genetics;

use matrices::IDENTITY;
use crate::solver::Solver;

fn main() {

    println!("Hello!");

    let angles: Vec<f32> = vec![PI / 2.0,PI / 4.0,0.0];
    let axes: Vec<Vector3<f32>> = vec![*Vector3::x_axis(), *Vector3::y_axis(), *Vector3::z_axis()];
    let radii: Vec<f32> = vec![5.0,3.0,1.0];

    let ik_solver = Solver::new(IDENTITY, &angles, &axes, &radii);

    println!("{}", ik_solver);

}