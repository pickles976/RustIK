extern crate nalgebra as na;
use std::time::Instant;
use na::{Vector3, Matrix4};
use krust::solver_ga::IKSolverGA;

fn main() {
    println!("Hello, world!");

    const TARGET: Matrix4<f32> = Matrix4::new(  
        1.0,0.0,0.0,0.0,
        0.0,1.0,0.0,3.0,
        0.0,0.0,1.0,3.0,
        0.0,0.0,0.0,1.0  
    );

    const IDENTITY: Matrix4<f32> = Matrix4::new(  
        1.0,0.0,0.0,0.0,
        0.0,1.0,0.0,0.0,
        0.0,0.0,1.0,0.0,
        0.0,0.0,0.0,1.0  
    );

    let angles: Vec<f32> = vec![0.0,0.0,0.0];
    let axes: Vec<Vector3<f32>> = vec![*Vector3::x_axis(), *Vector3::x_axis(), *Vector3::x_axis()];
    let radii: Vec<f32> = vec![2.0,2.0,2.0];

    let mut ik_solver: IKSolverGA = IKSolverGA::new(IDENTITY, angles.to_vec(), axes.to_vec(), radii.to_vec());

    let start = Instant::now();
    ik_solver.solve(TARGET, 0.0001);
    let duration = start.elapsed();
    println!("Elapsed time: {:?}", duration);

}
