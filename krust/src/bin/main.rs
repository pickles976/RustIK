#[macro_use]
extern crate approx; // For the macro relative_eq!
extern crate nalgebra as na;
use na::{Vector3, Rotation3, Translation3, Matrix4};
use std::{f32::consts::PI, ops::Mul};
use krust::transform_matrix;

fn main() {

    let THETAS: Vec<f32> = vec![0.0, 0.0, 0.0];

    let identity = Matrix4::new(  
        1.0,0.0,0.0,0.0,
        0.0,1.0,0.0,0.0,
        0.0,0.0,1.0,0.0,
        0.0,0.0,0.0,1.0  
    );

    let expected = Matrix4::new(  
        1.0,0.0,0.0,0.0,
        0.0,1.0,0.0,1.0,
        0.0,0.0,1.0,0.0,
        0.0,0.0,0.0,1.0 
    );

    println!("Testing matrices!");

    let axis  = Vector3::x_axis();
    let angle = PI / 2.0;
    let position  = Vector3::new(0.0, 0.0, 1.0);

    let result = transform_matrix(angle, &axis, &position);

    println!("{}", result);

    // relative_eq!(result, expected);

}
