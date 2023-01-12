#[macro_use]
extern crate approx; // For the macro relative_eq!
extern crate nalgebra as na;
use na::{Vector3, Rotation3, Translation3, Matrix4};
use std::f64::consts::PI;

fn main() {

    let THETAS: Vec<f32> = vec![0.0, 0.0, 0.0];

    let IDENTITY = Matrix4::new(  
        1.0,0.0,0.0,0.0,
        0.0,1.0,0.0,0.0,
        0.0,0.0,1.0,0.0,
        0.0,0.0,0.0,1.0  
    );

    let EXPECTED = Matrix4::new(  
        1.0,0.0,0.0,0.0,
        0.0,1.0,0.0,1.0,
        0.0,0.0,1.0,0.0,
        0.0,0.0,0.0,1.0 
    );

    println!("Testing matrices!");

    let axis  = Vector3::x_axis();
    let angle = PI / 2.0;
    let axis = axis * angle;
    let r_mat  = Matrix4::new_rotation(axis);

    let pos  = Vector3::new(0.0, 0.0, 1.0);
    let t_mat = Matrix4::new_translation(&pos);

    // let result = tMat * rMat;
    // let result = result.ToH

    // println!("{}", r_mat);

    // relative_eq!(result, EXPECTED);

}
