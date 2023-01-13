extern crate nalgebra as na;
use na::{Vector3, Matrix4};
use std::{ops::Mul};

/// Create a homogeneous transformation matrix with an angle, axis and position
pub fn transform_matrix(angle: f32, axis: &Vector3<f32>, position: &Vector3<f32>) -> Matrix4<f32> {

    let r_mat  = Matrix4::new_rotation(axis.mul(angle));
    let t_mat = Matrix4::new_translation(&position);

    return r_mat * t_mat;

}

pub fn generate_matrices(angles: Vec<f32>, axes: Vec<Vector3<f32>>, radii: Vec<f32>) -> Vec<Matrix4<f32>> {

    assert!(angles.len() == axes.len() && angles.len() == radii.len(), 
    "Vector lengths unequal! angles: {}, axes: {}, radii: {}", angles.len(), axes.len(), radii.len());

    let radii:Vec<Vector3<f32>> = radii.iter().map(|radius|(Vector3::new(0.0,0.0,*radius))).collect();

    let data: Vec<Matrix4<f32>> = angles.iter()
    .zip(axes.iter())
    .zip(radii.iter())
    .map(
        |((angle, axis), radius)|
        (transform_matrix(*angle,axis,radius))).collect();

    data

}

// pub struct Solver {
//     axes: Vec<Vector3<f32>>,
//     radii: Vec<f32>,
//     thetas: Vec<f32>,
//     mats: Vec<Matrix4<f32>>,
//     // forward_mats: Vec<Matrix4<f32>>,
//     // backward_mats: Vec<Matrix4<f32>>,
//     // end_effector: Matrix4<f32>,
//     // target: Matrix4<f32>,
// }

// impl Solver {

//     pub fn new(axes: Vec<Vector3<f32>>, radii: Vec<f32>,thetas: Vec<f32>) -> Solver {



//         Solver {
//             axes: axes,
//             radii: radii,
//             thetas: thetas
//         }

//     }
// }