extern crate nalgebra as na;
use na::{Vector3, Matrix4};
use std::{ops::Mul, vec};

pub const IDENTITY: Matrix4<f32> = Matrix4::new(  
    1.0,0.0,0.0,0.0,
    0.0,1.0,0.0,0.0,
    0.0,0.0,1.0,0.0,
    0.0,0.0,0.0,1.0  
);

/// Create a homogeneous transformation matrix with an angle, axis and position
pub fn transform_matrix(angle: f32, axis: &Vector3<f32>, position: &Vector3<f32>) -> Matrix4<f32> {

    let r_mat  = Matrix4::new_rotation(axis.mul(angle));
    let t_mat = Matrix4::new_translation(&position);

    return r_mat * t_mat;

}

/// Create a Vec of homogeneous transform matrices from minimal parameters
pub fn generate_matrices(origin: Matrix4<f32>, angles: &Vec<f32>, axes: &Vec<Vector3<f32>>, radii: &Vec<f32>) -> Vec<Matrix4<f32>> {

    assert!(angles.len() == axes.len() && angles.len() == radii.len(), 
    "Vector lengths unequal! angles: {}, axes: {}, radii: {}", angles.len(), axes.len(), radii.len());

    let radii:Vec<Vector3<f32>> = radii.iter().map(|radius|(Vector3::new(0.0,0.0,*radius))).collect();

    let mut matrices: Vec<Matrix4<f32>> = vec![origin * IDENTITY];

    angles.iter()
    .zip(axes.iter())
    .zip(radii.iter())
    .for_each(
        |((angle, axis), radius)|
        (matrices.push(transform_matrix(*angle,axis,radius))));

    matrices

}

/// generate all the forward partial matrix products
/// [ O, O x A, O x A x B, O x A x B x C]
pub fn generate_forward_matrices(matrices: &Vec<Matrix4<f32>>) -> Vec<Matrix4<f32>> {

    let mut forward: Vec<Matrix4<f32>> = vec![matrices[0]];

    for i in 1..matrices.len() {
        forward.push(matrices[i] * forward[i - 1]);
    }

    forward

}

/// generate all the backwards partial matrix products
/// [E, D x E, C x D x E] -> [C x D x E, D x E, E] + [ I ]
pub fn generate_backward_matrices(matrices: &Vec<Matrix4<f32>>) -> Vec<Matrix4<f32>> {

    let mut backward: Vec<Matrix4<f32>> = vec![matrices[matrices.len() - 1]];

    for i in 1..matrices.len() {
        backward.push(matrices[matrices.len() - i - 1] * backward[i - 1]);
    }

    backward.reverse();
    backward.push(IDENTITY);

    backward

}
