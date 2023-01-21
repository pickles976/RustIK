extern crate nalgebra as na;
use na::{Vector3, Matrix4};
use std::{ops::Mul, vec};

pub const IDENTITY: Matrix4<f64> = Matrix4::new(  
    1.0,0.0,0.0,0.0,
    0.0,1.0,0.0,0.0,
    0.0,0.0,1.0,0.0,
    0.0,0.0,0.0,1.0  
);

/// Create a homogeneous transformation matrix with an angle, axis and position
pub fn transform_matrix(angle: f64, axis: &Vector3<f64>, position: &Vector3<f64>) -> Matrix4<f64> {

    let r_mat  = Matrix4::new_rotation(axis.mul(angle));
    let t_mat = Matrix4::new_translation(&position);

    return t_mat * r_mat;

}

/// Create a Vec of homogeneous transform matrices from minimal parameters
pub fn generate_matrices(origin: Matrix4<f64>, angles: &Vec<f64>, axes: &Vec<Vector3<f64>>, radii: &Vec<f64>) -> Vec<Matrix4<f64>> {

    assert!(angles.len() == axes.len() && angles.len() == radii.len(), 
    "Vector lengths unequal! angles: {}, axes: {}, radii: {}", angles.len(), axes.len(), radii.len());

    let radii:Vec<Vector3<f64>> = radii.iter().map(|radius|(Vector3::new(0.0,0.0,*radius))).collect();

    let mut matrices: Vec<Matrix4<f64>> = vec![origin * IDENTITY];

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
pub fn generate_forward_matrices(matrices: &Vec<Matrix4<f64>>) -> Vec<Matrix4<f64>> {

    let mut forward: Vec<Matrix4<f64>> = vec![matrices[0]];

    for i in 1..matrices.len() {
        forward.push(forward[i - 1] * matrices[i]);
    }

    forward

}

/// generate all the backwards partial matrix products
/// [E, D x E, C x D x E] -> [C x D x E, D x E, E] + [ I ]
pub fn generate_backward_matrices(matrices: &Vec<Matrix4<f64>>) -> Vec<Matrix4<f64>> {

    let mut backward: Vec<Matrix4<f64>> = vec![matrices[matrices.len() - 1]];

    for i in 1..matrices.len() {
        backward.push(matrices[matrices.len() - i - 1] * backward[i - 1]);
    }

    backward.reverse();
    backward.push(IDENTITY);

    backward

}

/// Loss function of distance
pub fn distance_loss(actual: &Matrix4<f64>, expected: &Matrix4<f64>, dist_correction: f64) -> f64 {
    let err_x: f64 = f64::powf((*expected.get(12).unwrap() - *actual.get(12).unwrap()) / dist_correction, 2.0);
    let err_y: f64 = f64::powf((*expected.get(13).unwrap() - *actual.get(13).unwrap()) / dist_correction, 2.0);
    let err_z: f64 = f64::powf((*expected.get(14).unwrap() - *actual.get(14).unwrap()) / dist_correction, 2.0);

    err_x + err_y + err_z
}

/// Loss function of rotation
pub fn rotation_loss(actual: &Matrix4<f64>, expected: &Matrix4<f64>, rot_correction: f64) -> f64 {

    let mut err: f64 = 0.0;
    err += f64::powf((*expected.get(0).unwrap() - *actual.get(0).unwrap()) / rot_correction, 2.0);
    err += f64::powf((*expected.get(1).unwrap() - *actual.get(1).unwrap()) / rot_correction, 2.0);
    err += f64::powf((*expected.get(2).unwrap() - *actual.get(2).unwrap()) / rot_correction, 2.0);

    err += f64::powf((*expected.get(4).unwrap() - *actual.get(4).unwrap()) / rot_correction, 2.0);
    err += f64::powf((*expected.get(5).unwrap() - *actual.get(5).unwrap()) / rot_correction, 2.0);
    err += f64::powf((*expected.get(6).unwrap() - *actual.get(6).unwrap()) / rot_correction, 2.0);

    err += f64::powf((*expected.get(8).unwrap() - *actual.get(8).unwrap()) / rot_correction, 2.0);
    err += f64::powf((*expected.get(9).unwrap() - *actual.get(9).unwrap()) / rot_correction, 2.0);
    err += f64::powf((*expected.get(10).unwrap() - *actual.get(10).unwrap()) / rot_correction, 2.0);

    err /= rot_correction;
    err

}

/// Loss/Err function of homogeneous transform matrix, with normalization parameters
pub fn transform_loss(actual: &Matrix4<f64>, expected: &Matrix4<f64>, dist_correction: f64, rot_correction: f64) -> f64 {

    let mut err: f64 = 0.0;

    // Distance-based error
    err += distance_loss(actual, expected, dist_correction);

    // Rotation-based error
    err += rotation_loss(actual, expected, rot_correction);

    err

}
