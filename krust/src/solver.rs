extern crate nalgebra as na;
use na::{Vector3, Matrix4};
use std::{fmt, f32::consts::PI};
use crate::matrices::{generate_matrices, generate_forward_matrices, generate_backward_matrices};

pub struct Solver {

    pub axes: Vec<Vector3<f32>>,
    pub radii: Vec<f32>,
    pub thetas: Vec<f32>,
    pub origin: Matrix4<f32>, 

    arm_length: f32,
    pub end_effector: Matrix4<f32>,
    target: Option<Matrix4<f32>>,

    pub mats: Vec<Matrix4<f32>>,
    pub forward_mats: Vec<Matrix4<f32>>,
    pub backward_mats: Vec<Matrix4<f32>>,

    pub loss: f32,
    iterations: i32,

}

impl Solver {

    const ROT_CORRECTION: f32 = PI;

    pub fn new(origin: Matrix4<f32>, thetas: &Vec<f32>, axes: &Vec<Vector3<f32>>, radii: &Vec<f32>) -> Solver {

        assert!(thetas.len() == axes.len() && thetas.len() == radii.len(), 
        "Vector lengths unequal! angles: {}, axes: {}, radii: {}", thetas.len(), axes.len(), radii.len());

        let matrices: Vec<Matrix4<f32>> = generate_matrices(origin, &thetas, &axes, &radii);

        Solver {
            origin: origin,
            thetas: thetas.to_vec(),
            axes: axes.to_vec(),
            radii: radii.to_vec(),

            arm_length: radii.iter().sum(),
            end_effector: matrices[matrices.len() - 1],
            target: None,

            forward_mats: generate_forward_matrices(&matrices),
            backward_mats: generate_backward_matrices(&matrices),
            mats: matrices,

            loss: 100.0,
            iterations: 0,
        }
    }

    pub fn generate_mats(&mut self) {
        self.mats = generate_matrices(self.origin, &self.thetas, &self.axes, &self.radii);
        self.forward_mats = generate_forward_matrices(&self.mats);
        self.backward_mats = generate_backward_matrices(&self.mats);
    }

    

}

impl fmt::Display for Solver {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f,"Origin: {}, Axes: {:?}, Radii: {:?}, Thetas: {:?}", self.origin, self.axes, self.radii, self.thetas)
    }
}