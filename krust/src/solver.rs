extern crate nalgebra as na;
use na::{Vector3, Matrix4, clamp};
use std::{fmt, f32::consts::PI, char::MAX};
use crate::matrices::{generate_matrices, generate_forward_matrices, generate_backward_matrices, transform_matrix, transform_loss, distance_loss, self};

const ROT_CORRECTION: f32 = PI;
const MAX_D_LOSS: f32 = 0.5;

pub struct Solver {

    pub axes: Vec<Vector3<f32>>,
    pub radii: Vec<f32>,
    pub thetas: Vec<f32>,
    pub origin: Matrix4<f32>, 

    pub arm_length: f32,
    pub end_effector: Matrix4<f32>,
    pub target: Option<Matrix4<f32>>,

    pub mats: Vec<Matrix4<f32>>,
    pub forward_mats: Vec<Matrix4<f32>>,
    pub backward_mats: Vec<Matrix4<f32>>,

    pub loss: f32,
    pub iterations: i32,

}

impl Solver {

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

    /// Generate mats and 
    pub fn update_matrices(&mut self) {
        self.mats = generate_matrices(self.origin, &self.thetas, &self.axes, &self.radii);
        self.forward_mats = generate_forward_matrices(&self.mats);
        self.backward_mats = generate_backward_matrices(&self.mats);

        self.end_effector = self.mats[self.mats.len() - 1];
        self.loss = self.calculate_loss(&self.end_effector);
    }

    pub fn update_thetas(&mut self) {

        let d: f32 = 0.00001;
        // let mut new_thetas: Vec<f32> = self.thetas.clone();

        for i in 0..self.thetas.len() {

            let d_theta: f32 = self.thetas[i] + d;
            let radius: f32 = self.radii[i];
            let axis: Vector3<f32> = self.axes[i];
            let d_mat: Matrix4<f32> = transform_matrix(d_theta, &axis, &Vector3::new(0.0,0.0,radius));

            let delta_end_effector: Matrix4<f32> = self.backward_mats[i + 2] * (d_mat * self.forward_mats[i]);

            if (i == 2){
                // println!("forward {}", self.forward_mats[i]);
                // println!("curr {}", d_mat);
                // println!("backward{}", self.backward_mats[i + 2]);
                // println!("new_ee {}", delta_end_effector);
                println!("Old loss: {}, new Loss: {}", self.loss, self.calculate_loss(&delta_end_effector));
            }

            let d_loss = (self.calculate_loss(&delta_end_effector) - self.loss) / d;

            println!("d_loss: {}", d_loss);

            // clamp d_loss
            // let d_loss = clamp(d_loss, -MAX_D_LOSS, MAX_D_LOSS);

            // TODO: add momentum
            let nudge = d_loss * 0.7;

            self.thetas[i] -= nudge;
            // let new_theta = self.thetas[i] - nudge;
            // new_thetas[i] = new_theta;

        }

        // self.thetas = new_thetas;

    }

    pub fn calculate_loss(&self, end_effector: &Matrix4<f32>) -> f32 {
        distance_loss(end_effector, &self.target.unwrap(), self.arm_length)
    }

    

}

impl fmt::Display for Solver {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f,"Origin: {}, Axes: {:?}, Radii: {:?}, Thetas: {:?}", self.origin, self.axes, self.radii, self.thetas)
    }
}