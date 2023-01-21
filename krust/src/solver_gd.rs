extern crate nalgebra as na;
use na::{Vector3, Matrix4, clamp};
use std::{fmt, f64::consts::PI};
use crate::matrices::{generate_matrices, generate_forward_matrices, generate_backward_matrices, transform_matrix, transform_loss, distance_loss, self};

const ROT_CORRECTION: f64 = PI;
const MAX_D_LOSS: f64 = 0.5;
const MAX_STEPS: i32 = 250;

pub struct IKSolverGD {

    pub axes: Vec<Vector3<f64>>,
    pub radii: Vec<f64>,
    pub thetas: Vec<f64>,
    pub origin: Matrix4<f64>, 

    pub arm_length: f64,
    pub end_effector: Matrix4<f64>,
    pub target: Option<Matrix4<f64>>,

    pub mats: Vec<Matrix4<f64>>,
    pub forward_mats: Vec<Matrix4<f64>>,
    pub backward_mats: Vec<Matrix4<f64>>,

    pub loss: f64,
    pub iterations: i32,

    learn_rate: f64,
    current_learn_rate: f64,
    decay: f64,
    momentums: Vec<f64>,
    momentum_retain: f64,

}

/// The base Solver class. 
/// Uses gradient descent to solve IK for a given target position
impl IKSolverGD {

    pub fn new(origin: Matrix4<f64>, thetas: &Vec<f64>, axes: &Vec<Vector3<f64>>, radii: &Vec<f64>) -> IKSolverGD {

        // Make sure arm properties have the same length
        assert!(thetas.len() == axes.len() && thetas.len() == radii.len(), 
        "Vector lengths unequal! angles: {}, axes: {}, radii: {}", thetas.len(), axes.len(), radii.len());

        // Generate the matrices to avoid Option<> for matrix types
        let matrices: Vec<Matrix4<f64>> = generate_matrices(origin, &thetas, &axes, &radii);

        IKSolverGD {
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

            learn_rate: 0.7,
            current_learn_rate: 0.7,
            decay: 0.000005,
            momentums: vec![0.0; thetas.len()],
            momentum_retain: 0.25,
        }
    }

    /// Generate mats and update end-effector position/loss for the given configuration
    fn update_matrices(&mut self) {
        self.mats = generate_matrices(self.origin, &self.thetas, &self.axes, &self.radii);
        self.forward_mats = generate_forward_matrices(&self.mats);
        self.backward_mats = generate_backward_matrices(&self.mats);

        self.end_effector = self.forward_mats[self.forward_mats.len() - 1];
        self.loss = self.calculate_loss(&self.end_effector);
    }

    /// Perform a gradient descent step to update arm angles
    fn update_thetas(&mut self) {

        let d: f64 = 0.00001;

        for i in 0..self.thetas.len() {

            let d_theta: f64 = self.thetas[i] + d;
            let radius: f64 = self.radii[i];
            let axis: Vector3<f64> = self.axes[i];
            let d_mat: Matrix4<f64> = transform_matrix(d_theta, &axis, &Vector3::new(0.0,0.0,radius));

            let delta_end_effector: Matrix4<f64> = (self.forward_mats[i] * d_mat) * self.backward_mats[i + 2];

            let d_loss = (self.calculate_loss(&delta_end_effector) - self.loss) / d;

            // clamp d_loss
            let d_loss = clamp(d_loss, -MAX_D_LOSS, MAX_D_LOSS);

            // momentum
            let nudge = (self.momentums[i] * self.momentum_retain) + (d_loss * self.learn_rate);

            self.thetas[i] -= nudge;
            self.momentums[i] = nudge;
        }

    }

    /// Update learning parameters
    fn update_params(&mut self) {
        self.iterations += 1;
        self.current_learn_rate = self.learn_rate * (1.0 / (1.0 + self.decay * self.iterations as f64));
    }

    pub fn update(&mut self) {
        self.update_matrices();
        self.update_thetas();
        self.update_params();
    }

    pub fn solve(&mut self, target: Matrix4<f64>, thresh: f64) {

        self.target = Some(target);
        self.reset_params();

        while self.loss > thresh {
            
            if self.iterations < MAX_STEPS {
                self.update();
            } else {
                println!("Failed to solve in {} steps!", self.iterations);
                return;
            }

        }

        println!("Solution found in {} steps!", self.iterations);

    }

    /// Calculate loss for the descent
    fn calculate_loss(&self, end_effector: &Matrix4<f64>) -> f64 {
        // distance_loss(end_effector, &self.target.unwrap(), self.arm_length)
        transform_loss(end_effector, &self.target.unwrap(), self.arm_length, ROT_CORRECTION)
    }

    // Reset parameters between runs
    pub fn reset_params(&mut self) {
        self.iterations = 0;
        self.loss = 100.0;
        self.current_learn_rate = self.learn_rate;
        self.momentums = vec![0.0; self.thetas.len()];
    }

}

impl fmt::Display for IKSolverGD {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f,"Origin: {}, Axes: {:?}, Radii: {:?}, Thetas: {:?}", self.origin, self.axes, self.radii, self.thetas)
    }
}