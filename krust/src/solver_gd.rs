extern crate nalgebra as na;
use na::{Vector3, Matrix4, clamp};
use std::{fmt, f32::consts::PI};
use crate::{matrices::{generate_matrices, generate_forward_matrices, generate_backward_matrices, transform_matrix, transform_loss, distance_loss, self}, collision_handler::CollisionHandler};
use fxhash::FxHashMap;
use std::collections::HashMap;

const ROT_CORRECTION: f32 = PI;
const MAX_D_LOSS: f32 = 0.5;
const MAX_STEPS: i32 = 10;

#[derive(Eq, PartialEq, Hash, Clone)]
struct Key{
    index: usize,
    angle: i32,
}

pub struct IKSolverGD {

    pub axes: Vec<Vector3<f32>>,
    pub radii: Vec<f32>,
    pub thetas: Vec<f32>,
    pub origin: Matrix4<f32>, 

    pub min_angles: Vec<f32>,
    pub max_angles: Vec<f32>,

    pub arm_length: f32,
    pub end_effector: Matrix4<f32>,
    pub target: Option<Matrix4<f32>>,

    pub mats: Vec<Matrix4<f32>>,
    pub forward_mats: Vec<Matrix4<f32>>,
    pub backward_mats: Vec<Matrix4<f32>>,

    pub loss: f32,
    pub iterations: i32,

    learn_rate: f32,
    current_learn_rate: f32,
    decay: f32,
    momentums: Vec<f32>,
    momentum_retain: f32,

    pub collision_handler: CollisionHandler,
    // mat_cache: FxHashMap<Key, Matrix4<f32>>,

}

/// The base Solver class. 
/// Uses gradient descent/optimization to solve IK for a given target position
impl IKSolverGD {

    pub fn new(origin: Matrix4<f32>, thetas: &Vec<f32>, axes: &Vec<Vector3<f32>>, radii: &Vec<f32>, min_angles: &Vec<f32>, max_angles: &Vec<f32>, col_handler: CollisionHandler) -> IKSolverGD {

        // Make sure arm properties have the same length
        assert!(thetas.len() == axes.len() && thetas.len() == radii.len() && thetas.len() == min_angles.len() && thetas.len() == max_angles.len(), 
        "Vector lengths unequal! angles: {}, axes: {}, radii: {}, min angles: {}, max angles: {}", thetas.len(), axes.len(), radii.len(), min_angles.len(), max_angles.len());

        // Generate the matrices to avoid Option<> for matrix types
        let matrices: Vec<Matrix4<f32>> = generate_matrices(origin, &thetas, &axes, &radii);

        IKSolverGD {
            origin: origin,
            thetas: thetas.to_vec(),
            axes: axes.to_vec(),
            radii: radii.to_vec(),

            min_angles: min_angles.to_vec(),
            max_angles: max_angles.to_vec(),

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
            
            collision_handler: col_handler,
            // mat_cache: FxHashMap::default()
        }
    }

    /// Generate mats and update end-effector position/loss for the given configuration
    pub fn update_matrices(&mut self) {
        self.mats = generate_matrices(self.origin, &self.thetas, &self.axes, &self.radii);
        self.forward_mats = generate_forward_matrices(&self.mats);
        self.backward_mats = generate_backward_matrices(&self.mats);

        self.end_effector = self.forward_mats[self.forward_mats.len() - 1];
        self.loss = self.calculate_loss(&self.end_effector);
    }

    /// Perform a gradient descent step to update arm angles
    fn update_thetas(&mut self) {

        let d: f32 = 0.00001;

        // store matrices for optimization
        let mut mats: Vec<Matrix4<f32>> = self.mats.clone();

        for i in 0..self.thetas.len() {

            let d_theta: f32 = self.thetas[i] + d;
            let radius: f32 = self.radii[i];
            let axis: Vector3<f32> = self.axes[i];
            let d_mat: Matrix4<f32> = transform_matrix(d_theta, &axis, &Vector3::new(0.0,0.0,radius));

            let delta_end_effector: Matrix4<f32> = (self.forward_mats[i] * d_mat) * self.backward_mats[i + 2];

            let d_loss: f32 = (self.calculate_loss(&delta_end_effector) - self.loss) / d;

            // clamp d_loss
            let d_loss: f32 = clamp(d_loss, -MAX_D_LOSS, MAX_D_LOSS);

            // momentum
            let nudge: f32 = (self.momentums[i] * self.momentum_retain) + (d_loss * self.learn_rate);

            let mut new_thetas: Vec<f32> = self.thetas.to_vec();
            new_thetas[i] -= nudge;

            // check angle constraints
            if new_thetas[i] > self.min_angles[i] && new_thetas[i] < self.max_angles[i]
            {

                mats[i + 1] = transform_matrix(new_thetas[i], &axis, &Vector3::new(0.0,0.0,radius));
                let forward_mats: Vec<Matrix4<f32>> = generate_forward_matrices(&mats);
                mats[i + 1] = self.mats[i + 1];
                
                // check collision constraints
                if !self.collision_handler.is_arm_colliding_self(i, &forward_mats) && !self.collision_handler.is_arm_colliding_world(i, &forward_mats) {
                    self.thetas[i] -= nudge;
                    self.momentums[i] = nudge;
                } else {
                    self.thetas[i] += nudge;
                    self.momentums[i] = -nudge;
                }
            }

        }

    }

    /// Update learning parameters
    fn update_params(&mut self) {
        self.iterations += 1;
        self.current_learn_rate = self.learn_rate * (1.0 / (1.0 + self.decay * self.iterations as f32));
    }

    pub fn update(&mut self) {
        self.update_matrices();
        self.update_thetas();
        self.update_params();
    }

    pub fn solve(&mut self, target: Matrix4<f32>, thresh: f32) {

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
    fn calculate_loss(&self, end_effector: &Matrix4<f32>) -> f32 {
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