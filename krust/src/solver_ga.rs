extern crate nalgebra as na;
use na::{Vector3, Matrix4, clamp};
use std::{fmt, f32::consts::PI};
use crate::matrices::{generate_matrices, generate_forward_matrices, generate_backward_matrices, transform_matrix, transform_loss, distance_loss, self};
use crate::genetics::{Population};


const ROT_CORRECTION: f32 = PI;
const MAX_STEPS: i32 = 250;

pub struct IKSolverGA {

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

    pub population: Option<Population>,

}

impl IKSolverGA {

    pub fn new(origin: Matrix4<f32>, thetas: &Vec<f32>, axes: &Vec<Vector3<f32>>, radii: &Vec<f32>) -> IKSolverGA {

        // Make sure arm properties have the same length
        assert!(thetas.len() == axes.len() && thetas.len() == radii.len(), 
        "Vector lengths unequal! angles: {}, axes: {}, radii: {}", thetas.len(), axes.len(), radii.len());

        // Generate the matrices to avoid Option<> for matrix types
        let matrices: Vec<Matrix4<f32>> = generate_matrices(origin, &thetas, &axes, &radii);

        IKSolverGA {
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

            population: None,
        }

    }

    pub fn initialize(&self) {
        let pop: Population = Population::new(100, &self.thetas, self);
    }

    /// Generate mats and update end-effector position/loss for the given configuration
    fn update_matrices(&mut self) {
        self.mats = generate_matrices(self.origin, &self.thetas, &self.axes, &self.radii);
        self.forward_mats = generate_forward_matrices(&self.mats);
        self.backward_mats = generate_backward_matrices(&self.mats);

        self.end_effector = self.forward_mats[self.forward_mats.len() - 1];
        self.loss = self.calculate_loss(&self.end_effector);
    }

    fn update_thetas(&mut self) {
        self.population.unwrap().new_generation();
        self.thetas = self.population.unwrap().alpha.as_ref().unwrap().to_vec();
    }

    /// Update learning parameters
    fn update_params(&mut self) {
        self.iterations += 1;
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
        distance_loss(end_effector, &self.target.unwrap(), self.arm_length)
    }

    // Reset parameters between runs
    pub fn reset_params(&mut self) {
        self.iterations = 0;
        self.loss = 100.0;
    }

    // Generates the fitness function with custom injected values
    pub fn fitness(&self, thetas: &Vec<f32>) -> f32 {

            // 
            let mats: Vec<Matrix4<f32>> = generate_matrices(self.origin, thetas, &self.axes, &self.radii);
            let forward_mats: Vec<Matrix4<f32>> = generate_forward_matrices(&mats);
            let end_effector: Matrix4<f32> = forward_mats[forward_mats.len() - 1];

            let mut total: f32 = 0.0;

            total += distance_loss(&end_effector, self.target, self.arm_length);
    
            1.0 / total

    }

}

impl fmt::Display for IKSolverGA {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f,"Origin: {}, Axes: {:?}, Radii: {:?}, Thetas: {:?}", self.origin, self.axes, self.radii, self.thetas)
    }
}