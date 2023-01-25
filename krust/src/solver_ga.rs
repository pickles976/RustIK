extern crate nalgebra as na;
use na::{Vector3, Matrix4, clamp};
use std::{fmt, f32::consts::PI};
use crate::collision_handler::CollisionHandler;
use crate::matrices::{generate_matrices, generate_forward_matrices, generate_backward_matrices, transform_matrix, transform_loss, distance_loss, self};
use crate::genetics::{Population};


const ROT_CORRECTION: f32 = PI;
const MAX_STEPS: i32 = 5;
const PENALTY: f32 = 1000.0;

pub struct IKSolverGA {

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

    pub population: Option<Box<Population>>,

    pub col_handler: CollisionHandler,

}

impl IKSolverGA {

    pub fn new(origin: Matrix4<f32>, thetas: &Vec<f32>, axes: &Vec<Vector3<f32>>, radii: &Vec<f32>, min_angles: &Vec<f32>, max_angles: &Vec<f32>, col_handler: CollisionHandler) -> IKSolverGA {

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

            population: None,
            col_handler: col_handler,
        }

    }

    pub fn set_target(&mut self, target: Matrix4<f32>) {

        self.target = Some(target);

        // use Box ptr to point to heap data (Population)
        self.population = Some( 
            Box::new(
                Population::new(100, self.thetas.to_vec(),
                    // copy in values to avoid lifetime constraints
                    self.generate_fitness(
                        self.origin, 
                        self.axes.to_vec(), 
                        self.radii.to_vec(), 
                        self.target.unwrap(), 
                        self.arm_length)
                    )
                )
            );

    }

    /// Generate mats and update end-effector position/loss for the given configuration
    pub fn update_matrices(&mut self) {
        self.mats = generate_matrices(self.origin, &self.thetas, &self.axes, &self.radii);
        self.forward_mats = generate_forward_matrices(&self.mats);
        self.backward_mats = generate_backward_matrices(&self.mats);

        self.end_effector = self.forward_mats[self.forward_mats.len() - 1];
        self.loss = self.calculate_loss(&self.end_effector);
    }

    ///
    fn update_thetas(&mut self) {
        self.population.as_mut().unwrap().new_generation();
        self.thetas = self.population.as_ref().unwrap().alpha.as_ref().unwrap().to_vec();
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

        self.set_target(target);
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
        
        if self.col_handler.is_arm_colliding_self(&self.forward_mats) { return 1.0 /PENALTY }
        if self.col_handler.is_arm_colliding_world(&self.forward_mats) { return 1.0 /PENALTY }

        transform_loss(end_effector, &self.target.unwrap(), self.arm_length, ROT_CORRECTION)
    }

    // Reset parameters between runs
    pub fn reset_params(&mut self) {
        self.iterations = 0;
        self.loss = 100.0;
    }

    /// Generate a threadsafe fitness evaluation function for a given member of the population
    pub fn generate_fitness(&self, origin: Matrix4<f32>, axes: Vec<Vector3<f32>>, radii: Vec<f32>, target: Matrix4<f32>, arm_length: f32) -> Box<dyn Fn(&Vec<f32>) -> f32 + Send + Sync + 'static> {

        let _axes: Vec<Vector3<f32>> = axes.to_vec();
        let _origin: Matrix4<f32> = origin.clone();
        let _radii: Vec<f32> = radii.to_vec();
        let _target: Matrix4<f32> = target.clone();
        let _col_handler: CollisionHandler = self.col_handler.clone();

        let closure = move |thetas: &Vec<f32>| -> f32 {

            let mats: Vec<Matrix4<f32>> = generate_matrices(_origin, thetas, &_axes, &_radii);
            let forward_mats: Vec<Matrix4<f32>> = generate_forward_matrices(&mats);
            let end_effector: Matrix4<f32> = forward_mats[forward_mats.len() - 1];

            let mut total: f32 = 0.0;

            if _col_handler.is_arm_colliding_self(&forward_mats) { return 1.0 / PENALTY }
            if _col_handler.is_arm_colliding_world(&forward_mats) { return 1.0 / PENALTY }

            total += transform_loss(&end_effector, &_target, arm_length, ROT_CORRECTION);
    
            1.0 / total
        };

        Box::new(closure)

    }

}

impl fmt::Display for IKSolverGA {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f,"Origin: {}, Axes: {:?}, Radii: {:?}, Thetas: {:?}", self.origin, self.axes, self.radii, self.thetas)
    }
}