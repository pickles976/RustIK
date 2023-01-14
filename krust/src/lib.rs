pub mod matrices;

extern crate nalgebra as na;
use matrices::{generate_matrices, generate_forward_matrices, generate_backward_matrices};
use na::{Vector3, Matrix4};

pub struct Solver {

    axes: Vec<Vector3<f32>>,
    radii: Vec<f32>,
    thetas: Vec<f32>,
    origin: Matrix4<f32>, 

    mats: Vec<Matrix4<f32>>,
    forward_mats: Vec<Matrix4<f32>>,
    backward_mats: Vec<Matrix4<f32>>,

}

impl Solver {

    pub fn new(axes: Vec<Vector3<f32>>, radii: Vec<f32>, thetas: Vec<f32>, origin: Matrix4<f32>) -> Solver {

        let matrices: Vec<Matrix4<f32>> = generate_matrices(origin, &thetas, &axes, &radii);

        Solver {
            axes: axes,
            radii: radii,
            thetas: thetas,
            origin: origin,

            forward_mats: generate_forward_matrices(&matrices),
            backward_mats: generate_backward_matrices(&matrices),
            mats: matrices,

        }
    }

    pub fn generate_mats(&mut self) {
        self.mats = generate_matrices(self.origin, &self.thetas, &self.axes, &self.radii);
        self.forward_mats = generate_forward_matrices(&self.mats);
        self.backward_mats = generate_backward_matrices(&self.mats);
    }

}