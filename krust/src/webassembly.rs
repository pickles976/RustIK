use wasm_bindgen::prelude::*;
use serde::{Serialize, Deserialize};

extern crate nalgebra as na;
use na::{Vector3, Matrix4};
use crate::{solver_gd::IKSolverGD, collision_handler::CollisionHandler, solver_ga::IKSolverGA};

#[derive(Serialize, Deserialize, Debug)]
struct Fields {
    origin: Matrix4<f32>,
    thetas: Vec<f32>,
    axes: Vec<Vector3<f32>>,
    radii: Vec<f32>,

    min_angles: Vec<f32>,
    max_angles: Vec<f32>,

    arm_half_extents: Vec<Vector3<f32>>,
    arm_offsets: Vec<Vector3<f32>>,
    world_half_extents: Vec<Vector3<f32>>,
    world_offsets: Vec<Vector3<f32>>
}

#[wasm_bindgen]
extern {
    pub fn alert(s: &str);
}

#[wasm_bindgen]
pub struct InverseKinematics {
    ik_solver_gd: IKSolverGD,
    ik_solver_ga: IKSolverGA,
}

#[wasm_bindgen]
impl InverseKinematics {

    pub fn new(field_str: &str) -> InverseKinematics {

        // TODO: something here to let the user know what failed if we didn't get good values in
        let fields: Fields = serde_json::from_str(field_str).unwrap();

        // alert(&format!("{:?}", fields));

        let collision_handler: CollisionHandler = CollisionHandler::new(&fields.arm_half_extents, &fields.world_half_extents, &fields. world_offsets);

        InverseKinematics {
            ik_solver_gd: IKSolverGD::new(fields.origin, &fields.thetas, &fields.axes, &fields.radii, &fields.min_angles, &fields.max_angles, collision_handler.clone()), 
            ik_solver_ga: IKSolverGA::new(fields.origin, &fields.thetas, &fields.axes, &fields.radii, &fields.min_angles, &fields.max_angles, collision_handler.clone()), 
        }
    }

    pub fn solve(&mut self, target_str: &str, thresh: f32) -> String {

        let target: Matrix4<f32> = serde_json::from_str(target_str).unwrap();

        // self.ik_solver_gd.solve(target, thresh);
    	// serde_json::to_string(&self.ik_solver_gd.thetas).unwrap()

        self.ik_solver_ga.solve(target, thresh);
        serde_json::to_string(&self.ik_solver_ga.thetas).unwrap()
    }
}