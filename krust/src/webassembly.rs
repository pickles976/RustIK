use wasm_bindgen::prelude::*;
use serde::{Serialize, Deserialize};

extern crate nalgebra as na;
use na::{Vector3, Matrix4};
use crate::{solver_gd::IKSolverGD, collision_handler::CollisionHandler, solver_ga::IKSolverGA};

// #[derive(Serialize, Deserialize, Debug)]
// struct Fields {
//     origin: Matrix4<f32>
// }

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

    // origin array, angles array, axes array
    // TODO: turn this into a struct
    pub fn new(origin_str: &str, thetas_str: &str, axes_str: &str, radii_str: &str, min_angles_str: &str, max_angles_str: &str, arm_colliders: &str, arm_offsets: &str, world_colliders: &str, world_offsets: &str) -> InverseKinematics {

        // TODO: something here to let the user know what failed if we didn't get good values in
        let origin: Matrix4<f32> = serde_json::from_str(origin_str).unwrap();
        let thetas: Vec<f32> = serde_json::from_str(thetas_str).unwrap();
        let axes: Vec<Vector3<f32>> = serde_json::from_str(axes_str).unwrap();
        let radii: Vec<f32> = serde_json::from_str(radii_str).unwrap();

        let min_angles: Vec<f32> = serde_json::from_str(min_angles_str).unwrap();
        let max_angles: Vec<f32> = serde_json::from_str(max_angles_str).unwrap();

        let arm_collider_vec: Vec<Vector3<f32>> = serde_json::from_str(arm_colliders).unwrap();
        let arm_offsets_vec: Vec<Vector3<f32>> = serde_json::from_str(arm_offsets).unwrap();
        let world_collider_vec: Vec<Vector3<f32>> = serde_json::from_str(world_colliders).unwrap();
        let world_offsets_vec: Vec<Vector3<f32>> = serde_json::from_str(world_offsets).unwrap();

        // let collision_handler: CollisionHandler = CollisionHandler::new(vec![], vec![], vec![]);
        let collision_handler: CollisionHandler = CollisionHandler::new(arm_collider_vec, world_collider_vec, world_offsets_vec);

        InverseKinematics {
            ik_solver_gd: IKSolverGD::new(origin, &thetas, &axes, &radii), 
            ik_solver_ga: IKSolverGA::new(origin, &thetas, &axes, &radii, collision_handler), 
        }
    }

    pub fn solve(&mut self, target_str: &str, thresh: f32) -> String {

        let target: Matrix4<f32> = serde_json::from_str(target_str).unwrap();

        self.ik_solver_gd.solve(target, thresh);

        serde_json::to_string(&self.ik_solver_gd.thetas).unwrap()
    }
}