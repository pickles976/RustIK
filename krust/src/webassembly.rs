use wasm_bindgen::prelude::*;
use js_sys::Array;

extern crate nalgebra as na;
use na::{Vector3, Matrix4};
use crate::{solver_gd::IKSolverGD, solver_ga::IKSolverGA, collision_handler::CollisionHandler};

#[wasm_bindgen]
extern {
    pub fn alert(s: &str);
}

#[wasm_bindgen]
pub struct InverseKinematics {
    ik_solver_jc: IKSolverGD,
    ik_solver_ga: IKSolverGA,
}

#[wasm_bindgen]
impl InverseKinematics {

    // origin array, angles array, axes array
    pub fn new(origin_array: Array, angles_array: Array, axes_array: Array, radii_array: Array, arm_colliders: Array, arm_offsets: Array, world_colliders: Array, world_offsets: Array) -> InverseKinematics {
        let _origin: Matrix4<f32> = js_array_to_matrix(origin_array);
        let angles_vec: Vec<f32> = js_array_to_vec_f32(angles_array);
        let axes_vec: Vec<Vector3<f32>> = js_array_to_axis(axes_array);
        let radii_vec: Vec<f32> = js_array_to_vec_f32(radii_array);

        // alert(&format!("Hello, {:?}!", js_array_to_vector3(arm_colliders)));


        // let collision_handler: CollisionHandler = CollisionHandler::new(js_array_to_vector3(arm_colliders), js_array_to_vector3(world_colliders), js_array_to_vector3(world_offsets));

        let collision_handler: CollisionHandler = CollisionHandler::new(vec![], vec![], vec![]);

        InverseKinematics {
            ik_solver_jc: IKSolverGD::new(_origin, &angles_vec, &axes_vec, &radii_vec), 
            ik_solver_ga: IKSolverGA::new(_origin, &angles_vec, &axes_vec, &radii_vec, collision_handler),
        }
    }

    pub fn solve(&mut self, target_array: Array, thresh: f32) -> js_sys::Float32Array {
        let target: Matrix4<f32> = js_array_to_matrix(target_array);

        self.ik_solver_ga.solve(target, 0.1);

        self.ik_solver_jc.thetas = self.ik_solver_ga.thetas.to_vec();
        self.ik_solver_jc.target = Some(target);
        self.ik_solver_jc.update_matrices();

        self.ik_solver_jc.solve(target, thresh);

        js_sys::Float32Array::from(&self.ik_solver_jc.thetas[..])
    }
}

fn js_array_to_axis(array: Array) -> Vec<Vector3<f32>> {

    js_array_to_vec_str(array).iter().map(|ax| -> Vector3<f32> 
        {
            let axis = match ax.as_str() {
                "x" => Vector3::x_axis(),
                "y" => Vector3::y_axis(),
                "z" => Vector3::z_axis(),
                _ => Vector3::x_axis(),
            };
            *axis
        }).collect()

}

fn js_array_to_matrix(array: Array) -> Matrix4<f32> {
    Matrix4::from_iterator(js_array_to_vec_f32(array).into_iter())
}

fn js_array_to_vec_str(array: Array) -> Vec<String> {
    let mut new_vec: Vec<String> = vec![];
    for i in 0..array.length() {
        new_vec.push(array.get(i).as_string().unwrap() as String);
    }
    new_vec
}

fn js_array_to_vec_f32(array: Array) -> Vec<f32> {
    let mut new_vec: Vec<f32> = vec![];
    for i in 0..array.length() {
        new_vec.push(array.get(i).as_f64().unwrap() as f32);
    }
    new_vec
}