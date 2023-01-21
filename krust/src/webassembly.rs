use wasm_bindgen::prelude::*;
use js_sys::Array;

extern crate nalgebra as na;
use na::{Vector3, Matrix4};
use crate::solver_gd::IKSolverGD;

#[wasm_bindgen]
pub struct InverseKinematics {
    ik_solver: IKSolverGD,
}

#[wasm_bindgen]
impl InverseKinematics {

    // origin array, angles array, axes array
    pub fn new(origin_array: Array, angles_array: Array, axes_array: Array, radii_array: Array) -> InverseKinematics {
        let _origin: Matrix4<f64> = js_array_to_matrix(origin_array);
        let angles_vec: Vec<f64> = js_array_to_vec_f64(angles_array);
        let axes_vec: Vec<Vector3<f64>> = js_array_to_vector3(axes_array);
        let radii_vec: Vec<f64> = js_array_to_vec_f64(radii_array);

        InverseKinematics {
            ik_solver: IKSolverGD::new(_origin, &angles_vec, &axes_vec, &radii_vec), 
        }
    }

    pub fn solve(&mut self, target_array: Array, thresh: f64) -> js_sys::Float64Array {
        let _target: Matrix4<f64> = js_array_to_matrix(target_array);
        self.ik_solver.solve(_target, thresh);
        return js_sys::Float64Array::from(&self.ik_solver.thetas[..])
    }
}

fn js_array_to_vector3(array: Array) -> Vec<Vector3<f64>> {

    js_array_to_vec_str(array).iter().map(|ax| -> Vector3<f64> 
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

fn js_array_to_matrix(array: Array) -> Matrix4<f64> {
    Matrix4::from_iterator(js_array_to_vec_f64(array).into_iter())
}

fn js_array_to_vec_str(array: Array) -> Vec<String> {
    let mut new_vec: Vec<String> = vec![];
    for i in 0..array.length() {
        new_vec.push(array.get(i).as_string().unwrap() as String);
    }
    new_vec
}

fn js_array_to_vec_f64(array: Array) -> Vec<f64> {
    let mut new_vec: Vec<f64> = vec![];
    for i in 0..array.length() {
        new_vec.push(array.get(i).as_f64().unwrap());
    }
    new_vec
}