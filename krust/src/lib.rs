use wasm_bindgen::prelude::*;
use js_sys::Array;

extern crate nalgebra as na;
use na::{Vector3, Matrix4};

pub mod matrices;
pub mod solver_gd;
pub mod solver_ga;
pub mod genetics;

use solver_gd::IKSolverGD;

#[wasm_bindgen]
pub fn solve_gd(target_array: Array, origin_array: Array, angles_array: Array, axes_array: Array, radii_array: Array) -> js_sys::Float32Array {

    let _target: Matrix4<f32> = js_array_to_matrix(target_array);
    let _origin: Matrix4<f32> = js_array_to_matrix(origin_array);
    let angles_vec: Vec<f32> = js_array_to_vec_f32(angles_array);
    let radii_vec: Vec<f32> = js_array_to_vec_f32(radii_array);
    let axes_vec: Vec<Vector3<f32>> = js_array_to_vector3(axes_array);

    let mut ik_solver: IKSolverGD = IKSolverGD::new(_origin, &angles_vec, &axes_vec, &radii_vec);
    ik_solver.solve(_target, 0.000000001);

    return js_sys::Float32Array::from(&ik_solver.thetas[..])
}

fn js_array_to_vector3(array: Array) -> Vec<Vector3<f32>> {

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