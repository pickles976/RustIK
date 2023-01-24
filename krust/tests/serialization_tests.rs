#[macro_use]
extern crate approx; // For the macro relative_eq!
extern crate nalgebra as na;

// TODO: finish these
#[cfg(test)]
mod matrices_tests {

    use na::{Vector3, Matrix4};
    use std::{f32::consts::PI};
    use serde::{Serialize, Deserialize};
    use krust::matrices::IDENTITY;

    const matrix_str: &str = "[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1]";
    

    #[test]
    fn test_serialization() {

        let matrix: Matrix4<f32> = serde_json::from_str(matrix_str).unwrap();
        assert!(relative_eq!(matrix, IDENTITY));
    }

}