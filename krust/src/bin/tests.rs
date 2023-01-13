#[macro_use]
extern crate approx; // For the macro relative_eq!
extern crate nalgebra as na;

#[cfg(test)]
mod tests {

    use na::{Vector3, Matrix4};
    use std::{f32::consts::PI};
    use krust::{transform_matrix, generate_matrices};

    #[test]
    fn test_transform_creation() {

        let rot_x : Matrix4<f32> = Matrix4::new(  
            1.0,0.0,0.0,0.0,
            0.0,0.0,-1.0,-1.0,
            0.0,1.0,0.0,0.0,
            0.0,0.0,0.0,1.0 
        );

        let rot_y : Matrix4<f32> = Matrix4::new(  
            0.0,0.0,1.0,1.0,
            0.0,1.0,0.0,0.0,
            -1.0,0.0,0.0,0.0,
            0.0,0.0,0.0,1.0 
        );

        let rot_z : Matrix4<f32> = Matrix4::new(  
            0.0,-1.0,0.0,0.0,
            1.0,0.0,0.0,0.0,
            0.0,0.0,1.0,1.0,
            0.0,0.0,0.0,1.0 
        );

        let position : Vector3<f32> = Vector3::new(0.0, 0.0, 1.0);
        let angle : f32= PI / 2.0;

        // test x rotation
        let x_test : Matrix4<f32> = transform_matrix(angle, &Vector3::x_axis(), &position);
        assert!(relative_eq!(x_test, rot_x));

        // test y rotation
        let y_test : Matrix4<f32>= transform_matrix(angle, &Vector3::y_axis(), &position);
        assert!(relative_eq!(y_test, rot_y));

        // test z rotation
        let z_test : Matrix4<f32> = transform_matrix(angle, &Vector3::z_axis(), &position);
        assert!(relative_eq!(z_test, rot_z));

    }

    #[test]
    fn test_generate_matrices_success() {

        let mat_1: Matrix4<f32> = Matrix4::new(  
            1.0,0.0,0.0,0.0,
            0.0,1.0,0.0,0.0,
            0.0,0.0,1.0,5.0,
            0.0,0.0,0.0,1.0 
        );

        // Small case
        let angles: Vec<f32> = vec![0.0,0.0,0.0];
        let axes: Vec<Vector3<f32>> = vec![*Vector3::x_axis(), *Vector3::y_axis(), *Vector3::z_axis()];
        let radii: Vec<f32> = vec![5.0,3.0,1.0];

        let test_mats: Vec<Matrix4<f32>> = generate_matrices(angles, axes, radii);

        // asset matrix has length 3
        assert_eq!(test_mats.len(), 3);

        // asset first matrix is what we expect
        assert!(relative_eq!(*test_mats.get(0).unwrap(), mat_1));

    }

    #[test]
    fn test_generate_matrices_fail() {
        
    }

}