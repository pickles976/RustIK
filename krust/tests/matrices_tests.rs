#[macro_use]
extern crate approx; // For the macro relative_eq!
extern crate nalgebra as na;

#[cfg(test)]
mod matrices_tests {

    use na::{Vector3, Matrix4};
    use std::{f32::consts::PI};
    use krust::matrices::{transform_matrix, generate_matrices, generate_forward_matrices, generate_backward_matrices};

    const ORIGIN: Matrix4<f32> = Matrix4::new(  
        1.0,0.0,0.0,0.0,
        0.0,1.0,0.0,0.0,
        0.0,0.0,1.0,0.0,
        0.0,0.0,0.0,1.0  
    );
    

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

        let test_mats: Vec<Matrix4<f32>> = generate_matrices(ORIGIN, &angles, &axes, &radii);

        // asset matrix has length 3
        assert_eq!(test_mats.len(), 4);

        // asset first matrix is what we expect
        assert!(relative_eq!(*test_mats.get(1).unwrap(), mat_1));

    }

    #[test]
    #[should_panic(expected="Vector lengths unequal! angles: 2, axes: 3, radii: 3")]
    fn test_generate_matrices_fail() {

        // Test with unequal input vectors
        let angles: Vec<f32> = vec![0.0,0.0];
        let axes: Vec<Vector3<f32>> = vec![*Vector3::x_axis(), *Vector3::y_axis(), *Vector3::z_axis()];
        let radii: Vec<f32> = vec![5.0,3.0,1.0];

        generate_matrices(ORIGIN, &angles, &axes, &radii);

    }

    #[test]
    fn test_forward_backward_matrices() {

        let f_mat_1 : Matrix4<f32> = Matrix4::new(  
            1.0,0.0,0.0,0.0,
            0.0,1.0,0.0,0.0,
            0.0,0.0,1.0,5.0,
            0.0,0.0,0.0,1.0  
        );

        let f_mat_2 : Matrix4<f32> = Matrix4::new(  
            1.0,0.0,0.0,0.0,
            0.0,1.0,0.0,0.0,
            0.0,0.0,1.0,9.0,
            0.0,0.0,0.0,1.0  
        );

        let f_mat_3 : Matrix4<f32> = Matrix4::new(  
            1.0,0.0,0.0,0.0,
            0.0,0.0,-1.0,-9.0,
            0.0,1.0,0.0,0.0,
            0.0,0.0,0.0,1.0  
        );

        let b_mat_0 : Matrix4<f32> = Matrix4::new(  
            1.0,0.0,0.0,0.0,
            0.0,0.0,-1.0,0.0,
            0.0,1.0,0.0,9.0,
            0.0,0.0,0.0,1.0  
        );

        let b_mat_1 : Matrix4<f32> = Matrix4::new(  
            1.0,0.0,0.0,0.0,
            0.0,0.0,-1.0,0.0,
            0.0,1.0,0.0,9.0,
            0.0,0.0,0.0,1.0  
        );

        let b_mat_2 : Matrix4<f32> = Matrix4::new(  
            1.0,0.0,0.0,0.0,
            0.0,0.0,-1.0,0.0,
            0.0,1.0,0.0,4.0,
            0.0,0.0,0.0,1.0  
        );

        let b_mat_3 : Matrix4<f32> = Matrix4::new(  
            1.0,0.0,0.0,0.0,
            0.0,0.0,-1.0,0.0,
            0.0,1.0,0.0,0.0,
            0.0,0.0,0.0,1.0  
        );

        let angles: Vec<f32> = vec![0.0,0.0,PI / 2.0];
        let axes: Vec<Vector3<f32>> = vec![*Vector3::z_axis(), *Vector3::y_axis(), *Vector3::x_axis()];
        let radii: Vec<f32> = vec![5.0,4.0,0.0];

        let matrices: Vec<Matrix4<f32>> = generate_matrices(ORIGIN, &angles, &axes, &radii);

        // Forward mats
        let forward_mats : Vec<Matrix4<f32>> = generate_forward_matrices(&matrices);

        assert_eq!(forward_mats.len(), 4);
        assert!(relative_eq!(ORIGIN, forward_mats[0]));
        assert!(relative_eq!(f_mat_1, forward_mats[1]));
        assert!(relative_eq!(f_mat_2, forward_mats[2]));
        assert!(relative_eq!(f_mat_3, forward_mats[3], max_relative=1.0));

        // Backward mats
        let backward_mats : Vec<Matrix4<f32>> = generate_backward_matrices(&matrices);

        assert_eq!(backward_mats.len(), 5);
        assert!(relative_eq!(b_mat_0, backward_mats[0]));
        assert!(relative_eq!(b_mat_1, backward_mats[1]));
        assert!(relative_eq!(b_mat_2, backward_mats[2]));
        assert!(relative_eq!(b_mat_3, backward_mats[3]));
        assert!(relative_eq!(ORIGIN, backward_mats[4]));

    }


}