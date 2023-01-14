#[macro_use]
extern crate approx; // For the macro relative_eq!
extern crate nalgebra as na;

#[cfg(test)]
mod solver_tests {

    use na::{Vector3, Matrix4};
    use std::{f32::consts::PI};
    use krust::matrices::{IDENTITY};
    use krust::solver::Solver;

    const ORIGIN: Matrix4<f32> = Matrix4::new(  
        1.0,0.0,0.0,0.0,
        0.0,1.0,0.0,4.5,
        0.0,0.0,1.0,0.0,
        0.0,0.0,0.0,1.0  
    );
    

    #[test]
    fn test_new_solver_success() {

        // Small case
        let angles: Vec<f32> = vec![0.0,0.0,0.0];
        let axes: Vec<Vector3<f32>> = vec![*Vector3::x_axis(), *Vector3::y_axis(), *Vector3::z_axis()];
        let radii: Vec<f32> = vec![5.0,3.0,1.0];

        let ik_solver: Solver = Solver::new(ORIGIN, &angles, &axes, &radii);

        assert_eq!(angles, ik_solver.thetas);
        assert_eq!(axes, ik_solver.axes);
        assert_eq!(radii, ik_solver.radii);

    }

    #[test]
    #[should_panic(expected="Vector lengths unequal! angles: 2, axes: 3, radii: 3")]
    fn test_generate_matrices_fail() {

        // Test with unequal input vectors
        let angles: Vec<f32> = vec![0.0,0.0];
        let axes: Vec<Vector3<f32>> = vec![*Vector3::x_axis(), *Vector3::y_axis(), *Vector3::z_axis()];
        let radii: Vec<f32> = vec![5.0,3.0,1.0];

        Solver::new(ORIGIN, &angles, &axes, &radii);

    }
    
}