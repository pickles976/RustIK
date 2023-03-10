#[macro_use]
extern crate approx; // For the macro relative_eq!
extern crate nalgebra as na;

#[cfg(test)]
mod solver_tests {

    use krust::collision_handler::CollisionHandler;
    use na::{Vector3, Matrix4};
    use std::time::Instant;
    use krust::matrices::{IDENTITY};
    use krust::solver_gd::IKSolverGD;

    const TARGET: Matrix4<f32> = Matrix4::new(  
        1.0,0.0,0.0,0.0,
        0.0,1.0,0.0,3.0,
        0.0,0.0,1.0,3.0,
        0.0,0.0,0.0,1.0  
    );
    
    #[test]
    fn test_new_solver_success() {

        // Small case
        let angles: Vec<f32> = vec![0.0,0.0,0.0];
        let axes: Vec<Vector3<f32>> = vec![*Vector3::x_axis(), *Vector3::y_axis(), *Vector3::z_axis()];
        let radii: Vec<f32> = vec![5.0,3.0,1.0];

        let min_angles: Vec<f32> = vec![-100.0, -100.0, -100.0];
        let max_angles: Vec<f32> = vec![100.0, 100.0, 100.0];

        let collision_handler: CollisionHandler = CollisionHandler::new(&vec![], &vec![], &vec![]);

        let ik_solver: IKSolverGD = IKSolverGD::new(IDENTITY, &angles, &axes, &radii, &min_angles, &max_angles, collision_handler);

        assert_eq!(angles, ik_solver.thetas);
        assert_eq!(axes, ik_solver.axes);
        assert_eq!(radii, ik_solver.radii);
        assert_eq!(9.0, ik_solver.arm_length);

    }

    #[test]
    #[should_panic(expected="Vector lengths unequal! angles: 2, axes: 3, radii: 3")]
    fn test_generate_matrices_fail() {

        // Test with unequal input vectors
        let angles: Vec<f32> = vec![0.0,0.0];
        let axes: Vec<Vector3<f32>> = vec![*Vector3::x_axis(), *Vector3::y_axis(), *Vector3::z_axis()];
        let radii: Vec<f32> = vec![5.0,3.0,1.0];

        let min_angles: Vec<f32> = vec![-100.0, -100.0, -100.0];
        let max_angles: Vec<f32> = vec![100.0, 100.0, 100.0];

        let collision_handler: CollisionHandler = CollisionHandler::new(&vec![], &vec![], &vec![]);

        IKSolverGD::new(IDENTITY, &angles, &axes, &radii, &min_angles, &max_angles, collision_handler);

    }

    #[test]
    fn test_solver_run() {

        let angles: Vec<f32> = vec![0.0,0.0,0.0];
        let axes: Vec<Vector3<f32>> = vec![*Vector3::x_axis(), *Vector3::x_axis(), *Vector3::x_axis()];
        let radii: Vec<f32> = vec![2.0,2.0,2.0];

        let min_angles: Vec<f32> = vec![-100.0, -100.0, -100.0];
        let max_angles: Vec<f32> = vec![100.0, 100.0, 100.0];

        let collision_handler: CollisionHandler = CollisionHandler::new(&vec![], &vec![], &vec![]);

        let mut ik_solver: IKSolverGD = IKSolverGD::new(IDENTITY, &angles, &axes, &radii, &min_angles, &max_angles, collision_handler);

        ik_solver.target = Some(TARGET);

        for i in 0..10 {
            ik_solver.update();
        }

    }
    
    #[test]
    fn test_solver_solve() {

        let angles: Vec<f32> = vec![0.0,0.0,0.0];
        let axes: Vec<Vector3<f32>> = vec![*Vector3::x_axis(), *Vector3::x_axis(), *Vector3::x_axis()];
        let radii: Vec<f32> = vec![2.0,2.0,2.0];

        let min_angles: Vec<f32> = vec![-100.0, -100.0, -100.0];
        let max_angles: Vec<f32> = vec![100.0, 100.0, 100.0];

        let collision_handler: CollisionHandler = CollisionHandler::new(&vec![], &vec![], &vec![]);

        let mut ik_solver: IKSolverGD = IKSolverGD::new(IDENTITY, &angles, &axes, &radii, &min_angles, &max_angles, collision_handler);

        let start = Instant::now();
        ik_solver.solve(TARGET, 0.000000001);
        let duration = start.elapsed();
        println!("Elapsed time: {:?}", duration);
        println!("Thetas: {:?}", ik_solver.thetas);
    }

}