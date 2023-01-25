#[macro_use]
extern crate approx; // For the macro relative_eq!
extern crate nalgebra as na;
use std::time::{Duration, Instant};

#[cfg(test)]
mod solver_tests {

    use krust::collision_handler::CollisionHandler;
    use na::{Vector3, Matrix4};
    use std::time::Instant;
    use std::vec;
    use krust::matrices::{IDENTITY};
    use krust::solver_ga::IKSolverGA;

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

        let ik_solver: IKSolverGA = IKSolverGA::new(IDENTITY, &angles, &axes, &radii, &min_angles, &max_angles, collision_handler);

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

        IKSolverGA::new(IDENTITY, &angles, &axes, &radii, &min_angles, &max_angles, collision_handler);

    }

    #[test]
    fn test_solver_run() {

        let angles: Vec<f32> = vec![0.0,0.0,0.0];
        let axes: Vec<Vector3<f32>> = vec![*Vector3::x_axis(), *Vector3::x_axis(), *Vector3::x_axis()];
        let radii: Vec<f32> = vec![2.0,2.0,2.0];

        let min_angles: Vec<f32> = vec![-100.0, -100.0, -100.0];
        let max_angles: Vec<f32> = vec![100.0, 100.0, 100.0];

        let collision_handler: CollisionHandler = CollisionHandler::new(&vec![], &vec![], &vec![]);

        let mut ik_solver: IKSolverGA = IKSolverGA::new(IDENTITY, &angles, &axes, &radii, &min_angles, &max_angles, collision_handler);

        ik_solver.set_target(TARGET);

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

        let mut ik_solver: IKSolverGA = IKSolverGA::new(IDENTITY, &angles, &axes, &radii, &min_angles, &max_angles, collision_handler);

        let start = Instant::now();
        ik_solver.solve(TARGET, 0.000000001);
        let duration = start.elapsed();
        println!("Elapsed time: {:?}", duration);
    }

    #[test]
    fn test_solver_collisions() {

        // Create Arm
        let angles: Vec<f32> = vec![0.7902403290998004, -0.4098993668690266, 1.0666514226729367, -0.0014371393973027246, 1.7565908821583627, 0.6499562818474293, 0.011124862179012937, -1.2653301687691405, -1.7979686318114076, -0.7780116748429015];
        let axes: Vec<Vector3<f32>> = vec![*Vector3::z_axis(), *Vector3::y_axis(), *Vector3::y_axis(), *Vector3::z_axis(), *Vector3::y_axis(), *Vector3::y_axis(), *Vector3::z_axis(), *Vector3::y_axis(), *Vector3::y_axis(), *Vector3::z_axis()];
        let radii: Vec<f32> = vec![1.0, 4.0, 4.0, 4.0, 2.0, 4.0, 4.0, 1.0, 2.0, 2.0];

        let min_angles: Vec<f32> = vec![-100.0, -100.0, -100.0, -100.0, -100.0, -100.0, -100.0, -100.0, -100.0, -100.0];
        let max_angles: Vec<f32> = vec![100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0];

        let arm: Vec<Vector3<f32>> = radii.iter().map(|length| Vector3::new(0.6, 0.6, *length / 2.0)).collect();

        // Create obstacles
        let obs1: Vector3<f32> = Vector3::new(5.0, 0.5, 2.0);
        let obs2: Vector3<f32> = Vector3::new(5.0, 0.5, 2.0);

        let offset1: Vector3<f32> = Vector3::new(0.0, 5.0, 2.0);
        let offset2: Vector3<f32> = Vector3::new(0.0, 5.0, 10.0);

        // collision handler
        let collision_handler: CollisionHandler = CollisionHandler::new(&arm, &vec![obs1, obs2], &vec![offset1, offset2]);

        let mut ik_solver: IKSolverGA = IKSolverGA::new(IDENTITY, &angles, &axes, &radii, &min_angles, &max_angles, collision_handler);

        ik_solver.set_target(TARGET);
        ik_solver.reset_params();
        ik_solver.update();
        println!("Loss: {:?}", ik_solver.loss);

    }

    #[test]
    fn test_solver_collisions_solve() {

        // Create Arm
        let angles: Vec<f32> = vec![0.7902403290998004, -0.4098993668690266, 1.0666514226729367, -0.0014371393973027246, 1.7565908821583627, 0.6499562818474293, 0.011124862179012937, -1.2653301687691405, -1.7979686318114076, -0.7780116748429015];
        let axes: Vec<Vector3<f32>> = vec![*Vector3::z_axis(), *Vector3::y_axis(), *Vector3::y_axis(), *Vector3::z_axis(), *Vector3::y_axis(), *Vector3::y_axis(), *Vector3::z_axis(), *Vector3::y_axis(), *Vector3::y_axis(), *Vector3::z_axis()];
        let radii: Vec<f32> = vec![1.0, 4.0, 4.0, 4.0, 2.0, 4.0, 4.0, 1.0, 2.0, 2.0];

        let min_angles: Vec<f32> = vec![-100.0, -100.0, -100.0, -100.0, -100.0, -100.0, -100.0, -100.0, -100.0, -100.0];
        let max_angles: Vec<f32> = vec![100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0];

        let arm: Vec<Vector3<f32>> = radii.iter().map(|length| Vector3::new(0.6, 0.6, *length / 2.0)).collect();

        // Create obstacles
        let obs1: Vector3<f32> = Vector3::new(5.0, 0.5, 2.0);
        let obs2: Vector3<f32> = Vector3::new(5.0, 0.5, 2.0);

        let offset1: Vector3<f32> = Vector3::new(0.0, 5.0, 2.0);
        let offset2: Vector3<f32> = Vector3::new(0.0, 5.0, 10.0);

        // collision handler
        let collision_handler: CollisionHandler = CollisionHandler::new(&arm, &vec![obs1, obs2], &vec![offset1, offset2]);

        let mut ik_solver: IKSolverGA = IKSolverGA::new(IDENTITY, &angles, &axes, &radii, &min_angles, &max_angles, collision_handler);

        let start = Instant::now();
        ik_solver.solve(TARGET, 0.1);
        let duration = start.elapsed();
        println!("Elapsed time: {:?}", duration);

    }

}