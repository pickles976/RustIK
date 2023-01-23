extern crate nalgebra as na;

#[cfg(test)]
mod solver_tests {

    use krust::collision_handler::CollisionHandler;
    use na::{Vector3, Matrix4};
    use krust::matrices::{IDENTITY, generate_matrices, generate_forward_matrices};
    
    #[test]
    fn test_arm_collisions_true() {

        // Small case
        let angles: Vec<f32> = vec![0.2886566990628971, -0.7049275159440052, 1.00318577714416, -0.25468406207327215, 2.179700577307322, 0.9332915810151338, -0.27952073760436214, -1.4061270559735584, -2.006117831803292, -0.3146089084602238];
        let axes: Vec<Vector3<f32>> = vec![*Vector3::z_axis(), *Vector3::y_axis(), *Vector3::y_axis(), *Vector3::z_axis(), *Vector3::y_axis(), *Vector3::y_axis(), *Vector3::z_axis(), *Vector3::y_axis(), *Vector3::y_axis(), *Vector3::z_axis()];
        let radii: Vec<f32> = vec![1.0, 4.0, 4.0, 4.0, 2.0, 4.0, 4.0, 1.0, 2.0, 2.0];

        let arm: Vec<Vector3<f32>> = radii.iter().map(|length| Vector3::new(0.6, 0.6, *length / 2.0)).collect();

        let collision_handler: CollisionHandler = CollisionHandler::new(arm);

        let mats: Vec<Matrix4<f32>> = generate_matrices(IDENTITY, &angles, &axes, &radii);
        let forward_mats: Vec<Matrix4<f32>> = generate_forward_matrices(&mats);

        assert_eq!(collision_handler.is_arm_colliding(&forward_mats), true);
        assert_eq!(collision_handler.find_arm_collisions(&forward_mats), vec![false, false, true, false, false, false, false, true, true, false]);

    }

    #[test]
    fn test_arm_collisions_false() {

        // Small case
        let angles: Vec<f32> = vec![0.7767914999010721, -0.4100893989899414, 1.0667832789736131, 0.008537214886546855, 1.7568289819525176, 0.6496030444375622, 0.013711598062124932, -1.2653288134922276, -1.7978490766201884, -0.7687129563016489];
        let axes: Vec<Vector3<f32>> = vec![*Vector3::z_axis(), *Vector3::y_axis(), *Vector3::y_axis(), *Vector3::z_axis(), *Vector3::y_axis(), *Vector3::y_axis(), *Vector3::z_axis(), *Vector3::y_axis(), *Vector3::y_axis(), *Vector3::z_axis()];
        let radii: Vec<f32> = vec![1.0, 4.0, 4.0, 4.0, 2.0, 4.0, 4.0, 1.0, 2.0, 2.0];

        let arm: Vec<Vector3<f32>> = radii.iter().map(|length| Vector3::new(0.6, 0.6, *length / 2.0)).collect();

        let collision_handler: CollisionHandler = CollisionHandler::new(arm);

        let mats: Vec<Matrix4<f32>> = generate_matrices(IDENTITY, &angles, &axes, &radii);
        let forward_mats: Vec<Matrix4<f32>> = generate_forward_matrices(&mats);

        assert_eq!(collision_handler.is_arm_colliding(&forward_mats), false);
        assert_eq!(collision_handler.find_arm_collisions(&forward_mats), vec![false, false, false, false, false, false, false, false, false, false]);

    }
}