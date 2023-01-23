extern crate nalgebra as na;

#[cfg(test)]
mod solver_tests {

    use krust::collision_handler::CollisionHandler;
    use na::{Vector3, Matrix4};
    use krust::matrices::{IDENTITY, generate_matrices, generate_forward_matrices};
    use nalgebra::Vector;
    
    #[test]
    fn test_arm_collisions_true() {

        // Small case
        let angles: Vec<f32> = vec![0.2886566990628971, -0.7049275159440052, 1.00318577714416, -0.25468406207327215, 2.179700577307322, 0.9332915810151338, -0.27952073760436214, -1.4061270559735584, -2.006117831803292, -0.3146089084602238];
        let axes: Vec<Vector3<f32>> = vec![*Vector3::z_axis(), *Vector3::y_axis(), *Vector3::y_axis(), *Vector3::z_axis(), *Vector3::y_axis(), *Vector3::y_axis(), *Vector3::z_axis(), *Vector3::y_axis(), *Vector3::y_axis(), *Vector3::z_axis()];
        let radii: Vec<f32> = vec![1.0, 4.0, 4.0, 4.0, 2.0, 4.0, 4.0, 1.0, 2.0, 2.0];

        let arm: Vec<Vector3<f32>> = radii.iter().map(|length| Vector3::new(0.6, 0.6, *length / 2.0)).collect();

        let collision_handler: CollisionHandler = CollisionHandler::new(arm, vec![], vec![]);

        let mats: Vec<Matrix4<f32>> = generate_matrices(IDENTITY, &angles, &axes, &radii);
        let forward_mats: Vec<Matrix4<f32>> = generate_forward_matrices(&mats);

        assert_eq!(collision_handler.is_arm_colliding_self(&forward_mats), true);
        assert_eq!(collision_handler.find_arm_collisions_self(&forward_mats), vec![false, false, true, false, false, false, false, true, true, false]);

    }

    #[test]
    fn test_arm_collisions_false() {

        // Small case
        let angles: Vec<f32> = vec![0.7767914999010721, -0.4100893989899414, 1.0667832789736131, 0.008537214886546855, 1.7568289819525176, 0.6496030444375622, 0.013711598062124932, -1.2653288134922276, -1.7978490766201884, -0.7687129563016489];
        let axes: Vec<Vector3<f32>> = vec![*Vector3::z_axis(), *Vector3::y_axis(), *Vector3::y_axis(), *Vector3::z_axis(), *Vector3::y_axis(), *Vector3::y_axis(), *Vector3::z_axis(), *Vector3::y_axis(), *Vector3::y_axis(), *Vector3::z_axis()];
        let radii: Vec<f32> = vec![1.0, 4.0, 4.0, 4.0, 2.0, 4.0, 4.0, 1.0, 2.0, 2.0];

        let arm: Vec<Vector3<f32>> = radii.iter().map(|length| Vector3::new(0.6, 0.6, *length / 2.0)).collect();

        let collision_handler: CollisionHandler = CollisionHandler::new(arm, vec![], vec![]);

        let mats: Vec<Matrix4<f32>> = generate_matrices(IDENTITY, &angles, &axes, &radii);
        let forward_mats: Vec<Matrix4<f32>> = generate_forward_matrices(&mats);

        assert_eq!(collision_handler.is_arm_colliding_self(&forward_mats), false);
        assert_eq!(collision_handler.find_arm_collisions_self(&forward_mats), vec![false, false, false, false, false, false, false, false, false, false]);

    }

    #[test]
    fn test_world_collisions_true() {

        // Create Arm
        let angles: Vec<f32> = vec![1.6165608842843704, -0.46143427543400467, 0.809012100772468, -0.019469167636494276, 1.7568695682192097, 0.9759448225973109, 0.10845480123758768, -1.3190962704639775, -1.7608831199691244, -1.4900014494665197];
        let axes: Vec<Vector3<f32>> = vec![*Vector3::z_axis(), *Vector3::y_axis(), *Vector3::y_axis(), *Vector3::z_axis(), *Vector3::y_axis(), *Vector3::y_axis(), *Vector3::z_axis(), *Vector3::y_axis(), *Vector3::y_axis(), *Vector3::z_axis()];
        let radii: Vec<f32> = vec![1.0, 4.0, 4.0, 4.0, 2.0, 4.0, 4.0, 1.0, 2.0, 2.0];

        let arm: Vec<Vector3<f32>> = radii.iter().map(|length| Vector3::new(0.6, 0.6, *length / 2.0)).collect();

        // Create obstacles
        let obs1: Vector3<f32> = Vector3::new(5.0, 0.5, 2.0);
        let obs2: Vector3<f32> = Vector3::new(5.0, 0.5, 2.0);

        let offset1: Vector3<f32> = Vector3::new(0.0, 5.0, 2.0);
        let offset2: Vector3<f32> = Vector3::new(0.0, 5.0, 10.0);

        // collision handler
        let collision_handler: CollisionHandler = CollisionHandler::new(arm, vec![obs1, obs2], vec![offset1, offset2]);

        // configure arm position
        let mats: Vec<Matrix4<f32>> = generate_matrices(IDENTITY, &angles, &axes, &radii);
        let forward_mats: Vec<Matrix4<f32>> = generate_forward_matrices(&mats);

        assert_eq!(collision_handler.is_arm_colliding_world(&forward_mats), true);
        assert_eq!(collision_handler.find_arm_collisions_world(&forward_mats), vec![false, false, false, false, false, false, true, true, false, true]);

    }

    #[test]
    fn test_world_collisions_false() {

        // Create Arm
        let angles: Vec<f32> = vec![0.7902403290998004, -0.4098993668690266, 1.0666514226729367, -0.0014371393973027246, 1.7565908821583627, 0.6499562818474293, 0.011124862179012937, -1.2653301687691405, -1.7979686318114076, -0.7780116748429015];
        let axes: Vec<Vector3<f32>> = vec![*Vector3::z_axis(), *Vector3::y_axis(), *Vector3::y_axis(), *Vector3::z_axis(), *Vector3::y_axis(), *Vector3::y_axis(), *Vector3::z_axis(), *Vector3::y_axis(), *Vector3::y_axis(), *Vector3::z_axis()];
        let radii: Vec<f32> = vec![1.0, 4.0, 4.0, 4.0, 2.0, 4.0, 4.0, 1.0, 2.0, 2.0];

        let arm: Vec<Vector3<f32>> = radii.iter().map(|length| Vector3::new(0.6, 0.6, *length / 2.0)).collect();

        // Create obstacles
        let obs1: Vector3<f32> = Vector3::new(5.0, 0.5, 2.0);
        let obs2: Vector3<f32> = Vector3::new(5.0, 0.5, 2.0);

        let offset1: Vector3<f32> = Vector3::new(0.0, 5.0, 2.0);
        let offset2: Vector3<f32> = Vector3::new(0.0, 5.0, 10.0);

        // collision handler
        let collision_handler: CollisionHandler = CollisionHandler::new(arm, vec![obs1, obs2], vec![offset1, offset2]);

        // configure arm position
        let mats: Vec<Matrix4<f32>> = generate_matrices(IDENTITY, &angles, &axes, &radii);
        let forward_mats: Vec<Matrix4<f32>> = generate_forward_matrices(&mats);

        assert_eq!(collision_handler.is_arm_colliding_world(&forward_mats), false);
        assert_eq!(collision_handler.find_arm_collisions_world(&forward_mats), vec![false, false, false, false, false, false, false, false, false, false]);

    }

}