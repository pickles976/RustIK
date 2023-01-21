pub struct CollisionHandler {

    arm_colliders: Vec<f32>,
    world_colliders: Vec<f32>,

}

impl CollisionHandler{
    pub fn new(arm: Vec<f32>, world: Vec<f32>) -> CollisionHandler {
        CollisionHandler {
            arm_colliders: arm,
            world_colliders: world,
        }
    }

    // utility functions here
}