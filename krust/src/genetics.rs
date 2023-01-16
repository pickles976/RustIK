use rand::Rng;

const MUTATION_SIZE: f32 = 0.25; // how large a mutation can be

struct Gene {
    
    parent1: Option<&Gene>,
    parent2: Option<&Gene>,
    learn_rate: f32,
    thetas: Vec<f32>,
    // angle constraints

}

impl Gene {

    pub fn new(thetas: Vec<f32>, learn_rate: f32) -> Gene {
        Gene {
            parent1: None,
            parent2: None,
            learn_rate: learn_rate,
            thetas: thetas,
        }
    }

    pub fn from_parents(parent1: &Gene, parent2: &Gene) -> Gene {

        assert!(parent1.thetas.len() == parent2.thetas.len(), "Parent gene lengths unequal! {} {}", parent1.thetas.len(), parent2.thetas.len());

        let mut rng = rand::thread_rng();

        // crossover
        let repr = parent1.thetas.iter().zip(parent2.iter.zip()).map(|(theta_1, theta_2)| {
            let new_theta = if rng.gen::<f32>() > 0.5 {theta_1} else {theta_2};
        }).collect();

        assert!();

        Gene {
            parent1: Some(parent1),
            parent2: Some(parent2),
            learn_rate: parent1.learn_rate,
            thetas: repr,
        }

    }

}