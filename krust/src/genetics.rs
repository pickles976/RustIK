use std::{f32::consts::PI, fmt};
use rand::{Rng, rngs::ThreadRng};
use rayon::prelude::*;

const MUTATION_SIZE: f32 = 0.25; // how large a mutation can be
const MUTATION_RATE: f32 = 0.2;

pub struct Gene {
    
    pub learn_rate: f32,
    pub thetas: Vec<f32>,
    pub min_angles: Vec<f32>,
    pub max_angles: Vec<f32>,

}

impl Gene {

    pub fn new(thetas: &Vec<f32>, min_angles: &Vec<f32>, max_angles: &Vec<f32>, learn_rate: f32) -> Gene {
        Gene {
            learn_rate: learn_rate,
            thetas: thetas.to_vec(),
            min_angles: min_angles.to_vec(),
            max_angles: max_angles.to_vec(),
        }
    }

    pub fn from_parents(parent1: &Gene, parent2: &Gene) -> Gene {

        assert!(parent1.thetas.len() == parent2.thetas.len(), "Parent gene lengths unequal! {} {}", parent1.thetas.len(), parent2.thetas.len());

        let mut rng: ThreadRng = rand::thread_rng();

        // crossover
        let repr: Vec<f32> = parent1.thetas.iter().zip(parent2.thetas.iter()).map(|(theta_1, theta_2)| {
            let new_theta: f32 = if rng.gen::<f32>() > 0.5 {*theta_1} else {*theta_2};
            new_theta
        }).collect();

        Gene {
            learn_rate: parent1.learn_rate,
            min_angles: parent1.min_angles.to_vec(),
            max_angles: parent1.max_angles.to_vec(),
            thetas: repr,
        }

    }

    pub fn mutate(&mut self) {

        let mut rng: ThreadRng = rand::thread_rng();

        for i in 0..self.thetas.len() {
            if rng.gen::<f32>() < MUTATION_RATE {
                self.thetas[i] += MUTATION_SIZE * PI * (rng.gen::<f32>() - 0.5);
                // clamp to constraints
                self.thetas[i] = self.max_angles[i].min(self.min_angles[i].max(self.thetas[i]));
            } 
        }
    }

}

impl fmt::Display for Gene {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f,"Thetas: {:?}", self.thetas)
    }
}

pub struct Population {

    // fitness function,
    fitness: Box<dyn Fn(&Vec<f32>) -> f32 + Send + Sync + 'static>,

    pub size: i32,
    pub generation: i32,
    pub alpha: Option<Vec<f32>>,
    pub min_err: f32,
    members: Vec<Gene>,

}

impl Population {

    pub fn new(size: i32, thetas: Vec<f32>, min_angles: Vec<f32>, max_angles: Vec<f32>, fitness: Box<dyn Fn(&Vec<f32>) -> f32 + Send + Sync + 'static>) -> Population {

        let mut first_gen: Vec<Gene> = Vec::new();

        for i in 0..size {
            first_gen.push(Gene::new(&thetas, &min_angles, &max_angles, 0.5));
        }

        Population {
            fitness: fitness,
            size: size,
            generation: 0,
            alpha: None,
            min_err: f32::MAX,
            members: first_gen,
        }

    }

    ///
    pub fn new_generation(&mut self) {
        self.generation += 1;
        
        // softmax scores
        let scores: Vec<f32> = self.get_scores();
        // let scores: Vec<f32> = self.get_scores_threaded();
        let softmax: Vec<f32> = self.softmax(&scores);

        // get population member with highest fitness
        let index = self.argmax(&softmax);
        self.alpha = Some(self.members[index].thetas.to_vec());
        self.min_err = 1.0 / scores[index];

        let mut new_pop: Vec<Gene> = Vec::new();

        // breed new population
        for i in 0..self.size {
            let p1: &Gene = self.pick_parent(&scores);
            let p2: &Gene = self.pick_parent(&scores);

            let mut temp_gene: Gene = Gene::from_parents(p1, p2);
            temp_gene.mutate();
            new_pop.push(temp_gene);
        }

        self.members = new_pop;

    }

    ///
    fn get_scores(&self) -> Vec<f32>{

        let scores = self.members.iter().map(|gene: &Gene| {
            return (*self.fitness)(&gene.thetas);
        }).collect();

        scores
    }

    fn get_scores_threaded(&self) -> Vec<f32> {

        // threadsafe scores vector
        self.members.par_iter().map(|gene: &Gene| (self.fitness)(&gene.thetas)).collect()

    }

    ///
    fn softmax(&self, scores: &Vec<f32>) -> Vec<f32> {
        let sum: f32 = scores.iter().sum();
        scores.iter().map(|score: &f32| score / sum).collect()
    }

    ///
    fn argmax(&self, scores: &Vec<f32>) -> usize {
        scores
        .iter()
        .enumerate()
        .max_by(|(_, a), (_, b)| a.total_cmp(b))
        .map(|(index, _)| index)
        .unwrap() // unwrap because guaranteed nonexistence of NaN
    }

    ///
    fn pick_parent(&self, scores: &Vec<f32>) -> &Gene {

        let mut rng: ThreadRng = rand::thread_rng();
        let mut thresh: f32 = 0.0;

        for i in 0..self.size {
            thresh += scores[i as usize];
            if thresh > rng.gen::<f32>() {
                return &self.members[i as usize]
            }
        }

        &self.members[0]

    }

}