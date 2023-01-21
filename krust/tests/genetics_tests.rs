#[cfg(test)]
mod genetics_tests {

    use std::time::Instant;

    use krust::{genetics::Population};
    use::krust::genetics::{Gene};
    use rand::{rngs::ThreadRng, Rng};

    #[test]
    fn test_gene_crossover() {

        let thetas: Vec<f64> = vec![0.0, 1.0, 2.0];
        let learn_rate: f64 = 0.5;

        let gene_1: Gene = Gene::new(&thetas, learn_rate);
        let gene_2: Gene = Gene::new(&thetas, learn_rate);

        assert_eq!(gene_1.thetas, thetas);
        assert_eq!(gene_1.learn_rate, learn_rate);

        assert_eq!(gene_2.thetas, thetas);
        assert_eq!(gene_2.learn_rate, learn_rate);

        // Test crossover/inheritance
        let mut gene_3: Gene = Gene::from_parents(&gene_1, &gene_2);

        assert_eq!(gene_3.thetas, thetas);
        assert_eq!(gene_3.learn_rate, learn_rate);

        for i in 0..100 {
            gene_3.mutate(); 
        }

        assert_ne!(gene_3.thetas, thetas)

    }

    #[test]
    fn test_population() {

        let thetas: Vec<f64> = vec![5.0, 5.0, 5.0, 5.0];

        // fitness function measures closeness to this thing
        let closure = move |thetas: &Vec<f64>| -> f64 {

            let target: Vec<f64> = vec![0.0; thetas.len()];

            let mut total: f64 = 0.0;

            for i in 0..target.len() {
                total += f64::powf(target[i] - thetas[i], 2.0);
            }

            1.0 / total
        };

        let mut population: Population = Population::new(100, thetas, Box::new(closure));

        let start = Instant::now();
        for i in 0..100 {
            population.new_generation();
            // println!("Closest fit: {:?}", population.alpha.as_ref().unwrap());
            // println!("Minimum error: {:?}", population.min_err);
        }
        let duration = start.elapsed();
        println!("Elapsed time: {:?}", duration);

    }

}