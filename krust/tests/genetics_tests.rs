#[macro_use]
extern crate approx; // For the macro relative_eq!

#[cfg(test)]
mod genetics_tests {

    use::krust::genetics::{Gene};

    #[test]
    fn test_gene_crossover() {

        let thetas: Vec<f32> = vec![0.0, 1.0, 2.0];
        let learn_rate: f32 = 0.5;

        let gene_1: Gene = Gene::new(&thetas, learn_rate);
        let gene_2: Gene = Gene::new(&thetas, learn_rate);

        assert_eq!(gene_1.thetas, thetas);
        assert_eq!(gene_1.learn_rate, learn_rate);

        assert_eq!(gene_2.thetas, thetas);
        assert_eq!(gene_2.learn_rate, learn_rate);

        // Test crossover/inheritance
        let gene_3: Gene = Gene::from_parents(&gene_1, &gene_2);

        assert_eq!(gene_3.thetas, thetas);
        assert_eq!(gene_3.learn_rate, learn_rate);

    }

}