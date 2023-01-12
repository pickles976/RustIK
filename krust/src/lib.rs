pub struct Solver {

    axes: bool, // array of axes
    radii: bool, // Array of radii
    thetas: bool, // array of thetas
    origin: bool, // Origin matrix

} 

impl Solver {
    pub fn new(axes: bool, radii: bool, thetas: bool, origin: bool) -> Solver {
        Solver {
            axes: axes,
            radii: radii, 
            thetas: thetas,
            origin: origin,
        }
    }
}