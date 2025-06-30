use bytemuck::cast_vec;
use fxhash::FxHashMap;
use linfa_clustering::Dbscan;
use nalgebra::Vector2;
// use linfa_datasets::generate;
use ndarray::{ArrayBase, Dim};
// use ndarray_rand::rand::SeedableRng;
// use rand_xoshiro::Xoshiro256Plus;
use linfa::traits::Transformer;
// use rayon::iter::{IndexedParallelIterator, IntoParallelIterator, ParallelIterator};

pub struct Clusterer<F> {
    pub tolerance: f64,
    pub heatmap_scale: f64,
    pub heatmap_width: usize,
    pub min_points: usize,
    points_buffer: Vec<[f64; 2]>,
    sum_buffer: FxHashMap<usize, (Vector2<f64>, usize)>,
    pub density: F,
}

impl<F: FnMut(f64) -> usize + Sync> Clusterer<F> {
    pub fn new(
        tolerance: f64,
        heatmap_scale: f64,
        heatmap_width: usize,
        min_points: usize,
        density: F,
    ) -> Self {
        Self {
            tolerance,
            heatmap_scale,
            heatmap_width,
            density,
            min_points,
            points_buffer: Vec::new(),
            sum_buffer: FxHashMap::default(),
        }
    }

    pub fn cluster<'a>(&'a mut self, heatmap: &[f64]) -> impl Iterator<Item = Vector2<f64>> + 'a {
        let mut buffer = std::mem::take(&mut self.points_buffer);

        let iter = heatmap.into_iter().enumerate().flat_map(|(i, &value)| {
            let x = (i % self.heatmap_width) as f64 * self.heatmap_scale;
            let y = (i / self.heatmap_width) as f64 * self.heatmap_scale;
            std::iter::repeat_n([x, y], (self.density)(value))
        });
        buffer.clear();
        buffer.extend(iter);
        let observations = ArrayBase::<_, Dim<[usize; 2]>>::from(buffer);

        let clusters = Dbscan::params(self.min_points)
            .tolerance(self.tolerance)
            .transform(&observations)
            .unwrap();

        self.points_buffer = cast_vec(observations.into_raw_vec());

        for (i, id) in clusters
            .indexed_iter()
            .filter_map(|(i, &id)| id.map(|id| (i, id)))
        {
            let point = Vector2::from(self.points_buffer[i]);
            let (sum, count) = self.sum_buffer.entry(id).or_insert((Vector2::zeros(), 0));
            *sum += point;
            *count += 1;
        }

        self.sum_buffer
            .drain()
            .map(|(_, (sum, count))| sum / count as f64)
    }
}
