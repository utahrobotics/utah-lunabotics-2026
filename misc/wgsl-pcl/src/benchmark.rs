use std::time::{Duration, Instant};
use crate::wgsl_setup::GpuDevice;
use anyhow::Result;

#[derive(Debug, Clone)]
pub struct BenchmarkConfig {
    pub warmup_iterations: u32,
    pub timed_iterations: u32,
    pub workgroup_size: (u32, u32, u32),
}

impl Default for BenchmarkConfig {
    fn default() -> Self {
        Self {
            warmup_iterations: 3,
            timed_iterations: 10,
            workgroup_size: (8, 8, 1),
        }
    }
}

impl BenchmarkConfig {
    pub fn new(workgroup_size: (u32, u32, u32)) -> Self {
        Self {
            workgroup_size,
            ..Default::default()
        }
    }

    pub fn with_warmup(mut self, warmup_iterations: u32) -> Self {
        self.warmup_iterations = warmup_iterations;
        self
    }

    pub fn with_iterations(mut self, timed_iterations: u32) -> Self {
        self.timed_iterations = timed_iterations;
        self
    }
}

#[derive(Debug, Clone)]
pub struct BenchmarkResult {
    pub workgroup_size: (u32, u32, u32),
    pub avg_duration: Duration,
    pub min_duration: Duration,
    pub max_duration: Duration,
    pub std_dev: Duration,
    pub durations: Vec<Duration>,
}

impl BenchmarkResult {
    pub fn throughput(&self, total_operations: u64) -> f64 {
        total_operations as f64 / self.avg_duration.as_secs_f64()
    }

    pub fn relative_performance(&self, baseline: &BenchmarkResult) -> f64 {
        baseline.avg_duration.as_secs_f64() / self.avg_duration.as_secs_f64()
    }

    pub fn print_summary(&self, label: Option<&str>) {
        let prefix = label.map(|l| format!("{}: ", l)).unwrap_or_default();
        println!("{}Workgroup Size: {:?}", prefix, self.workgroup_size);
        println!("  Avg: {:?}", self.avg_duration);
        println!("  Min: {:?}", self.min_duration);
        println!("  Max: {:?}", self.max_duration);
        println!("  StdDev: {:?}", self.std_dev);
    }
}

#[derive(Debug, Clone)]
pub struct BenchmarkSuite {
    pub results: Vec<BenchmarkResult>,
}

impl BenchmarkSuite {
    pub fn new() -> Self {
        Self {
            results: Vec::new(),
        }
    }

    pub fn add_result(&mut self, result: BenchmarkResult) {
        self.results.push(result);
    }

    pub fn fastest(&self) -> Option<&BenchmarkResult> {
        self.results
            .iter()
            .min_by_key(|r| r.avg_duration)
    }

    pub fn slowest(&self) -> Option<&BenchmarkResult> {
        self.results
            .iter()
            .max_by_key(|r| r.avg_duration)
    }

    pub fn print_summary(&self) {
        println!("\n=== Benchmark Suite Results ===");
        for (idx, result) in self.results.iter().enumerate() {
            result.print_summary(Some(&format!("Config {}", idx + 1)));
        }

        if let Some(fastest) = self.fastest() {
            println!("\nFastest Configuration:");
            println!("  Workgroup Size: {:?}", fastest.workgroup_size);
            println!("  Avg Duration: {:?}", fastest.avg_duration);
        }

        if self.results.len() > 1 {
            if let (Some(fastest), Some(slowest)) = (self.fastest(), self.slowest()) {
                let speedup = slowest.avg_duration.as_secs_f64() 
                    / fastest.avg_duration.as_secs_f64();
                println!("\n📊 Speedup: {:.2}x", speedup);
            }
        }
    }

    pub fn to_csv(&self) -> String {
        let mut csv = String::from("workgroup_x,workgroup_y,workgroup_z,avg_ms,min_ms,max_ms,stddev_ms\n");
        for result in &self.results {
            csv.push_str(&format!(
                "{},{},{},{:.3},{:.3},{:.3},{:.3}\n",
                result.workgroup_size.0,
                result.workgroup_size.1,
                result.workgroup_size.2,
                result.avg_duration.as_secs_f64() * 1000.0,
                result.min_duration.as_secs_f64() * 1000.0,
                result.max_duration.as_secs_f64() * 1000.0,
                result.std_dev.as_secs_f64() * 1000.0,
            ));
        }
        csv
    }
}

impl Default for BenchmarkSuite {
    fn default() -> Self {
        Self::new()
    }
}

pub trait BenchmarkableComputePipeline {
    type Input;
    
    /// The type of output data the pipeline produces
    type Output;

    fn create_with_workgroup(
        device: &GpuDevice,
        workgroup_size: (u32, u32, u32),
    ) -> Result<Self>
    where
        Self: Sized;


    fn execute_once(&mut self, device: &GpuDevice, input: &Self::Input) -> Result<Self::Output>;


    fn operation_count(&self) -> u64;

    fn benchmark_config(
        &mut self,
        device: &GpuDevice,
        input: &Self::Input,
        config: &BenchmarkConfig,
    ) -> Result<BenchmarkResult> {
        for _ in 0..config.warmup_iterations {
            let _ = self.execute_once(device, input)?;
        }

        let mut durations = Vec::with_capacity(config.timed_iterations as usize);
        for _ in 0..config.timed_iterations {
            let start = Instant::now();
            let _ = self.execute_once(device, input)?;
            durations.push(start.elapsed());
        }

        let avg_duration = durations.iter().sum::<Duration>() / durations.len() as u32;
        let min_duration = *durations.iter().min().unwrap();
        let max_duration = *durations.iter().max().unwrap();

        let variance: f64 = durations
            .iter()
            .map(|d| {
                let diff = d.as_secs_f64() - avg_duration.as_secs_f64();
                diff * diff
            })
            .sum::<f64>()
            / durations.len() as f64;
        let std_dev = Duration::from_secs_f64(variance.sqrt());

        Ok(BenchmarkResult {
            workgroup_size: config.workgroup_size,
            avg_duration,
            min_duration,
            max_duration,
            std_dev,
            durations,
        })
    }

    fn benchmark_suite(
        device: &GpuDevice,
        input: &Self::Input,
        configs: &[BenchmarkConfig],
    ) -> Result<BenchmarkSuite>
    where
        Self: Sized,
    {
        let mut suite = BenchmarkSuite::new();

        for config in configs {
            println!(
                "Benchmarking workgroup size: {:?}",
                config.workgroup_size
            );
            
            let mut pipeline = Self::create_with_workgroup(device, config.workgroup_size)?;
            let result = pipeline.benchmark_config(device, input, config)?;
            
            println!("  Avg: {:?}", result.avg_duration);
            suite.add_result(result);
        }

        Ok(suite)
    }

    fn common_2d_workgroup_sizes() -> Vec<(u32, u32, u32)> {
        vec![
            (4, 4, 1),
            (8, 8, 1),
            (16, 16, 1),
            (32, 32, 1),
            (8, 4, 1),
            (4, 8, 1),
            (16, 8, 1),
            (8, 16, 1),
            (32, 16, 1),
            (16, 32, 1),
        ]
    }

    fn common_1d_workgroup_sizes() -> Vec<(u32, u32, u32)> {
        vec![
            (32, 1, 1),
            (64, 1, 1),
            (128, 1, 1),
            (256, 1, 1),
            (512, 1, 1),
        ]
    }
}

