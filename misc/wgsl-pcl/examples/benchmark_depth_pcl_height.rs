use std::fs;
use std::path::PathBuf;
use nalgebra::Matrix4;
use wgsl_pcl::{
    benchmark::{BenchmarkableComputePipeline, BenchmarkConfig},
    gpu_types::AlignedMatrix4,
    pipelines::depth_to_pcl_and_height::{DepthToPclBenchmarkInput, DepthToPclAndHeightPipeline},
    wgsl_setup::{get_device, init_gpu_blocking},
};

fn load_depth_frame(path: &str) -> anyhow::Result<Vec<u16>> {
    let bytes = fs::read(path)?;
    Ok(bytemuck::cast_slice(&bytes).to_vec())
}

fn find_depth_frames() -> anyhow::Result<Vec<PathBuf>> {
    let mut frames = Vec::new();
    let bench_dir = PathBuf::from("benchmarking_frames");
    
    let search_dir = if bench_dir.exists() && bench_dir.is_dir() {
        bench_dir
    } else {
        PathBuf::from(".")
    };
    
    for entry in fs::read_dir(&search_dir)? {
        let entry = entry?;
        let path = entry.path();
        if let Some(filename) = path.file_name().and_then(|f| f.to_str()) {
            if filename.starts_with("depth_frame_") && filename.ends_with(".bin") {
                frames.push(path);
            }
        }
    }
    
    frames.sort();
    Ok(frames)
}

fn main() -> anyhow::Result<()> {
    init_gpu_blocking()?;
    let device = get_device();

    let frames = find_depth_frames()?;
    if frames.is_empty() {
        anyhow::bail!("No depth frames found in benchmarking_frames/ or current directory");
    }
    
    let depths = load_depth_frame(frames[0].to_str().unwrap())?;
    let width = 640u32;
    let height = 480u32;
    let depths: Vec<u16> = depths.into_iter().take((width * height) as usize).collect();
    
    let input = DepthToPclBenchmarkInput {
        depths,
        camera_transform: AlignedMatrix4::from(Matrix4::<f32>::identity()),
        depth_scale: 0.001,
        max_depth: 3.0,
        depth_image_dimensions: (width, height),
        focal_length_px: 383.0,
        principal_point_px: (317.44882, 245.91605),
    };

    println!("Configuration: {}x{} -> 400x400 heightmap", width, height);
    println!("Stage 1: Depth->PCL+Heightmap | Stage 2: Bilateral Filter (8x8 fixed)\n");

    let workgroup_sizes = vec![
        (4, 4, 1), (8, 8, 1), (16, 16, 1), (32, 32, 1),
        (8, 16, 1), (16, 8, 1), (32, 16, 1), (16, 32, 1),
        (64, 1, 1), (32, 2, 1), (32, 4, 1), (32, 8, 1),
    ];

    let configs: Vec<BenchmarkConfig> = workgroup_sizes
        .into_iter()
        .map(|ws| {
            BenchmarkConfig::new(ws)
                .with_warmup(10)
                .with_iterations(50)
        })
        .collect();

    println!("Benchmarking {} configurations (10 warmup, 50 timed iterations)\n", configs.len());
    
    let suite = DepthToPclAndHeightPipeline::benchmark_suite(device, &input, &configs)?;

    println!("\n{:<12} {:<12} {:<12} {:<12} {:<12} {:<8}", 
        "Workgroup", "Avg (ms)", "Min (ms)", "Max (ms)", "StdDev (ms)", "CV (%)");
    println!("{}", "-".repeat(80));
    
    let mut results_with_cv: Vec<_> = suite.results.iter().map(|r| {
        let cv = r.std_dev.as_secs_f64() / r.avg_duration.as_secs_f64() * 100.0;
        (r, cv)
    }).collect();
    
    for (result, cv) in &results_with_cv {
        let ws = result.workgroup_size;
        println!("{:<12} {:<12.3} {:<12.3} {:<12.3} {:<12.3} {:<8.2}", 
            format!("{}x{}x{}", ws.0, ws.1, ws.2),
            result.avg_duration.as_secs_f64() * 1000.0,
            result.min_duration.as_secs_f64() * 1000.0,
            result.max_duration.as_secs_f64() * 1000.0,
            result.std_dev.as_secs_f64() * 1000.0,
            cv);
    }
    
    results_with_cv.sort_by(|a, b| a.0.avg_duration.cmp(&b.0.avg_duration));
    
    println!("\nTop 5 by Performance:");
    for (i, (result, cv)) in results_with_cv.iter().take(5).enumerate() {
        let ws = result.workgroup_size;
        let fps = 1000.0 / (result.avg_duration.as_secs_f64() * 1000.0);
        println!("  {}. {}x{}x{}: {:.3}ms ({:.1} FPS | CV={:.2}%)", 
            i + 1, ws.0, ws.1, ws.2,
            result.avg_duration.as_secs_f64() * 1000.0, fps, cv);
    }
    
    results_with_cv.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
    
    println!("\nTop 5 by Stability (lowest CV):");
    for (i, (result, cv)) in results_with_cv.iter().take(5).enumerate() {
        let ws = result.workgroup_size;
        println!("  {}. {}x{}x{}: CV={:.2}% (avg={:.3}ms)", 
            i + 1, ws.0, ws.1, ws.2, cv,
            result.avg_duration.as_secs_f64() * 1000.0);
    }
    
    if let Some(fastest) = suite.fastest() {
        let ws = fastest.workgroup_size;
        let fps = 1000.0 / (fastest.avg_duration.as_secs_f64() * 1000.0);
        let cv = fastest.std_dev.as_secs_f64() / fastest.avg_duration.as_secs_f64() * 100.0;
        println!("\nRecommendation: {}x{}x{} ({:.3}ms | {:.1} FPS | CV={:.2}%)",
            ws.0, ws.1, ws.2,
            fastest.avg_duration.as_secs_f64() * 1000.0,
            fps, cv);
    }
    
    let csv_filename = format!("benchmark_{}x{}.csv", width, height);
    fs::write(&csv_filename, suite.to_csv())?;
    println!("\nResults saved to: {}", csv_filename);

    Ok(())
}

