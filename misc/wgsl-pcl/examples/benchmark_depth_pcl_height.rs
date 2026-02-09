use nalgebra::Matrix4;
use std::fs;
use std::path::PathBuf;
use std::time::{Duration, Instant};
use wgsl_pcl::{
    gpu_types::AlignedMatrix4,
    map_layout::MapLayout,
    pipelines::{
        depth_to_obstacle::{
            ClearAffectedCellsOptions, DepthToPclAndHeightPipeline, ObstacleExpanderOptions,
        },
        filters::{BilateralOptions, BlurFilterOptions, GaussianOptions, OutlierFilterOptions},
    },
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

// Config from copperconfig.ron
struct BenchmarkParams {
    focal_length: f32,
    ppx: f32,
    ppy: f32,
    heightmap_max_x: f32,
    heightmap_max_y: f32,
    heightmap_min_x: f32,
    heightmap_min_y: f32,
    heightmap_cell_size: f32,
    bilateral_filter_kernel_radius: u32,
    bilateral_filter_sigma_spatial: u32,
    bilateral_filter_sigma_range: f32,
    gradient_filter_kernel_radius: u32,
    gaussian_blur_kernel_radius: u32,
    gaussian_sigma_spatial: f32,
    outlier_filter_kernel_radius: u32,
    outlier_filter_std_dev_threshold: f32,
    use_bilateral_filter: bool,
    robot_radius_meters: f32,
    min_depth: f32,
    max_depth: f32,
    clear_affected_cells: bool,
    min_distance_to_clear: f32,
    obstacle_gradient_threshold: f32,
}

impl Default for BenchmarkParams {
    fn default() -> Self {
        // Values from copperconfig.ron
        Self {
            focal_length: 383.0,
            ppx: 317.44882,
            ppy: 245.91605,
            heightmap_max_x: 10.0,
            heightmap_max_y: 10.0,
            heightmap_min_x: -5.0,
            heightmap_min_y: -10.0,
            heightmap_cell_size: 0.025,
            bilateral_filter_kernel_radius: 15,
            bilateral_filter_sigma_spatial: 15,
            bilateral_filter_sigma_range: 0.05,
            gradient_filter_kernel_radius: 3,
            gaussian_blur_kernel_radius: 3,
            gaussian_sigma_spatial: 3.0,
            outlier_filter_kernel_radius: 3,
            outlier_filter_std_dev_threshold: 3.5,
            use_bilateral_filter: true,
            robot_radius_meters: 0.1,
            min_depth: 0.2,
            max_depth: 3.0,
            clear_affected_cells: true,
            min_distance_to_clear: 2.0,
            obstacle_gradient_threshold: 1.0,
        }
    }
}

struct BenchmarkResult {
    workgroup_size_stage1: (u32, u32),
    workgroup_size_stage2: (u32, u32),
    avg_duration: Duration,
    min_duration: Duration,
    max_duration: Duration,
    std_dev: Duration,
}

/// Verification result for each pipeline stage
struct StageVerification {
    stage_name: &'static str,
    total_cells: usize,
    valid_cells: usize,     // cells with actual data (not f32::MIN)
    min_value: f32,
    max_value: f32,
}

impl StageVerification {
    fn print(&self) {
        let valid_pct = (self.valid_cells as f64 / self.total_cells as f64) * 100.0;
        println!(
            "    {:<30} {:>8} / {:<8} ({:>5.1}%) valid | range: [{:.4}, {:.4}]",
            self.stage_name,
            self.valid_cells,
            self.total_cells,
            valid_pct,
            self.min_value,
            self.max_value
        );
    }
}

fn analyze_heightmap(data: &[f32], stage_name: &'static str) -> StageVerification {
    let total_cells = data.len();
    let valid_cells = data.iter().filter(|&&v| v > f32::MIN).count();
    let valid_values: Vec<f32> = data.iter().copied().filter(|&v| v > f32::MIN).collect();
    
    let (min_value, max_value) = if valid_values.is_empty() {
        (f32::NAN, f32::NAN)
    } else {
        (
            valid_values.iter().copied().fold(f32::INFINITY, f32::min),
            valid_values.iter().copied().fold(f32::NEG_INFINITY, f32::max),
        )
    };
    
    StageVerification {
        stage_name,
        total_cells,
        valid_cells,
        min_value,
        max_value,
    }
}

fn verify_pipeline_stages(
    pipeline: &DepthToPclAndHeightPipeline,
    device: &wgsl_pcl::wgsl_setup::GpuDevice,
    point_cloud_len: usize,
) -> anyhow::Result<Vec<StageVerification>> {
    let mut verifications = Vec::new();
    
    // Stage 1: Raw height map (from depth_to_pcl_and_height)
    let raw_height = pipeline.get_raw_height_map(device)?;
    verifications.push(analyze_heightmap(&raw_height, "1. Raw Height Map"));
    
    // Stage 2: Outlier filtered height map
    let outlier_filtered = pipeline.get_outlier_filtered_height_map(device)?;
    verifications.push(analyze_heightmap(&outlier_filtered, "2. Outlier Filtered"));
    
    // Stage 3: Blur filtered height map
    let blur_filtered = pipeline.get_blur_filtered_height_map(device)?;
    verifications.push(analyze_heightmap(&blur_filtered, "3. Blur Filtered"));
    
    // Stage 4: Gradient map
    let gradient = pipeline.get_gradient_map(device)?;
    verifications.push(analyze_heightmap(&gradient, "4. Gradient Map"));
    
    // Stage 5: Obstacle expanded gradient map
    let expanded = pipeline.get_obstacle_expanded_height_map(device)?;
    verifications.push(analyze_heightmap(&expanded, "5. Obstacle Expanded"));
    
    // Also report point cloud stats
    println!("    {:<30} {:>8} points generated", "Point Cloud:", point_cloud_len);
    
    Ok(verifications)
}

fn create_pipeline(
    device: &wgsl_pcl::wgsl_setup::GpuDevice,
    params: &BenchmarkParams,
    depth_image_dimensions: (u32, u32),
    workgroup_size_stage1: (u32, u32),
    workgroup_size_stage2: (u32, u32),
) -> anyhow::Result<DepthToPclAndHeightPipeline> {
    let map_layout = MapLayout::new(
        params.heightmap_max_x,
        params.heightmap_min_x,
        params.heightmap_max_y,
        params.heightmap_min_y,
        params.heightmap_cell_size,
    );

    let blur_filter_options = if params.use_bilateral_filter {
        BlurFilterOptions::Bilateral(BilateralOptions {
            kernel_radius: params.bilateral_filter_kernel_radius,
            sigma_spatial: params.bilateral_filter_sigma_spatial,
            sigma_range: params.bilateral_filter_sigma_range,
        })
    } else {
        BlurFilterOptions::Gaussian(GaussianOptions {
            kernel_radius: params.gaussian_blur_kernel_radius,
            sigma: params.gaussian_sigma_spatial,
        })
    };

    let outlier_filter_options = OutlierFilterOptions {
        kernel_radius: params.outlier_filter_kernel_radius,
        std_dev_threshold: params.outlier_filter_std_dev_threshold,
    };

    let obstacle_expander_options = ObstacleExpanderOptions {
        expansion_radius_meters: params.robot_radius_meters,
        obstacle_gradient_threshold: params.obstacle_gradient_threshold,
    };

    let clear_affected_cells = if params.clear_affected_cells {
        Some(ClearAffectedCellsOptions {
            min_distance_to_clear: params.min_distance_to_clear,
        })
    } else {
        None
    };

    Ok(DepthToPclAndHeightPipeline::new(
        params.max_depth,
        depth_image_dimensions,
        params.focal_length,
        (params.ppx, params.ppy),
        device,
        workgroup_size_stage1,
        workgroup_size_stage2,
        map_layout,
        blur_filter_options,
        outlier_filter_options,
        obstacle_expander_options,
        params.gradient_filter_kernel_radius,
        params.min_depth,
        clear_affected_cells,
    )?)
}

fn benchmark_pipeline(
    device: &wgsl_pcl::wgsl_setup::GpuDevice,
    depths: &[u16],
    params: &BenchmarkParams,
    depth_image_dimensions: (u32, u32),
    workgroup_size_stage1: (u32, u32),
    workgroup_size_stage2: (u32, u32),
    warmup_iterations: u32,
    timed_iterations: u32,
) -> anyhow::Result<BenchmarkResult> {
    let mut pipeline = create_pipeline(
        device,
        params,
        depth_image_dimensions,
        workgroup_size_stage1,
        workgroup_size_stage2,
    )?;

    let camera_transform = AlignedMatrix4::from(Matrix4::<f32>::identity());
    let depth_scale = 0.001f32;

    // Warmup
    for _ in 0..warmup_iterations {
        let _ = pipeline.process(depths, device, camera_transform, depth_scale)?;
    }

    // Timed iterations
    let mut durations = Vec::with_capacity(timed_iterations as usize);
    for _ in 0..timed_iterations {
        let start = Instant::now();
        let _ = pipeline.process(depths, device, camera_transform, depth_scale)?;
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
        workgroup_size_stage1,
        workgroup_size_stage2,
        avg_duration,
        min_duration,
        max_duration,
        std_dev,
    })
}

fn main() -> anyhow::Result<()> {
    init_gpu_blocking()?;
    let device = get_device();

    let frames = find_depth_frames()?;
    if frames.is_empty() {
        anyhow::bail!("No depth frames found in benchmarking_frames/ or current directory");
    }

    let width = 640u32;
    let height = 480u32;
    let depths = load_depth_frame(frames[0].to_str().unwrap())?;
    let depths: Vec<u16> = depths.into_iter().take((width * height) as usize).collect();

    let params = BenchmarkParams::default();
    
    // Calculate heightmap dimensions
    let heightmap_width = ((params.heightmap_max_x - params.heightmap_min_x) / params.heightmap_cell_size).ceil() as u32;
    let heightmap_height = ((params.heightmap_max_y - params.heightmap_min_y) / params.heightmap_cell_size).ceil() as u32;

    println!("{}", "=".repeat(80));
    println!("DEPTH TO OBSTACLE PIPELINE BENCHMARK");
    println!("{}", "=".repeat(80));
    println!();
    println!("Input Configuration:");
    println!("  Depth image:     {}x{} pixels", width, height);
    println!("  Focal length:    {} px", params.focal_length);
    println!("  Principal point: ({}, {})", params.ppx, params.ppy);
    println!("  Depth range:     {} - {} m", params.min_depth, params.max_depth);
    println!();
    println!("Output Configuration:");
    println!("  Heightmap:       {}x{} cells", heightmap_width, heightmap_height);
    println!("  Cell size:       {} m", params.heightmap_cell_size);
    println!("  Map bounds:      X=[{}, {}] Y=[{}, {}] m", 
        params.heightmap_min_x, params.heightmap_max_x,
        params.heightmap_min_y, params.heightmap_max_y);
    println!();
    
    // Print pipeline stages
    println!("Pipeline Stages:");
    let num_stages = if params.clear_affected_cells { 6 } else { 5 };
    let mut stage_num = 1;
    
    if params.clear_affected_cells {
        println!("  {}. Clear Affected Cells (min distance: {} m)", stage_num, params.min_distance_to_clear);
        stage_num += 1;
    }
    println!("  {}. Depth to PCL + Height Map", stage_num);
    stage_num += 1;
    println!("  {}. Outlier Removal Filter (kernel: {}, threshold: {} std dev)", 
        stage_num, params.outlier_filter_kernel_radius, params.outlier_filter_std_dev_threshold);
    stage_num += 1;
    if params.use_bilateral_filter {
        println!("  {}. Bilateral Blur Filter (kernel: {}, sigma_s: {}, sigma_r: {})", 
            stage_num, params.bilateral_filter_kernel_radius, 
            params.bilateral_filter_sigma_spatial, params.bilateral_filter_sigma_range);
    } else {
        println!("  {}. Gaussian Blur Filter (kernel: {}, sigma: {})", 
            stage_num, params.gaussian_blur_kernel_radius, params.gaussian_sigma_spatial);
    }
    stage_num += 1;
    println!("  {}. Gradient Computation (kernel: {})", stage_num, params.gradient_filter_kernel_radius);
    stage_num += 1;
    println!("  {}. Obstacle Expansion (radius: {} m, threshold: {})", 
        stage_num, params.robot_radius_meters, params.obstacle_gradient_threshold);
    println!();
    println!("Total stages: {}", num_stages);
    println!();

    // Verify pipeline stages are working
    println!("{}", "-".repeat(80));
    println!("PIPELINE VERIFICATION");
    println!("{}", "-".repeat(80));
    println!("Running single execution to verify all stages produce output...\n");
    
    let mut test_pipeline = create_pipeline(device, &params, (width, height), (8, 8), (8, 8))?;
    let camera_transform = AlignedMatrix4::from(Matrix4::<f32>::identity());
    let (pcl, _gradient) = test_pipeline.process(&depths, device, camera_transform, 0.001)?;
    
    println!("Stage output verification:");
    let verifications = verify_pipeline_stages(&test_pipeline, device, pcl.len())?;
    for v in &verifications {
        v.print();
    }
    
    // Check if stages are producing meaningful output
    let all_stages_ok = verifications.iter().all(|v| v.valid_cells > 0);
    if all_stages_ok {
        println!("\n  [OK] All stages producing valid output");
    } else {
        println!("\n  [WARNING] Some stages have no valid output - check input data");
    }
    println!();

    // Workgroup sizes to test for stage 1 (depth -> pcl + height map)
    let stage1_workgroup_sizes: Vec<(u32, u32)> = vec![
        (4, 4),
        (8, 8),
        (16, 16),
        (32, 32),
        (8, 16),
        (16, 8),
        (32, 16),
        (16, 32),
        (32, 8),
    ];

    // Fixed stage 2 workgroup size for initial comparison
    let stage2_workgroup_size = (8, 8);

    let warmup_iterations = 10;
    let timed_iterations = 50;

    println!("{}", "-".repeat(80));
    println!("BENCHMARK RESULTS");
    println!("{}", "-".repeat(80));
    println!(
        "Testing {} stage1 workgroup configurations ({} warmup, {} timed iterations)",
        stage1_workgroup_sizes.len(),
        warmup_iterations,
        timed_iterations
    );
    println!("Stage 2 workgroup size fixed at: {}x{}", stage2_workgroup_size.0, stage2_workgroup_size.1);
    println!();

    let mut results = Vec::new();

    for (i, &ws1) in stage1_workgroup_sizes.iter().enumerate() {
        print!(
            "[{}/{}] Stage1: {}x{} ... ",
            i + 1,
            stage1_workgroup_sizes.len(),
            ws1.0,
            ws1.1
        );
        std::io::Write::flush(&mut std::io::stdout()).ok();

        match benchmark_pipeline(
            device,
            &depths,
            &params,
            (width, height),
            ws1,
            stage2_workgroup_size,
            warmup_iterations,
            timed_iterations,
        ) {
            Ok(result) => {
                println!("{:.3}ms", result.avg_duration.as_secs_f64() * 1000.0);
                results.push(result);
            }
            Err(e) => {
                println!("FAILED: {}", e);
            }
        }
    }

    println!(
        "\n{:<15} {:<12} {:<12} {:<12} {:<12} {:<8}",
        "Stage1 WG", "Avg (ms)", "Min (ms)", "Max (ms)", "StdDev (ms)", "CV (%)"
    );
    println!("{}", "-".repeat(80));

    let mut results_with_cv: Vec<_> = results
        .iter()
        .map(|r| {
            let cv = r.std_dev.as_secs_f64() / r.avg_duration.as_secs_f64() * 100.0;
            (r, cv)
        })
        .collect();

    for (result, cv) in &results_with_cv {
        let ws1 = result.workgroup_size_stage1;
        println!(
            "{:<15} {:<12.3} {:<12.3} {:<12.3} {:<12.3} {:<8.2}",
            format!("{}x{}", ws1.0, ws1.1),
            result.avg_duration.as_secs_f64() * 1000.0,
            result.min_duration.as_secs_f64() * 1000.0,
            result.max_duration.as_secs_f64() * 1000.0,
            result.std_dev.as_secs_f64() * 1000.0,
            cv
        );
    }

    results_with_cv.sort_by(|a, b| a.0.avg_duration.cmp(&b.0.avg_duration));

    println!("\nTop 5 by Performance:");
    for (i, (result, cv)) in results_with_cv.iter().take(5).enumerate() {
        let ws1 = result.workgroup_size_stage1;
        let fps = 1000.0 / (result.avg_duration.as_secs_f64() * 1000.0);
        println!(
            "  {}. Stage1={}x{}: {:.3}ms ({:.1} FPS | CV={:.2}%)",
            i + 1,
            ws1.0,
            ws1.1,
            result.avg_duration.as_secs_f64() * 1000.0,
            fps,
            cv
        );
    }

    results_with_cv.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());

    println!("\nTop 5 by Stability (lowest CV):");
    for (i, (result, cv)) in results_with_cv.iter().take(5).enumerate() {
        let ws1 = result.workgroup_size_stage1;
        println!(
            "  {}. Stage1={}x{}: CV={:.2}% (avg={:.3}ms)",
            i + 1,
            ws1.0,
            ws1.1,
            cv,
            result.avg_duration.as_secs_f64() * 1000.0
        );
    }

    // Find fastest and recommend
    results_with_cv.sort_by(|a, b| a.0.avg_duration.cmp(&b.0.avg_duration));
    if let Some((fastest, cv)) = results_with_cv.first() {
        let ws1 = fastest.workgroup_size_stage1;
        let fps = 1000.0 / (fastest.avg_duration.as_secs_f64() * 1000.0);
        println!();
        println!("{}", "=".repeat(80));
        println!(
            "RECOMMENDATION: Stage1={}x{} ({:.3}ms | {:.1} FPS | CV={:.2}%)",
            ws1.0,
            ws1.1,
            fastest.avg_duration.as_secs_f64() * 1000.0,
            fps,
            cv
        );
        println!("{}", "=".repeat(80));
    }

    // Save CSV
    let csv_filename = format!("benchmark_{}x{}.csv", width, height);
    let mut csv = String::from("stage1_wg_x,stage1_wg_y,stage2_wg_x,stage2_wg_y,avg_ms,min_ms,max_ms,stddev_ms\n");
    for result in &results {
        csv.push_str(&format!(
            "{},{},{},{},{:.3},{:.3},{:.3},{:.3}\n",
            result.workgroup_size_stage1.0,
            result.workgroup_size_stage1.1,
            result.workgroup_size_stage2.0,
            result.workgroup_size_stage2.1,
            result.avg_duration.as_secs_f64() * 1000.0,
            result.min_duration.as_secs_f64() * 1000.0,
            result.max_duration.as_secs_f64() * 1000.0,
            result.std_dev.as_secs_f64() * 1000.0,
        ));
    }
    fs::write(&csv_filename, csv)?;
    println!("\nResults saved to: {}", csv_filename);

    Ok(())
}
