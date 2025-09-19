use std::num::NonZeroU32;

use gputter::{
    buffers::{
        storage::{HostReadOnly, ShaderReadWrite, StorageBuffer},
        uniform::UniformBuffer,
        GpuBufferSet,
    },
    compute::ComputePipeline,
    shader::BufferGroupBinding,
    types::AlignedMatrix4,
};
use nalgebra::{Vector2, Vector3};

use crate::{
    adjust_for_camera_position::AdjustOccupancyForCameraPosition, clear_cells::ClearCells,
    expand_occupancy::ExpandOccupancy, occupancy_normalize::OccupancyNormalize,
    pcl2occupancy::Pcl2Occupancy, PipelineSharedItems, ExpanderBindGrp, Occupancy,
    OccupancyGridBindGroups, Pcl2OccupancyBindGrp,
};

pub struct OccupancyGridPipeline {
    pipeline: ComputePipeline<OccupancyGridBindGroups, 5>,
    grid_dimensions: Vector2<NonZeroU32>,
    bind_grps: Option<(
        GpuBufferSet<Pcl2OccupancyBindGrp>, // uniforms
        GpuBufferSet<(StorageBuffer<[u32], HostReadOnly, ShaderReadWrite>,)>, // raw occupancy
        GpuBufferSet<(StorageBuffer<[u32], HostReadOnly, ShaderReadWrite>,)>, // normalized occupancy
        GpuBufferSet<ExpanderBindGrp>,
    )>,
    pub occupancy_grid_ref: PipelineSharedItems,
    neighborhood_radius: u32,
    min_known_neighbors_ratio: u32,
}

impl OccupancyGridPipeline {
    pub fn will_process(&self) -> bool {
        self.occupancy_grid_ref.shared.lock().1
    }

    pub fn process(
        &mut self,
        robot_radius_m: f32,
        cell_size: f32,
        occupancy_grid_out: &mut [Occupancy],
        camera_transform: &AlignedMatrix4<f32>,
    ) {
        let mut shared_lock = self.occupancy_grid_ref.shared.lock();

        if !shared_lock.1 {
            return;
        }

        let mut shared = shared_lock.0.take().unwrap();
        let (uniforms_grp, raw_occupancy_grp, normalized_occupancy_grp, expander_grp) =
            self.bind_grps.take().unwrap();

        let mut bind_grps: OccupancyGridBindGroups = (
            shared.points,
            uniforms_grp,
            raw_occupancy_grp,
            normalized_occupancy_grp,
            expander_grp,
        );

        // Set workgroups for both stages
        self.pipeline.workgroups[0] = Vector3::new(
            shared.image_dimensions.x.div_ceil(8),
            shared.image_dimensions.y.div_ceil(8),
            1,
        );
        self.pipeline.workgroups[1] = Vector3::new(
            shared.image_dimensions.x.div_ceil(8),
            shared.image_dimensions.y.div_ceil(8),
            1,
        );
        self.pipeline.workgroups[1] = Vector3::new(
            shared.image_dimensions.x.div_ceil(8),
            shared.image_dimensions.y.div_ceil(8),
            1,
        );
        self.pipeline.workgroups[2] = Vector3::new(
            self.grid_dimensions.x.get().div_ceil(8),
            self.grid_dimensions.y.get().div_ceil(8),
            1,
        );

        self.pipeline
            .new_pass(|mut lock| {
                bind_grps
                    .1
                    .write::<0, _>(&shared.image_dimensions.into(), &mut lock);
                bind_grps
                    .1
                    .write::<1, _>(&self.neighborhood_radius, &mut lock);
                bind_grps
                    .1
                    .write::<2, _>(&self.min_known_neighbors_ratio, &mut lock);
                bind_grps.1.write::<3, _>(camera_transform, &mut lock);
                let radius_in_cells = (robot_radius_m / cell_size).ceil() as u32;
                bind_grps.4.write::<1, _>(&radius_in_cells, &mut lock);
                &mut bind_grps
            })
            .finish();

        let (points_grp, uniforms_grp, raw_occupancy_grp, normalized_occupancy_grp, expander_grp) =
            bind_grps;

        expander_grp
            .buffers
            .0
            .read(bytemuck::cast_slice_mut(occupancy_grid_out));

        self.bind_grps = Some((
            uniforms_grp,
            raw_occupancy_grp,
            normalized_occupancy_grp,
            expander_grp,
        ));
        shared.points = points_grp;
        shared_lock.0.replace(shared);
        shared_lock.1 = false;
    }
}

pub struct OccupancyGridPipelineBuilder {
    pub occupancy_grid_dimensions: Vector2<NonZeroU32>,
    pub cell_size: f32,
    /// The radius (in cells) that each cell is compared against to score obsticallyness
    pub neighborhood_radius: u32,
    /// If a cell has > min_known_neighbors_ratio of cells in its neighborhood that have an unknown status,
    /// then the cell will also be marked unknown.
    pub min_known_neighbors_ratio: u32,
    /// The number of points that have to have to fall in a cell for it to be considered to be a known cell
    /// this prevents outliers from the realsense being less accurate and noisier further away.
    pub min_points_for_occupied: u32,
    /// Each cell is scored from 1-100 on how occupied it seems to be, and this threshold is what determines
    /// the limit for what is considered an obstacle and what isnt.
    pub obstacle_threshold: NonZeroU32,
}

impl OccupancyGridPipelineBuilder {
    pub fn build(self) -> OccupancyGridPipeline {
        let cell_count =
            self.occupancy_grid_dimensions.x.get() * self.occupancy_grid_dimensions.y.get();
        let cell_count = NonZeroU32::new(cell_count).unwrap();

        let [clear_map] = ClearCells {
            cell_size: self.cell_size,
            obstacle_map: BufferGroupBinding::<_, OccupancyGridBindGroups>::get::<2, 0>(),
            points: BufferGroupBinding::<_, OccupancyGridBindGroups>::get::<0, 0>(),
            image_dimensions: BufferGroupBinding::<_, OccupancyGridBindGroups>::get::<1, 0>(),
            heightmap_width: self.occupancy_grid_dimensions.x,
            cell_count,
        }
        .compile();

        let [pcl2occupancy] = Pcl2Occupancy {
            cell_size: self.cell_size,
            obstacle_map: BufferGroupBinding::<_, OccupancyGridBindGroups>::get::<2, 0>(),
            points: BufferGroupBinding::<_, OccupancyGridBindGroups>::get::<0, 0>(),
            image_dimensions: BufferGroupBinding::<_, OccupancyGridBindGroups>::get::<1, 0>(),
            heightmap_width: self.occupancy_grid_dimensions.x,
            cell_count,
            camera_transform: BufferGroupBinding::<_, OccupancyGridBindGroups>::get::<1, 3>(),
        }
        .compile();

        let [adjust_for_camera_position] = AdjustOccupancyForCameraPosition {
            obstacle_map: BufferGroupBinding::<_, OccupancyGridBindGroups>::get::<2, 0>(),
            points: BufferGroupBinding::<_, OccupancyGridBindGroups>::get::<0, 0>(),
            image_dimensions: BufferGroupBinding::<_, OccupancyGridBindGroups>::get::<1, 0>(),
            camera_transform: BufferGroupBinding::<_, OccupancyGridBindGroups>::get::<1, 3>(),
            heightmap_width: self.occupancy_grid_dimensions.x,
            cell_count,
            cell_size: self.cell_size,
        }
        .compile();

        let [occupancy_normalizer] =
            OccupancyNormalize {
                raw_occupancy: BufferGroupBinding::<_, OccupancyGridBindGroups>::get::<2, 0>(),
                normalized_occupancy: BufferGroupBinding::<_, OccupancyGridBindGroups>::get::<3, 0>(
                ),
                min_points_for_known: self.min_points_for_occupied,
                neighborhood_radius: BufferGroupBinding::<_, OccupancyGridBindGroups>::get::<1, 1>(
                ),
                cell_count,
                grid_width: self.occupancy_grid_dimensions.x,
                grid_height: self.occupancy_grid_dimensions.y,
                min_known_neighbors_ratio: BufferGroupBinding::<_, OccupancyGridBindGroups>::get::<
                    1,
                    2,
                >(),
            }
            .compile();

        let [occupancy_expander] =
            ExpandOccupancy {
                normalized_occupancy_grid: BufferGroupBinding::<_, OccupancyGridBindGroups>::get::<
                    3,
                    0,
                >(),
                expanded_obstacles: BufferGroupBinding::<_, OccupancyGridBindGroups>::get::<4, 0>(),
                radius_in_cells: BufferGroupBinding::<_, OccupancyGridBindGroups>::get::<4, 1>(),
                grid_width: self.occupancy_grid_dimensions.x,
                grid_height: self.occupancy_grid_dimensions.y,
                obstacle_threshold: self.obstacle_threshold,
            }
            .compile();

        let mut pipeline = ComputePipeline::new([
            &clear_map,
            &pcl2occupancy,
            &adjust_for_camera_position,
            &occupancy_normalizer,
            &occupancy_expander,
        ]);

        pipeline.workgroups = [
            Vector3::new(1, 1, 1), // set in process()
            Vector3::new(1, 1, 1), // set in process()
            Vector3::new(1, 1, 1), // set in process()
            Vector3::new(
                self.occupancy_grid_dimensions.x.get().div_ceil(8),
                self.occupancy_grid_dimensions.y.get().div_ceil(8),
                1,
            ),
            Vector3::new(
                self.occupancy_grid_dimensions.x.get().div_ceil(8),
                self.occupancy_grid_dimensions.y.get().div_ceil(8),
                1,
            ),
        ];

        let bind_grps = Some((
            GpuBufferSet::from((
                UniformBuffer::new(), // image_dimensions
                UniformBuffer::new(), // neighborhood_radius
                UniformBuffer::new(), // min_known_neighbors_ratio
                UniformBuffer::new(), // camera transform
            )),
            GpuBufferSet::from((StorageBuffer::new_dyn(cell_count.get() as usize).unwrap(),)), // raw occupancy
            GpuBufferSet::from((StorageBuffer::new_dyn(cell_count.get() as usize).unwrap(),)), // normalized occupancy
            GpuBufferSet::from((
                StorageBuffer::new_dyn(cell_count.get() as usize).unwrap(),
                UniformBuffer::new(),
            )),
        ));

        OccupancyGridPipeline {
            pipeline,
            grid_dimensions: self.occupancy_grid_dimensions,
            bind_grps,
            occupancy_grid_ref: PipelineSharedItems::noop(),
            neighborhood_radius: self.neighborhood_radius,
            min_known_neighbors_ratio: self.min_known_neighbors_ratio,
        }
    }
}
