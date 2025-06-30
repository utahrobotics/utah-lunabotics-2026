use std::{num::NonZeroU32, sync::Arc};

use bytemuck::{Pod, Zeroable};
use depth2pcl::Depth2Pcl;
use gputter::{
    buffers::{
        storage::{HostReadOnly, HostWriteOnly, ShaderReadOnly, ShaderReadWrite, StorageBuffer},
        uniform::UniformBuffer,
        GpuBufferSet,
    },
    compute::ComputePipeline,
    shader::BufferGroupBinding,
    types::{AlignedMatrix4, AlignedVec2, AlignedVec4},
};
use nalgebra::{Vector2, Vector3};

mod depth2pcl;
mod obstaclefilter;
mod pcl2obstacle;

mod expand_obstacles;
use expand_obstacles::ExpandObstacles;
use obstaclefilter::ObstacleFilter;
use parking_lot::Mutex;
use pcl2obstacle::Pcl2Obstacle;
// use pcl2sum::Pcl2Sum;
// use sum2height::Sum2Height;

/// 1. Depths in arbitrary units
/// 2. Global Transform of the camera
/// 3. Depth Scale (meters per depth unit)
///
/// This bind group serves as the input for all DepthProjectors
type DepthBindGrp = (
    StorageBuffer<[u32], HostWriteOnly, ShaderReadOnly>,
    UniformBuffer<AlignedMatrix4<f32>>,
    UniformBuffer<f32>,
    // UniformBuffer<u32>,
);

type PointCloudGrp = (StorageBuffer<[AlignedVec4<f32>], HostReadOnly, ShaderReadWrite>,);

/// The set of bind groups used by the DepthProjector
type AlphaBindGroups = (GpuBufferSet<DepthBindGrp>, GpuBufferSet<PointCloudGrp>);

type Pcl2ObstacleBindGrp = (UniformBuffer<AlignedVec2<u32>>, UniformBuffer<f32>);

type UnfilteredObstacleMapBindGrp = (StorageBuffer<[u32], HostReadOnly, ShaderReadWrite>,);

type FilteredObstacleMapBindGrp = (StorageBuffer<[u32], HostReadOnly, ShaderReadWrite>,);

/// 1. The radius of the robot in meters
///
/// This bind group is the input to the expander.
type ExpanderBindGrp = (
    StorageBuffer<[u32], HostReadOnly, ShaderReadWrite>,
    UniformBuffer<u32>,
);

/// The set of bind groups used by the rest of the thalassic pipeline
type BetaBindGroups = (
    GpuBufferSet<PointCloudGrp>,
    GpuBufferSet<Pcl2ObstacleBindGrp>,
    GpuBufferSet<UnfilteredObstacleMapBindGrp>,
    GpuBufferSet<FilteredObstacleMapBindGrp>,
    GpuBufferSet<ExpanderBindGrp>,
);

#[derive(Debug, Clone, Copy)]
pub struct DepthProjectorBuilder {
    pub image_size: Vector2<NonZeroU32>,
    pub focal_length_px: f32,
    pub principal_point_px: Vector2<f32>,
    pub max_depth: f32,
}

impl DepthProjectorBuilder {
    pub fn build(self, thalassic_ref: ThalassicPipelineRef) -> DepthProjector {
        let pixel_count = self.image_size.x.get() * self.image_size.y.get();
        let stride = std::env::var("STRIDE").unwrap_or("4".into()).parse::<u32>().expect("STRIDE MUST BE A U32");
        let [depth_fn] = Depth2Pcl {
            depths: BufferGroupBinding::<_, AlphaBindGroups>::get::<0, 0>(),
            points: BufferGroupBinding::<_, AlphaBindGroups>::get::<1, 0>(),
            transform: BufferGroupBinding::<_, AlphaBindGroups>::get::<0, 1>(),
            depth_scale: BufferGroupBinding::<_, AlphaBindGroups>::get::<0, 2>(),
            max_depth: self.max_depth,
            image_width: self.image_size.x,
            focal_length_px: self.focal_length_px,
            principal_point_px: self.principal_point_px.into(),
            pixel_count: NonZeroU32::new(pixel_count).unwrap(),
            half_pixel_count: NonZeroU32::new(pixel_count.div_ceil(2)).unwrap(),
            stride: NonZeroU32::new(stride).unwrap(),
        }
        .compile();

        let mut pipeline = ComputePipeline::new([&depth_fn]);
        pipeline.workgroups = [Vector3::new(
            self.image_size.x.get() / 8,
            self.image_size.y.get() / 8,
            1,
        )];
        thalassic_ref.shared.lock().0.replace(Shared {
            points: GpuBufferSet::from((StorageBuffer::new_dyn(pixel_count as usize).unwrap(),)),
            image_dimensions: AlignedVec2::from(Vector2::new(
                self.image_size.x.get(),
                self.image_size.y.get(),
            )),
        });
        DepthProjector {
            image_size: self.image_size,
            pipeline,
            depth_bind_grp: Some(GpuBufferSet::from((
                StorageBuffer::new_dyn(pixel_count.div_ceil(2) as usize).unwrap(),
                UniformBuffer::new(),
                UniformBuffer::new(),
            ))),
            thalassic_ref,
        }
    }
}

pub struct DepthProjector {
    image_size: Vector2<NonZeroU32>,
    pipeline: ComputePipeline<AlphaBindGroups, 1>,
    depth_bind_grp: Option<GpuBufferSet<DepthBindGrp>>,
    thalassic_ref: ThalassicPipelineRef,
}

impl DepthProjector {
    pub fn project(
        &mut self,
        depths: &[u16],
        camera_transform: &AlignedMatrix4<f32>,
        depth_scale: f32,
        point_cloud: Option<&mut [AlignedVec4<f32>]>,
    ) {
        let point_count = self.image_size.x.get() * self.image_size.y.get();
        debug_assert_eq!(depths.len(), point_count as usize);

        let depth_grp = self.depth_bind_grp.take().unwrap();
        let mut shared_lock = self.thalassic_ref.shared.lock();
        let mut shared = shared_lock.0.take().unwrap();

        let mut bind_grps = (depth_grp, shared.points);

        self.pipeline
            .new_pass(|mut lock| {
                // We have to write raw bytes because we can only cast to [u32] if the number of
                // depth pixels is even
                bind_grps
                    .0
                    .write_raw::<0>(bytemuck::cast_slice(depths), &mut lock);
                bind_grps.0.write::<1, _>(camera_transform, &mut lock);
                bind_grps.0.write::<2, _>(&depth_scale, &mut lock);
                &mut bind_grps
            })
            .finish();
        let (depth_grp, points_grp) = bind_grps;
        if let Some(point_cloud) = point_cloud {
            points_grp.buffers.0.read(point_cloud);
        }
        self.depth_bind_grp = Some(depth_grp);
        shared.points = points_grp;
        shared_lock.0.replace(shared);
        shared_lock.1 = true;
    }

    pub fn get_image_size(&self) -> Vector2<NonZeroU32> {
        self.image_size
    }

    pub fn get_pixel_count(&self) -> NonZeroU32 {
        NonZeroU32::new(self.image_size.x.get() * self.image_size.y.get()).unwrap()
    }
}

#[derive(Debug, Clone, Copy)]
pub struct ThalassicBuilder {
    pub heightmap_dimensions: Vector2<NonZeroU32>,
    pub cell_size: f32,
    pub max_point_count: NonZeroU32,
    pub feature_size_cells: u32,
    pub min_feature_count: u32,
    // pub image_dimensions: Vector2<NonZeroU32>,
}

impl ThalassicBuilder {
    pub fn build(self) -> ThalassicPipeline {
        let cell_count = self.heightmap_dimensions.x.get() * self.heightmap_dimensions.y.get();
        let cell_count = NonZeroU32::new(cell_count).unwrap();

        let [pcl2obstacle] = Pcl2Obstacle {
            cell_size: self.cell_size,
            obstacle_map: BufferGroupBinding::<_, BetaBindGroups>::get::<2, 0>(),
            points: BufferGroupBinding::<_, BetaBindGroups>::get::<0, 0>(),
            max_safe_gradient: BufferGroupBinding::<_, BetaBindGroups>::get::<1, 1>(),
            image_dimensions: BufferGroupBinding::<_, BetaBindGroups>::get::<1, 0>(),
            heightmap_width: self.heightmap_dimensions.x,
            cell_count,
        }
        .compile();

        let [obstacle_filter] = ObstacleFilter {
            in_obstacles: BufferGroupBinding::<_, BetaBindGroups>::get::<2, 0>(),
            filtered_obstacles: BufferGroupBinding::<_, BetaBindGroups>::get::<3, 0>(),
            feature_size_cells: self.feature_size_cells,
            min_count: self.min_feature_count,
            cell_size: self.cell_size,
            grid_width: self.heightmap_dimensions.x,
            grid_height: self.heightmap_dimensions.y,
        }
        .compile();

        let [expand_fn] = ExpandObstacles {
            filtered_obstacles: BufferGroupBinding::<_, BetaBindGroups>::get::<3, 0>(),
            expanded_obstacles: BufferGroupBinding::<_, BetaBindGroups>::get::<4, 0>(),
            radius_in_cells: BufferGroupBinding::<_, BetaBindGroups>::get::<4, 1>(),
            grid_width: self.heightmap_dimensions.x,
            grid_height: self.heightmap_dimensions.y,
        }
        .compile();

        let mut pipeline = ComputePipeline::new([&pcl2obstacle, &obstacle_filter, &expand_fn]);
        pipeline.workgroups = [Vector3::new(
            self.heightmap_dimensions.x.get() / 8,
            self.heightmap_dimensions.y.get() / 8,
            1,
        ); 3];

        let bind_grps = (
            GpuBufferSet::from((UniformBuffer::new(), UniformBuffer::new())),
            GpuBufferSet::from((StorageBuffer::new_dyn(cell_count.get() as usize).unwrap(),)),
            GpuBufferSet::from((StorageBuffer::new_dyn(cell_count.get() as usize).unwrap(),)),
            GpuBufferSet::from((
                StorageBuffer::new_dyn(cell_count.get() as usize).unwrap(),
                UniformBuffer::new(),
            )),
        );

        ThalassicPipeline {
            pipeline,
            cell_size: self.cell_size,
            bind_grps: Some(bind_grps),
            new_radius: Some(0.25),
            new_max_gradient: Some(45.0f32.to_radians()),
            thalassic_ref: ThalassicPipelineRef {
                shared: Arc::new(Mutex::new((None, false))),
            },
            cell_count,
        }
    }
}

#[repr(transparent)]
#[derive(Debug, Clone, Copy, Pod, Zeroable, PartialEq, Eq)]
pub struct Occupancy(u32);

impl Occupancy {
    pub const UNKNOWN: Self = Self(0);
    pub const FREE: Self = Self(1);
    pub const OCCUPIED: Self = Self(2);

    pub fn occupied(self) -> bool {
        // True iff the cell is not empty
        self.0 == 2
    }
}

struct Shared {
    points: GpuBufferSet<PointCloudGrp>,
    image_dimensions: AlignedVec2<u32>,
}

#[derive(Clone)]
pub struct ThalassicPipelineRef {
    shared: Arc<Mutex<(Option<Shared>, bool)>>,
}

impl ThalassicPipelineRef {
    pub fn noop() -> Self {
        Self {
            shared: Arc::new(Mutex::new((None, false))),
        }
    }
}

pub struct ThalassicPipeline {
    pipeline: ComputePipeline<BetaBindGroups, 3>,
    bind_grps: Option<(
        GpuBufferSet<Pcl2ObstacleBindGrp>,
        GpuBufferSet<UnfilteredObstacleMapBindGrp>,
        GpuBufferSet<FilteredObstacleMapBindGrp>,
        GpuBufferSet<ExpanderBindGrp>,
    )>,
    new_radius: Option<f32>,
    cell_size: f32,
    new_max_gradient: Option<f32>,
    thalassic_ref: ThalassicPipelineRef,
    cell_count: NonZeroU32,
}

impl ThalassicPipeline {
    pub fn will_process(&self) -> bool {
        self.thalassic_ref.shared.lock().1
    }

    pub fn process(&mut self, out_expanded_obstacles: &mut [Occupancy]) {
        let mut shared_lock = self.thalassic_ref.shared.lock();

        if !shared_lock.1 {
            return;
        }

        let mut shared = shared_lock.0.take().unwrap();

        let (
            pcl2obstacle_input_grp,
            unfiltered_obstacle_map,
            filtered_obstacle_map,
            expander_input_grp,
        ) = self.bind_grps.take().unwrap();

        let mut bind_grps: BetaBindGroups = (
            shared.points,
            pcl2obstacle_input_grp,
            unfiltered_obstacle_map,
            filtered_obstacle_map,
            expander_input_grp,
        );
        self.pipeline.workgroups[0] = Vector3::new(
            shared.image_dimensions.x / 8,
            shared.image_dimensions.y / 8,
            1,
        );

        self.pipeline
            .new_pass(|mut lock| {
                bind_grps
                    .1
                    .write::<0, _>(&shared.image_dimensions.into(), &mut lock);
                if let Some(new_radius) = self.new_radius.take() {
                    bind_grps
                        .4
                        .write::<1, _>(&((new_radius / self.cell_size).ceil() as u32), &mut lock);
                }
                if let Some(new_max_gradient) = self.new_max_gradient.take() {
                    bind_grps.1.write::<1, _>(&new_max_gradient, &mut lock);
                }
                &mut bind_grps
            })
            .finish();

        let (
            points_grp,
            pcl2obstacle_input_grp,
            unfiltered_obstacle_map,
            filtered_obstacle_map,
            expander_input_grp,
        ) = bind_grps;

        expander_input_grp
            .buffers
            .0
            .read(bytemuck::cast_slice_mut(out_expanded_obstacles));

        self.bind_grps.replace((
            pcl2obstacle_input_grp,
            unfiltered_obstacle_map,
            filtered_obstacle_map,
            expander_input_grp,
        ));
        shared.points = points_grp;
        shared_lock.0.replace(shared);
        shared_lock.1 = false;
    }

    pub fn reset_heightmap(&mut self) {
        // This actually resets the obstacle map
        let bind_grps = self.bind_grps.as_mut().unwrap();
        bind_grps.2 =
            GpuBufferSet::from((StorageBuffer::new_dyn(self.cell_count.get() as usize).unwrap(),));
    }

    pub fn set_radius(&mut self, radius: f32) {
        self.new_radius = Some(radius);
    }

    pub fn get_ref(&self) -> ThalassicPipelineRef {
        self.thalassic_ref.clone()
    }
}
