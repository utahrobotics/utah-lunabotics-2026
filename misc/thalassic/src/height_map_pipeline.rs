use std::num::NonZeroU32;

use gputter::{
    buffers::{
        storage::{HostReadOnly, HostReadWrite, ShaderReadWrite, StorageBuffer},
        uniform::UniformBuffer,
        GpuBuffer, GpuBufferSet,
    },
    compute::ComputePipeline,
    get_device,
    shader::BufferGroupBinding,
    types::{AlignedVec2, AlignedVec4},
};
use nalgebra::{Vector2, Vector3};

use crate::{
    clear_heights::ClearHeights, gaussian_blur::GaussianBlur, pcl2height::Pcl2HeightV3,
    PipelineSharedItems,
};

type HeightMapPipelineBindGroups = (
    GpuBufferSet<(StorageBuffer<[AlignedVec4<f32>], HostReadOnly, ShaderReadWrite>,)>, // Index 0 - pointcloud input grp
    GpuBufferSet<(StorageBuffer<[u32], HostReadWrite, ShaderReadWrite>,)>, // Index 1 - output height map
    GpuBufferSet<(UniformBuffer<u32>,)>, // Index 2 - number of points in the point cloud
    GpuBufferSet<(UniformBuffer<AlignedVec2<u32>>,)>, // Index 3 - depth image dimensions
    GpuBufferSet<(StorageBuffer<[u32], HostReadOnly, ShaderReadWrite>,)>, // Index 4 - gaussian blurred output height map
    GpuBufferSet<(UniformBuffer<u32>,)>, // Index 5 - request to clear cells
    GpuBufferSet<(StorageBuffer<[u32], HostReadOnly, ShaderReadWrite>,)>, // Index 6 - Keeps track of known cells (1 = known, 0 = unknown)
);

pub struct HeightMapPipelineBuilder {
    pub grid_dimentions: Vector2<NonZeroU32>,
    pub cell_size: f32,
    pub max_point_count: NonZeroU32,

    /// kernel size for the gaussian blur
    pub kernel_size: NonZeroU32,
}

/// Takes in point clouds and sets cell's values to the height of the highest point with an x,y that are within that cells boundaries.
/// Each cell remembers the highest point across multiple point clouds (meaning the height value of a cell can only increase) unless
/// request_clear_cells() is called true in which case when the next point cloud comes in, affected cells are completely re calculated, forgetting
/// previous max height values.
pub struct HeightMapPipeline {
    pub bind_grps: Option<(
        GpuBufferSet<(StorageBuffer<[u32], HostReadWrite, ShaderReadWrite>,)>, // unblurred height map
        GpuBufferSet<(UniformBuffer<u32>,)>,                                   // point count
        GpuBufferSet<(StorageBuffer<[u32], HostReadOnly, ShaderReadWrite>,)>,  // blurred height map
        GpuBufferSet<(StorageBuffer<[u32], HostReadOnly, ShaderReadWrite>,)>, // known cells (1 = known 0 = unknown)
    )>,
    pub pipeline: ComputePipeline<HeightMapPipelineBindGroups, 3>,
    pub grid_dimensions: Vector2<NonZeroU32>,
    /// Items shared between the depth projector and this pipeline
    pub pipeline_shared_ref: PipelineSharedItems,
    pub cell_size: f32,
    request_clear_cells: bool,
}

impl HeightMapPipelineBuilder {
    pub fn build(self, pipeline_shared_ref: PipelineSharedItems) -> HeightMapPipeline {
        let cell_count = self.grid_dimentions.x.get() * self.grid_dimentions.y.get();
        let cell_count = NonZeroU32::new(cell_count).unwrap();

        let [clear_heightmap] = ClearHeights {
            height_map: BufferGroupBinding::<_, HeightMapPipelineBindGroups>::get::<1, 0>(),
            points: BufferGroupBinding::<_, HeightMapPipelineBindGroups>::get::<0, 0>(),
            image_dimensions: BufferGroupBinding::<_, HeightMapPipelineBindGroups>::get::<3, 0>(),
            heightmap_width: self.grid_dimentions.x,
            cell_count,
            cell_size: self.cell_size,
            request_clear_cells: BufferGroupBinding::<_, HeightMapPipelineBindGroups>::get::<5, 0>(
            ),
            known_cells: BufferGroupBinding::<_, HeightMapPipelineBindGroups>::get::<6, 0>(),
        }
        .compile();

        let [pcl2_height_shader] = Pcl2HeightV3 {
            heightmap: BufferGroupBinding::<_, HeightMapPipelineBindGroups>::get::<1, 0>(),
            points: BufferGroupBinding::<_, HeightMapPipelineBindGroups>::get::<0, 0>(),
            heightmap_width: self.grid_dimentions.x,
            heightmap_height: self.grid_dimentions.y,
            cell_size: self.cell_size,
            cell_count,
            image_dimensions: BufferGroupBinding::<_, HeightMapPipelineBindGroups>::get::<3, 0>(),
        }
        .compile();

        let [gaussian_blur] = GaussianBlur {
            heightmap: BufferGroupBinding::<_, HeightMapPipelineBindGroups>::get::<1, 0>(),
            blurred_heightmap: BufferGroupBinding::<_, HeightMapPipelineBindGroups>::get::<4, 0>(),
            heightmap_width: self.grid_dimentions.x,
            heightmap_height: self.grid_dimentions.y,
            cell_size: self.cell_size,
            cell_count,
            kernel_size: self.kernel_size.get(),
            known_cells: BufferGroupBinding::<_, HeightMapPipelineBindGroups>::get::<6, 0>(),
        }
        .compile();
        let initial_height_map = StorageBuffer::new_dyn(cell_count.get() as usize).unwrap();
        let negative_values = vec![-9999.0f32; cell_count.get() as usize];
        get_device().queue.write_buffer(
            initial_height_map.get_buffer(),
            0,
            bytemuck::cast_slice(&negative_values),
        );

        let pipeline =
            ComputePipeline::new([&clear_heightmap, &pcl2_height_shader, &gaussian_blur]);
        let bind_grps = Some((
            GpuBufferSet::from((initial_height_map,)),
            GpuBufferSet::from((UniformBuffer::new(),)),
            GpuBufferSet::from((StorageBuffer::new_dyn(cell_count.get() as usize).unwrap(),)),
            GpuBufferSet::from((StorageBuffer::new_dyn(cell_count.get() as usize).unwrap(),)),
        ));

        HeightMapPipeline {
            bind_grps,
            pipeline,
            grid_dimensions: self.grid_dimentions,
            pipeline_shared_ref,
            cell_size: self.cell_size,
            request_clear_cells: false,
        }
    }
}

impl HeightMapPipeline {
    /// Check if the pipeline is ready to process (i.e., DepthProjector has finished)
    pub fn will_process(&self) -> bool {
        self.pipeline_shared_ref.shared.lock().1
    }

    /// Cells affected by the next point cloud will be re calculated, forgetting their previous maximum height
    pub fn request_clear_cells(&mut self) {
        // this gets set back to false in the process function
        self.request_clear_cells = true;
    }

    /// Process the height map using shared point cloud data from DepthProjector
    ///
    /// This method is designed to be used in a pipeline chain:
    /// DepthProjector::project() -> HeightMapPipeline::process()
    ///
    /// The point cloud data is automatically shared via PipelineSharedItems.
    pub fn process(&mut self, point_count: u32, height_map_out: &mut [u32]) {
        let mut shared_lock = self.pipeline_shared_ref.shared.lock();

        if !shared_lock.1 {
            return;
        }

        let mut shared = shared_lock.0.take().unwrap();
        let (height_map_grp, point_count_grp, blurred_grid, known_cells) =
            self.bind_grps.take().unwrap();

        let mut bind_grps: HeightMapPipelineBindGroups = (
            shared.points,
            height_map_grp,
            point_count_grp,
            GpuBufferSet::from((UniformBuffer::new(),)),
            blurred_grid,
            GpuBufferSet::from((UniformBuffer::new(),)), // write the request_clear_cells buffer on new pass
            known_cells,
        );

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
        self.pipeline.workgroups[2] = Vector3::new(
            self.grid_dimensions.x.get().div_ceil(8),
            self.grid_dimensions.y.get().div_ceil(8),
            1,
        );

        self.pipeline
            .new_pass(|mut lock| {
                // writes the depth image dimentions and point count
                bind_grps.2.write::<0, _>(&point_count, &mut lock);
                bind_grps
                    .3
                    .write::<0, _>(&shared.image_dimensions, &mut lock);
                bind_grps
                    .5
                    .write::<0, _>(&self.request_clear_cells.into(), &mut lock);
                &mut bind_grps
            })
            .finish();

        let (points_grp, height_map_grp, point_count_grp, _, blurred_grid, _, known_cells) =
            bind_grps;

        // reads out the resulting height map
        blurred_grid
            .buffers
            .0
            .read(bytemuck::cast_slice_mut(height_map_out));

        self.bind_grps = Some((height_map_grp, point_count_grp, blurred_grid, known_cells));
        shared.points = points_grp;
        shared_lock.0.replace(shared);
        shared_lock.1 = false;
        self.request_clear_cells = false;
    }
}
