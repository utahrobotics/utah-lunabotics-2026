// we arent actually using the thalassic pipeline, just the depth projector.
// points from the depth projector are sent to the occupancy grid pipeline

use thalassic::ThalassicPipelineRef;

#[allow(dead_code)]
struct ThalassicData;

impl Default for ThalassicData {
    fn default() -> Self {
        Self
    }
}

pub fn spawn_minimal_thalassic_pipeline() -> ThalassicPipelineRef {
    ThalassicPipelineRef::noop()
}
