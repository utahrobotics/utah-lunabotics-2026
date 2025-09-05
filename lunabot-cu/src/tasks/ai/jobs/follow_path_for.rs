use common::Steering;
use simple_motion::StaticNode;

use crate::tasks::ai::jobs::Job;

/// follows path for n meters, fails if the robot fails to move significantly in stuck_timeout_secs
pub fn follow_path_for_n_meters_job(
    stuck_timeout_secs: f32,
    chain: StaticNode,
    n: f32,
) -> Job<Steering> {
    todo!()
}
