use std::{cmp::Ordering, collections::{BinaryHeap, VecDeque}, hash::BuildHasherDefault, time::Duration};

use common::{FromLunabase, Steering, THALASSIC_CELL_SIZE, THALASSIC_HEIGHT, THALASSIC_WIDTH};
use fxhash::FxHasher;
use indexmap::{map::Entry, IndexMap};
use lunabot_ai_common::{FromAI, FromHost, AI_HEARTBEAT_RATE};
use nalgebra::{Vector2, Vector3};
use thalassic::Occupancy;

use crate::context::HostHandle;

use super::SoftStopped;

const COMPLETION_DISTANCE: f64 = 0.2;
const COMPLETION_ANGLE_DEGREES: f64 = 5.0;
const MAX_ARC_ANGLE_DEGREES: f64 = 60.0;
const SAFE_RADIUS: f64 = 0.5;

pub async fn navigate(host_handle: &mut HostHandle, target: Vector2<f64>) -> SoftStopped {
    host_handle.write_to_host(FromAI::SetStage(common::LunabotStage::Autonomy));
    host_handle.write_to_host(FromAI::SetSteering(Steering::default()));

    loop {
        eprintln!("Pausing to scan");
        tokio::select! {
            _ = tokio::time::sleep(Duration::from_millis(2000)) => {}
            _ = async {
                loop {
                    host_handle.write_to_host(FromAI::Heartbeat);
                    tokio::time::sleep(AI_HEARTBEAT_RATE).await;
                }
            } => {}
        }
        
        eprintln!("Scanning");
        host_handle.write_to_host(FromAI::RequestThalassic);

        let mut maybe_obstacle_map = None;
        for _ in 0..20 {
            match host_handle.read_from_host().await {
                FromHost::ThalassicData { obstacle_map } => {
                    maybe_obstacle_map = Some(obstacle_map);
                }
                _ => {}
            }
            host_handle.write_to_host(FromAI::RequestThalassic);
            tokio::time::sleep(Duration::from_millis(80)).await;
        };
        let obstacle_map = if maybe_obstacle_map.is_none() {loop {
            match host_handle.read_from_host().await {
                FromHost::ThalassicData { obstacle_map } => {
                    break obstacle_map;
                }
                _ => {}
            }
        }} else {
            maybe_obstacle_map.unwrap()
        };

        let mut isometry = loop {
            if let FromHost::BaseIsometry { isometry } = host_handle.read_from_host().await {
                break isometry;
            };
        };

        let mut read_fut = async || {
            loop {
                let msg = host_handle.read_from_host().await;
                let FromHost::FromLunabase { msg } = msg else {
                    continue;
                };
                match msg {
                    FromLunabase::SoftStop => break,
                    _ => {}
                }
            }
        };
        let start = Vector2::new((isometry.translation.x / THALASSIC_CELL_SIZE as f64) as usize, (isometry.translation.z / THALASSIC_CELL_SIZE as f64) as usize);
        let end = Vector2::new((target.x / THALASSIC_CELL_SIZE as f64) as usize, (target.y / THALASSIC_CELL_SIZE as f64) as usize);
    
        eprintln!("Obstacle Pathfinding");
        let mut path = if obstacle_map[start.x + start.y * THALASSIC_WIDTH as usize] == Occupancy::OCCUPIED {
            tokio::select! {
                _ = read_fut() => return SoftStopped { called: true },
                (path, _) = astar(
                    start,
                    end,
                    |cell| {
                        obstacle_map[cell.x + cell.y * THALASSIC_WIDTH as usize] != Occupancy::OCCUPIED
                    },
                    |_| true
                ) => path
            }
        } else {
            vec![]
        };

        eprintln!("Safe Pathfinding");
        let safe_path = tokio::select! {
            _ = read_fut() => return SoftStopped { called: true },
            (path, _) = astar(
                path.last().map(|p| Vector2::new((p.x / THALASSIC_CELL_SIZE as f64) as usize, (p.y / THALASSIC_CELL_SIZE as f64) as usize)).unwrap_or(start),
                end,
                |cell| cell == end,
                |cell| {
                    obstacle_map[cell.x + cell.y * THALASSIC_WIDTH as usize] != Occupancy::OCCUPIED
                }
            ) => path
        };
        path.extend_from_slice(&safe_path);

        let mut all_known = true;
        for (i, &p) in path.iter().enumerate() {
            if (Vector2::new(isometry.translation.x, isometry.translation.z) - p).magnitude() <= SAFE_RADIUS {
                continue;
            }
            if obstacle_map[(p.x / THALASSIC_CELL_SIZE as f64) as usize + (p.y / THALASSIC_CELL_SIZE as f64) as usize * THALASSIC_WIDTH as usize] == Occupancy::UNKNOWN {
                all_known = false;
                if i < path.len() - 1 {
                    path.drain((i + 1)..);
                }
                break;
            }
        }

        host_handle.write_to_host(FromAI::PathFound(path.clone()));
        let mut path = VecDeque::from(path);
        eprintln!("Following Path");

        loop {
            isometry = loop {
                match host_handle.read_from_host().await {
                    FromHost::BaseIsometry { isometry } => {
                        break isometry;
                    }
                    FromHost::FromLunabase { msg: FromLunabase::SoftStop } => {
                        host_handle.write_to_host(FromAI::SetSteering(Steering::default()));
                        return SoftStopped { called: true };
                    }
                    _ => {}
                }
            };
            let flat_origin = Vector2::new(isometry.translation.x, isometry.translation.z);
            let (i, mut closest_point, distance) = path.iter().enumerate().map(|(i, p)| {
                (i, p, (*p - flat_origin).magnitude())
            }).min_by(|(_, _, d1), (_, _, d2)| {
                d1.total_cmp(&d2)
            }).unwrap();

            if distance < COMPLETION_DISTANCE {
                if !all_known && i >= path.len() - 2 {
                    eprintln!("Aiming at unknown");
                    let unknown = *path.get(path.len() - 1).unwrap();
                    let known = *path.get(path.len() - 2).unwrap();

                    loop {
                        isometry = loop {
                            match host_handle.read_from_host().await {
                                FromHost::BaseIsometry { isometry } => {
                                    break isometry;
                                }
                                FromHost::FromLunabase { msg: FromLunabase::SoftStop } => {
                                    host_handle.write_to_host(FromAI::SetSteering(Steering::default()));
                                    return SoftStopped { called: true };
                                }
                                _ => {}
                            }
                        };
                        let travel = (unknown - known).normalize();

                        let forward = isometry.rotation * - Vector3::z();
                        let flat_forward = Vector2::new(forward.x, forward.z).normalize();
                        let cross = flat_forward.x * travel.y - flat_forward.y * travel.x;
                        let angle = flat_forward.angle(&travel).clamp(0.0, MAX_ARC_ANGLE_DEGREES.to_radians());

                        if angle < COMPLETION_ANGLE_DEGREES.to_radians() {
                            break;
                        }

                        let speed = angle / MAX_ARC_ANGLE_DEGREES.to_radians();

                        if cross > 0.0 {
                            host_handle.write_to_host(FromAI::SetSteering(Steering::new(speed, -speed, Steering::DEFAULT_WEIGHT)));
                        } else {
                            host_handle.write_to_host(FromAI::SetSteering(Steering::new(-speed, speed, Steering::DEFAULT_WEIGHT)));
                        }
                    }
                    eprintln!("Finished");
                    break;

                }
                path.drain(0..=i);
                if path.is_empty() {
                    eprintln!("Finished");
                    break;
                }
                closest_point = path.front().unwrap();
            }

            let travel = (closest_point - flat_origin).normalize();

            let forward = isometry.rotation * - Vector3::z();
            let flat_forward = Vector2::new(forward.x, forward.z).normalize();
            let cross = flat_forward.x * travel.y - flat_forward.y * travel.x;
            let angle = flat_forward.angle(&travel).clamp(0.0, MAX_ARC_ANGLE_DEGREES.to_radians());
            let angle_norm = angle / MAX_ARC_ANGLE_DEGREES.to_radians();
            let lesser_speed = -1.0 + (1.0 - angle_norm) * 2.0;

            if cross < 0.0 {
                host_handle.write_to_host(FromAI::SetSteering(Steering::new(lesser_speed, 1.0, Steering::DEFAULT_WEIGHT)));
            } else {
                host_handle.write_to_host(FromAI::SetSteering(Steering::new(1.0, lesser_speed, Steering::DEFAULT_WEIGHT)));
            }
        }

        host_handle.write_to_host(FromAI::SetSteering(Steering::default()));
        if all_known {
            break SoftStopped { called: false };
        }
    }
}

struct SmallestCostHolder {
    estimated_cost: usize,
    cost: usize,
    index: usize,
}

impl PartialEq for SmallestCostHolder {
    fn eq(&self, other: &Self) -> bool {
        self.estimated_cost.eq(&other.estimated_cost) && self.cost.eq(&other.cost)
    }
}

impl Eq for SmallestCostHolder {}

impl PartialOrd for SmallestCostHolder {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for SmallestCostHolder {
    fn cmp(&self, other: &Self) -> Ordering {
        match other.estimated_cost.cmp(&self.estimated_cost) {
            Ordering::Equal => self.cost.cmp(&other.cost),
            s => s,
        }
    }
}

fn reverse_path(parents: &IndexMap<Vector2<usize>, (usize, usize), BuildHasherDefault<FxHasher>>, mut parent: impl FnMut(&(usize, usize)) -> usize, start: usize) -> impl Iterator<Item = Vector2<usize>> {
    let mut i = start;
    let path = std::iter::from_fn(|| {
        parents.get_index(i).map(|(node, value)| {
            i = parent(value);
            node
        })
    })
    .collect::<Vec<&Vector2<usize>>>();
    // Collecting the going through the vector is needed to revert the path because the
    // unfold iterator is not double-ended due to its iterative nature.
    path.into_iter().rev().cloned()
}

fn fast_integer_sqrt(n: usize) -> usize {
    let mut x = n;
    let mut y = 1;
    while x > y {
        x = (x + y) / 2;
        y = n / x;
    }
    return x
}

pub async fn astar(
    start: Vector2<usize>,
    end: Vector2<usize>,
    mut success: impl FnMut(Vector2<usize>) -> bool,
    mut is_safe: impl FnMut(Vector2<usize>) -> bool,
) -> (Vec<Vector2<f64>>, f64) {
    let mut to_see = BinaryHeap::new();
    to_see.push(SmallestCostHolder {
        estimated_cost: 0,
        cost: 0,
        index: 0,
    });
    let mut parents = IndexMap::<Vector2<usize>, (usize, usize), BuildHasherDefault<FxHasher>>::default();
    parents.insert(start.clone(), (usize::MAX, 0));
    let mut closest_path = None;
    let mut closest_cost = 0usize;
    let heuristic = |node: Vector2<usize>| {
        fast_integer_sqrt(node.x.abs_diff(end.x) + node.y.abs_diff(end.y))
    };

    while let Some(SmallestCostHolder { cost, index, .. }) = to_see.pop() {
        let successors = {
            let (node, &(_, c)) = parents.get_index(index).unwrap(); // Cannot fail
            let h = heuristic(*node);
            if success(*node) {
                let path = reverse_path(&parents, |&(p, _)| p, index).map(|p| Vector2::new(p.x as f64 * THALASSIC_CELL_SIZE as f64, p.y as f64 * THALASSIC_CELL_SIZE as f64));
                return (path.collect(), cost as f64 / 10.0);
            } else if closest_path.is_none() || closest_cost > h + c {
                closest_cost = h + c;
                closest_path = Some(reverse_path(&parents, |&(p, _)| p, index).map(|p| Vector2::new(p.x as f64 * THALASSIC_CELL_SIZE as f64, p.y as f64 * THALASSIC_CELL_SIZE as f64)).collect());
            }
            // We may have inserted a node several time into the binary heap if we found
            // a better way to access it. Ensure that we are currently dealing with the
            // best path and discard the others.
            if cost > c {
                continue;
            }
            [
                (Vector2::new(1isize, 0), 10),
                (Vector2::new(0, 1), 10),
                (Vector2::new(-1, 0), 10),
                (Vector2::new(0, -1), 10),
                (Vector2::new(1, 1), 14),
                (Vector2::new(-1, 1), 14),
                (Vector2::new(-1, -1), 14),
                (Vector2::new(1, -1), 14),
            ].into_iter()
            .filter_map(|(offset, cost)| {
                let new_x = node.x.checked_add_signed(offset.x)?;
                let new_y = node.y.checked_add_signed(offset.y)?;

                if new_x >= THALASSIC_WIDTH as usize || new_y >= THALASSIC_HEIGHT as usize {
                    None
                } else {
                    Some((Vector2::new(new_x, new_y), cost))
                }

            })
            .filter(|(p, _)| is_safe(*p))
            .collect::<Vec<_>>()
        };
        for (successor, move_cost) in successors {
            let new_cost = cost + move_cost;
            let h; // heuristic(&successor)
            let n; // index for successor
            match parents.entry(successor) {
                Entry::Vacant(e) => {
                    h = heuristic(*e.key());
                    n = e.index();
                    e.insert((index, new_cost));
                }
                Entry::Occupied(mut e) => {
                    if e.get().1 > new_cost {
                        h = heuristic(*e.key());
                        n = e.index();
                        e.insert((index, new_cost));
                    } else {
                        continue;
                    }
                }
            }

            to_see.push(SmallestCostHolder {
                estimated_cost: new_cost + h,
                cost: new_cost,
                index: n,
            });
        }
        tokio::task::yield_now().await;
    }
    (closest_path.unwrap(), closest_cost as f64 / 10.0)
}