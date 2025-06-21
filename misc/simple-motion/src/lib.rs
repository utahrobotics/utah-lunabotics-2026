use std::{collections::VecDeque, ops::Deref, sync::Arc};

use crossbeam::atomic::AtomicCell;
use nalgebra::{Isometry3, Point3, UnitQuaternion, UnitVector3, Vector3};
use serde::Deserialize;

pub enum TranslationRestriction {
    Fixed {
        origin: Point3<f64>,
    },
    Linear {
        start_origin: Point3<f64>,
        axis: UnitVector3<f64>,
        min_length: Option<f64>,
        max_length: Option<f64>,
        current_length: Option<f64>,
    },
    Free {
        origin: Point3<f64>,
    },
}

pub enum RotationRestriction {
    Fixed {
        rotation: UnitQuaternion<f64>,
    },
    OneAxis {
        start_rotation: UnitQuaternion<f64>,
        axis: UnitVector3<f64>,
        min_angle: Option<f64>,
        max_angle: Option<f64>,
        current_angle: Option<f64>,
    },
    Free {
        rotation: UnitQuaternion<f64>,
    },
}

#[derive(Clone, Copy)]
struct LinearDynamicState {
    current_origin: Point3<f64>,
    current_length: f64,
}

enum TranslationRestrictionState {
    Fixed {
        origin: Point3<f64>,
    },
    Linear {
        start_origin: Point3<f64>,
        axis: UnitVector3<f64>,
        min_length: Option<f64>,
        max_length: Option<f64>,
        dynamic: AtomicCell<LinearDynamicState>,
    },
    Free {
        origin: AtomicCell<Point3<f64>>,
    },
}

impl From<TranslationRestriction> for TranslationRestrictionState {
    fn from(translation: TranslationRestriction) -> Self {
        match translation {
            TranslationRestriction::Fixed { origin } => {
                TranslationRestrictionState::Fixed { origin }
            }
            TranslationRestriction::Linear {
                start_origin,
                axis,
                min_length,
                max_length,
                current_length,
            } => TranslationRestrictionState::Linear {
                start_origin,
                axis,
                min_length,
                max_length,
                dynamic: AtomicCell::new(if let Some(current_length) = current_length {
                    LinearDynamicState {
                        current_origin: start_origin + axis.into_inner() * current_length,
                        current_length,
                    }
                } else if let Some(min_length) = min_length {
                    LinearDynamicState {
                        current_origin: start_origin + axis.into_inner() * min_length,
                        current_length: min_length,
                    }
                } else {
                    LinearDynamicState {
                        current_origin: start_origin,
                        current_length: 0.0,
                    }
                }),
            },
            TranslationRestriction::Free { origin } => TranslationRestrictionState::Free {
                origin: AtomicCell::new(origin),
            },
        }
    }
}

#[derive(Clone, Copy)]
struct OneAxisDynamicState {
    current_rotation: UnitQuaternion<f64>,
    current_angle: f64,
}

enum RotationRestrictionState {
    Fixed {
        rotation: UnitQuaternion<f64>,
    },
    OneAxis {
        start_rotation: UnitQuaternion<f64>,
        axis: UnitVector3<f64>,
        min_angle: Option<f64>,
        max_angle: Option<f64>,
        dynamic: AtomicCell<OneAxisDynamicState>,
    },
    Free {
        rotation: AtomicCell<UnitQuaternion<f64>>,
    },
}

impl From<RotationRestriction> for RotationRestrictionState {
    fn from(rotation: RotationRestriction) -> Self {
        match rotation {
            RotationRestriction::Fixed { rotation } => RotationRestrictionState::Fixed { rotation },
            RotationRestriction::OneAxis {
                start_rotation,
                axis,
                min_angle,
                max_angle,
                current_angle,
            } => RotationRestrictionState::OneAxis {
                start_rotation,
                axis,
                min_angle,
                max_angle,
                dynamic: AtomicCell::new(if let Some(current_angle) = current_angle {
                    OneAxisDynamicState {
                        current_rotation: UnitQuaternion::from_axis_angle(&axis, current_angle)
                            * start_rotation,
                        current_angle,
                    }
                } else if let Some(min_angle) = min_angle {
                    OneAxisDynamicState {
                        current_rotation: UnitQuaternion::from_axis_angle(&axis, min_angle)
                            * start_rotation,
                        current_angle: min_angle,
                    }
                } else {
                    OneAxisDynamicState {
                        current_rotation: start_rotation,
                        current_angle: 0.0,
                    }
                }),
            },
            RotationRestriction::Free { rotation } => RotationRestrictionState::Free {
                rotation: AtomicCell::new(rotation),
            },
        }
    }
}

pub struct ImmutableTransformable {
    translation_restriction: TranslationRestrictionState,
    rotation_restriction: RotationRestrictionState,
}

#[repr(transparent)]
pub struct Transformable(ImmutableTransformable);

impl ImmutableTransformable {
    pub fn get_local_origin(&self) -> Point3<f64> {
        match &self.translation_restriction {
            TranslationRestrictionState::Fixed { origin } => *origin,
            TranslationRestrictionState::Linear { dynamic, .. } => dynamic.load().current_origin,
            TranslationRestrictionState::Free { origin } => origin.load(),
        }
    }

    pub fn get_local_rotation(&self) -> UnitQuaternion<f64> {
        match &self.rotation_restriction {
            RotationRestrictionState::Fixed { rotation } => *rotation,
            RotationRestrictionState::OneAxis { dynamic, .. } => dynamic.load().current_rotation,
            RotationRestrictionState::Free { rotation } => rotation.load(),
        }
    }

    pub fn get_local_isometry(&self) -> Isometry3<f64> {
        Isometry3::from_parts(self.get_local_origin().into(), self.get_local_rotation())
    }

    pub fn get_local_length(&self) -> Option<f64> {
        match &self.translation_restriction {
            TranslationRestrictionState::Linear { dynamic, .. } => {
                Some(dynamic.load().current_length)
            }
            _ => None,
        }
    }

    pub fn get_local_angle_one_axis(&self) -> Option<f64> {
        match &self.rotation_restriction {
            RotationRestrictionState::OneAxis { dynamic, .. } => Some(dynamic.load().current_angle),
            _ => None,
        }
    }

    pub fn is_origin_fixed(&self) -> bool {
        matches!(
            &self.translation_restriction,
            TranslationRestrictionState::Fixed { .. }
        )
    }

    pub fn is_rotation_fixed(&self) -> bool {
        matches!(
            &self.rotation_restriction,
            RotationRestrictionState::Fixed { .. }
        )
    }

    pub fn is_origin_linear(&self) -> bool {
        matches!(
            &self.translation_restriction,
            TranslationRestrictionState::Linear { .. }
        )
    }

    pub fn is_rotation_one_axis(&self) -> bool {
        matches!(
            &self.rotation_restriction,
            RotationRestrictionState::OneAxis { .. }
        )
    }

    pub fn is_origin_free(&self) -> bool {
        matches!(
            &self.translation_restriction,
            TranslationRestrictionState::Free { .. }
        )
    }

    pub fn is_rotation_free(&self) -> bool {
        matches!(
            &self.rotation_restriction,
            RotationRestrictionState::Free { .. }
        )
    }
}

impl Transformable {
    pub fn try_set_origin(&self, new_origin: Point3<f64>) -> bool {
        match &self.0.translation_restriction {
            TranslationRestrictionState::Free { origin } => {
                origin.store(new_origin);
                true
            }
            _ => false,
        }
    }

    pub fn set_origin(&self, new_origin: Point3<f64>) {
        if !self.try_set_origin(new_origin) {
            eprintln!("Cannot set origin for a non-free translation restriction");
        }
    }

    pub fn try_set_length(&self, mut new_length: f64) -> bool {
        match &self.0.translation_restriction {
            TranslationRestrictionState::Linear {
                dynamic,
                start_origin,
                axis,
                min_length,
                max_length,
            } => {
                if let Some(min_length) = min_length {
                    new_length = new_length.max(*min_length);
                }
                if let Some(max_length) = max_length {
                    new_length = new_length.min(*max_length);
                }
                let new = LinearDynamicState {
                    current_origin: start_origin + axis.into_inner() * new_length,
                    current_length: new_length,
                };
                dynamic.store(new);
                true
            }
            _ => false,
        }
    }

    pub fn set_length(&self, new_length: f64) {
        if !self.try_set_length(new_length) {
            eprintln!("Cannot set length for a non-linear translation restriction");
        }
    }

    pub fn try_set_rotation(&self, new_rotation: UnitQuaternion<f64>) -> bool {
        match &self.0.rotation_restriction {
            RotationRestrictionState::Free { rotation } => {
                rotation.store(new_rotation);
                true
            }
            _ => false,
        }
    }

    pub fn set_rotation(&self, new_rotation: UnitQuaternion<f64>) {
        if !self.try_set_rotation(new_rotation) {
            eprintln!("Cannot set rotation for a non-free rotation restriction");
        }
    }

    pub fn try_set_angle_one_axis(&self, mut new_angle: f64) -> bool {
        match &self.0.rotation_restriction {
            RotationRestrictionState::OneAxis {
                dynamic,
                start_rotation,
                axis,
                min_angle,
                max_angle,
            } => {
                if let Some(min_angle) = min_angle {
                    new_angle = new_angle.max(*min_angle);
                }
                if let Some(max_angle) = max_angle {
                    new_angle = new_angle.min(*max_angle);
                }
                let new = OneAxisDynamicState {
                    current_rotation: 
                        UnitQuaternion::from_axis_angle(&axis, new_angle) * start_rotation,
                    current_angle: new_angle,
                };
                dynamic.store(new);
                true
            }
            _ => false,
        }
    }

    pub fn set_angle_one_axis(&self, new_angle: f64) {
        if !self.try_set_angle_one_axis(new_angle) {
            eprintln!("Cannot set angle for a non-one-axis rotation restriction");
        }
    }

    pub fn try_set_isometry(&self, new_isometry: Isometry3<f64>) -> bool {
        self.try_set_origin(new_isometry.translation.vector.into())
            && self.try_set_rotation(new_isometry.rotation)
    }

    pub fn set_isometry(&self, new_isometry: Isometry3<f64>) {
        if !self.try_set_isometry(new_isometry) {
            eprintln!("Cannot set isometry for a non-free transformable");
        }
    }
}

impl Deref for Transformable {
    type Target = ImmutableTransformable;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

pub struct NodeData {
    transformable: ImmutableTransformable,
    parent: Option<usize>,
    name: Option<Box<str>>,
}

#[derive(Clone, Copy)]
pub struct ImmutableNode<S = Arc<[NodeData]>> {
    arena: S,
    index: usize,
}

impl<S: Deref<Target = [NodeData]>> Deref for ImmutableNode<S> {
    type Target = ImmutableTransformable;

    fn deref(&self) -> &Self::Target {
        &self.arena[self.index].transformable
    }
}

impl<S> From<Node<S>> for ImmutableNode<S> {
    fn from(node: Node<S>) -> Self {
        node.0
    }
}

impl<S: Deref<Target = [NodeData]> + Clone> ImmutableNode<S> {
    pub fn get_parent(&self) -> Option<Self> {
        self.arena[self.index]
            .parent
            .map(|parent_index| ImmutableNode {
                arena: self.arena.clone(),
                index: parent_index,
            })
    }

    pub fn get_root(&self) -> Self {
        ImmutableNode {
            arena: self.arena.clone(),
            index: 0,
        }
    }

    pub fn get_node_with_name(&self, name: &str) -> Option<Self> {
        self.arena.iter().enumerate().find_map(|(index, node)| {
            if node.name.as_deref() == Some(name) {
                Some(ImmutableNode {
                    arena: self.arena.clone(),
                    index,
                })
            } else {
                None
            } // let mut pathfinder = DefaultPathfinder::new(world_to_grid, grid_to_world);
        })
    }

    pub fn get_global_isometry(&self) -> Isometry3<f64> {
        let Some(parent) = self.get_parent() else {
            return self.arena[self.index].transformable.get_local_isometry();
        };
        parent.get_global_isometry() * self.arena[self.index].transformable.get_local_isometry()
    }

    pub fn get_isometry_from_base(&self) -> Isometry3<f64> {
        let Some(parent) = self.get_parent() else {
            return Isometry3::identity();
        };
        parent.get_isometry_from_base() * self.arena[self.index].transformable.get_local_isometry()
    }
}

#[derive(Clone, Copy)]
#[repr(transparent)]
pub struct Node<S = Arc<[NodeData]>>(ImmutableNode<S>);

impl<S: Deref<Target = [NodeData]>> Deref for Node<S> {
    type Target = Transformable;

    fn deref(&self) -> &Self::Target {
        let immut: &ImmutableTransformable = &self.0.arena[self.0.index].transformable;
        unsafe { std::mem::transmute(immut) }
    }
}

impl<S: Deref<Target = [NodeData]> + Clone> Node<S> {
    pub fn get_parent(&self) -> Option<Self> {
        self.0.get_parent().map(Node)
    }

    pub fn get_root(&self) -> Self {
        Node(self.0.get_root())
    }

    pub fn get_node_with_name(&self, name: &str) -> Option<Self> {
        self.0.get_node_with_name(name).map(Node)
    }

    pub fn get_global_isometry(&self) -> Isometry3<f64> {
        self.0.get_global_isometry()
    }
    pub fn get_isometry_from_base(&self) -> Isometry3<f64> {
        self.0.get_isometry_from_base()
    }
}

pub type StaticImmutableNode = ImmutableNode<&'static [NodeData]>;
pub type StaticNode = Node<&'static [NodeData]>;

pub struct ChainBuilder {
    nodes: Vec<NodeData>,
}

impl ChainBuilder {
    pub fn new_fixed() -> Self {
        Self::new(
            TranslationRestriction::Fixed {
                origin: Point3::origin(),
            },
            RotationRestriction::Fixed {
                rotation: UnitQuaternion::identity(),
            },
        )
    }

    pub fn new_free() -> Self {
        Self::new(
            TranslationRestriction::Free {
                origin: Point3::origin(),
            },
            RotationRestriction::Free {
                rotation: UnitQuaternion::identity(),
            },
        )
    }

    pub fn new(translation: TranslationRestriction, rotation: RotationRestriction) -> Self {
        Self {
            nodes: vec![NodeData {
                transformable: ImmutableTransformable {
                    translation_restriction: translation.into(),
                    rotation_restriction: rotation.into(),
                },
                parent: None,
                name: None,
            }],
        }
    }

    pub fn add_node(
        &mut self,
        parent: usize,
        translation: TranslationRestriction,
        rotation: RotationRestriction,
    ) -> usize {
        let new_index = self.nodes.len();
        self.nodes.push(NodeData {
            transformable: ImmutableTransformable {
                translation_restriction: translation.into(),
                rotation_restriction: rotation.into(),
            },
            parent: Some(parent),
            name: None,
        });
        new_index
    }

    pub fn set_node_name(&mut self, node: usize, name: impl Into<Box<str>>) {
        self.nodes[node].name = Some(name.into());
    }

    pub fn finish_with<S>(self, f: impl FnOnce(Vec<NodeData>) -> S) -> Node<S> {
        Node(ImmutableNode {
            arena: f(self.nodes),
            index: 0,
        })
    }

    pub fn finish(self) -> Node {
        self.finish_with(|nodes| Arc::from(nodes.into_boxed_slice()))
    }

    pub fn finish_static(self) -> StaticNode {
        self.finish_with(|nodes: Vec<NodeData>| {
            let out: &_ = Box::leak(nodes.into_boxed_slice());
            out
        })
    }
}

#[derive(Deserialize)]
#[serde(untagged)]
enum TranslationRestrictionSerde {
    Free {
        #[serde(skip_serializing_if = "all_zeros")]
        free_origin: [f64; 3],
    },
    Linear {
        #[serde(default)]
        #[serde(skip_serializing_if = "all_zeros")]
        start_origin: [f64; 3],
        linear_translation_axis: [f64; 3],
        #[serde(skip_serializing_if = "Option::is_none")]
        min_length: Option<f64>,
        #[serde(skip_serializing_if = "Option::is_none")]
        max_length: Option<f64>,
        #[serde(skip_serializing_if = "Option::is_none")]
        current_length: Option<f64>,
    },
    Fixed {
        #[serde(default)]
        #[serde(skip_serializing_if = "all_zeros")]
        origin: [f64; 3],
    },
}

impl Default for TranslationRestrictionSerde {
    fn default() -> Self {
        TranslationRestrictionSerde::Fixed {
            origin: [0.0, 0.0, 0.0],
        }
    }
}

impl From<TranslationRestrictionSerde> for TranslationRestriction {
    fn from(translation: TranslationRestrictionSerde) -> Self {
        match translation {
            TranslationRestrictionSerde::Fixed { origin } => TranslationRestriction::Fixed {
                origin: Point3::from(origin),
            },
            TranslationRestrictionSerde::Linear {
                start_origin,
                linear_translation_axis: axis,
                min_length,
                max_length,
                current_length,
            } => TranslationRestriction::Linear {
                start_origin: Point3::from(start_origin),
                axis: UnitVector3::try_new(Vector3::from(axis), 0.1).expect("Axis is too short"),
                min_length,
                max_length,
                current_length,
            },
            TranslationRestrictionSerde::Free { free_origin } => TranslationRestriction::Free {
                origin: Point3::from(free_origin),
            },
        }
    }
}

#[derive(Deserialize)]
#[serde(untagged)]
enum RotationRestrictionSerde {
    Free {
        #[serde(skip_serializing_if = "all_zeros")]
        free_euler: [f64; 3],
    },
    OneAxis {
        #[serde(default)]
        #[serde(skip_serializing_if = "all_zeros")]
        start_euler: [f64; 3],
        rotation_axis: [f64; 3],
        min_angle: Option<f64>,
        max_angle: Option<f64>,
        current_angle: Option<f64>,
    },
    Fixed {
        #[serde(default)]
        #[serde(skip_serializing_if = "all_zeros")]
        euler: [f64; 3],
    },
}

impl Default for RotationRestrictionSerde {
    fn default() -> Self {
        RotationRestrictionSerde::Fixed {
            euler: [0.0, 0.0, 0.0],
        }
    }
}

fn from_euler_angles(roll: f64, pitch: f64, yaw: f64) -> UnitQuaternion<f64> {
    let mut current = UnitQuaternion::from_scaled_axis(Vector3::y() * yaw);
    let pitch_axis = current * Vector3::x();
    current = UnitQuaternion::from_scaled_axis(pitch_axis * pitch) * current;
    let roll_axis = current * Vector3::z();
    UnitQuaternion::from_scaled_axis(roll_axis * roll) * current
}

impl From<RotationRestrictionSerde> for RotationRestriction {
    fn from(rotation: RotationRestrictionSerde) -> Self {
        match rotation {
            RotationRestrictionSerde::Fixed { euler } => RotationRestriction::Fixed {
                rotation: from_euler_angles(
                    euler[0].to_radians(),
                    euler[1].to_radians(),
                    euler[2].to_radians(),
                ),
            },
            RotationRestrictionSerde::OneAxis {
                start_euler,
                rotation_axis: axis,
                min_angle,
                max_angle,
                current_angle,
            } => RotationRestriction::OneAxis {
                start_rotation: from_euler_angles(
                    start_euler[0].to_radians(),
                    start_euler[1].to_radians(),
                    start_euler[2].to_radians(),
                ),
                axis: UnitVector3::try_new(Vector3::from(axis), 0.1).expect("Axis is too short"),
                min_angle,
                max_angle,
                current_angle,
            },
            RotationRestrictionSerde::Free { free_euler } => RotationRestriction::Free {
                rotation: from_euler_angles(
                    free_euler[0].to_radians(),
                    free_euler[1].to_radians(),
                    free_euler[2].to_radians(),
                ),
            },
        }
    }
}

#[allow(unused)]
fn all_zeros(v: &[f64; 3]) -> bool {
    v.iter().all(|&x| x == 0.0)
}

#[derive(Deserialize)]
pub struct NodeSerde {
    #[serde(skip_serializing_if = "Option::is_none")]
    name: Option<String>,
    #[serde(default)]
    #[serde(skip_serializing_if = "Vec::is_empty")]
    children: Vec<NodeSerde>,
    #[serde(default, flatten)]
    fields: ron::Value,
}

impl From<NodeSerde> for ChainBuilder {
    fn from(
        NodeSerde {
            name,
            fields,
            children,
        }: NodeSerde,
    ) -> Self {
        let translation: TranslationRestrictionSerde =
            ron::de::from_str(&ron::ser::to_string(&fields).unwrap()).unwrap();
        let rotation: RotationRestrictionSerde =
            ron::de::from_str(&ron::ser::to_string(&fields).unwrap()).unwrap();

        let mut builder = ChainBuilder::new(translation.into(), rotation.into());
        if let Some(name) = name {
            builder.set_node_name(0, name);
        }

        let mut queue: VecDeque<_> = children.into_iter().zip(std::iter::repeat(0)).collect();

        while let Some((node_serde, parent_idx)) = queue.pop_front() {
            let translation: TranslationRestrictionSerde =
                ron::de::from_str(&ron::ser::to_string(&node_serde.fields).unwrap())
                    .unwrap();
            let rotation: RotationRestrictionSerde =
                ron::de::from_str(&ron::ser::to_string(&node_serde.fields).unwrap()).unwrap();

            let index = builder.add_node(parent_idx, translation.into(), rotation.into());
            if let Some(name) = node_serde.name {
                builder.set_node_name(index, name);
            }
            queue.extend(
                node_serde
                    .children
                    .into_iter()
                    .zip(std::iter::repeat(index)),
            );
        }

        builder
    }
}

impl NodeSerde {
    pub fn from_str(s: &str) -> Result<Self, ron::error::SpannedError> {
        ron::de::from_str(s)
    }

    pub fn from_reader<R: std::io::Read>(mut r: R) -> Result<Self, Box<dyn std::error::Error>> {
        let mut string = String::new();
        r.read_to_string(&mut string)?;
        ron::de::from_str(&string).map_err(|e| e.into())
    }
}

#[cfg(test)]
mod tests {
    use crate::{ChainBuilder, NodeSerde};

    #[test]
    fn simple_deserialize_ron01() {
        let ron_str = r#"{}"#;
        let node_serde: NodeSerde = ron::de::from_str(ron_str).unwrap();
        let builder = ChainBuilder::from(node_serde);
        let node = builder.finish();
        assert!(node.is_origin_fixed());
        assert!(node.is_rotation_fixed());

        assert_eq!(node.get_local_origin(), nalgebra::Point3::origin());
        assert_eq!(
            node.get_local_rotation(),
            nalgebra::UnitQuaternion::identity()
        );
    }

    #[test]
    fn simple_deserialize_ron02() {
        let ron_str = r#"{
            origin: [0.0, 1.0, 0.0]
        }"#;
        let node_serde: NodeSerde = ron::de::from_str(ron_str).unwrap();
        let builder = ChainBuilder::from(node_serde);
        let node = builder.finish();
        assert!(node.is_origin_fixed());
        assert!(node.is_rotation_fixed());

        assert_eq!(
            node.get_local_origin(),
            nalgebra::Point3::new(0.0, 1.0, 0.0)
        );
        assert_eq!(
            node.get_local_rotation(),
            nalgebra::UnitQuaternion::identity()
        );
    }
}
