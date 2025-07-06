#![allow(unused)]

use std::ops::{Add, Mul, Sub};

use crossbeam::atomic::AtomicCell;
use nalgebra::{
    Quaternion, RealField, SimdRealField, UnitQuaternion, UnitVector3, Vector2, Vector3,
};
use spin_sleep::SpinSleeper;

use common::Steering;

/// named as such to avoid confusion with `nalgebra::distance` and `pathfinding::distance`
pub fn distance_between_tuples((x1, y1): (usize, usize), (x2, y2): (usize, usize)) -> f32 {
    Vector2::new(x1.abs_diff(x2) as f32, y1.abs_diff(y2) as f32).magnitude()
}

pub fn lerp_value(delta: f64, weight: f64) -> f64 {
    0.5f64.powf(weight * delta)
}

#[allow(dead_code)]
pub fn lerp<T>(from: T, to: T, delta: f64, weight: f64) -> T
where
    T: Sub<Output = T> + Add<Output = T> + Mul<f64, Output = T> + Copy,
{
    let diff = to - from;
    from + diff * lerp_value(delta, weight)
}

/// Decomposes the `src` quaternion into two quaternions: the `twist` quaternion is the rotation around the `axis` vector, and the `swing` quaternion is the remaining rotation.
///
/// The returned order is `(swing, twist)`. The original quaternion can be reconstructed by `swing * twist`.
///
/// # Source
/// 1. https://stackoverflow.com/questions/3684269/component-of-a-quaternion-rotation-around-an-axis
/// 2. https://www.euclideanspace.com/maths/geometry/rotations/for/decomposition/
#[inline]
pub fn swing_twist_decomposition<F>(
    src: &UnitQuaternion<F>,
    axis: &UnitVector3<F>,
) -> (UnitQuaternion<F>, UnitQuaternion<F>)
where
    F: SimdRealField + Copy,
    F::Element: SimdRealField,
{
    let rotation_axis = Vector3::new(src.i, src.j, src.k);
    let dot = rotation_axis.dot(axis.as_ref());
    let projection = axis.into_inner() * dot;
    let twist = UnitQuaternion::new_normalize(Quaternion::new(
        src.w,
        projection.x,
        projection.y,
        projection.z,
    ));
    let swing = src * twist.conjugate();
    (swing, twist)
}

/// Calculates the instantaneous angular velocity that has to be applied to `q1` to reach `q2` in `dt` seconds.
///
/// This is an approximation and may not be accurate for large rotations.
///
/// # Source
/// 1. https://mariogc.com/post/angular-velocity-quaternions
pub fn quat_to_angular_velocity<F>(
    q1: UnitQuaternion<F>,
    q2: UnitQuaternion<F>,
    dt: F,
) -> Vector3<F>
where
    F: SimdRealField + Copy,
    F::Element: SimdRealField,
{
    Vector3::new(
        q1.w * q2.i - q1.i * q2.w - q1.j * q2.k + q1.k * q2.j,
        q1.w * q2.j + q1.i * q2.k - q1.j * q2.w - q1.k * q2.i,
        q1.w * q2.k - q1.i * q2.j + q1.j * q2.i - q1.k * q2.w,
    ) * ((F::one() + F::one()) / dt)
}

/// Applies the given angular velocity to `q1` for `dt` seconds.
///
/// # Source
/// 1. https://gamedev.stackexchange.com/questions/108920/applying-angular-velocity-to-quaternion
pub fn apply_angular_velocity<F>(
    q1: UnitQuaternion<F>,
    angular_velocity: Vector3<F>,
    dt: F,
) -> UnitQuaternion<F>
where
    F: SimdRealField + Copy,
    F::Element: SimdRealField,
{
    let q1 = q1.into_inner();
    UnitQuaternion::new_normalize(
        q1 + Quaternion::new(
            F::zero(),
            angular_velocity.x,
            angular_velocity.y,
            angular_velocity.z,
        ) * q1
            * dt
            / (F::one() + F::one()),
    )
}

/// Converts the given angular velocity to a quaternion rotation for `dt` seconds.
///
/// This is an alternative to using [`apply_angular_velocity`] on the identity quaternion which may be faster.
///
/// # Source
/// 1. https://math.stackexchange.com/questions/39553/how-do-i-apply-an-angular-velocity-vector3-to-a-unit-quaternion-orientation
pub fn angular_velocity_to_quat<F>(mut angular_velocity: Vector3<F>, dt: F) -> UnitQuaternion<F>
where
    F: SimdRealField + Copy + RealField,
    F::Element: SimdRealField,
{
    angular_velocity *= dt;
    let magnitude = angular_velocity.magnitude();

    let two = F::one() + F::one();
    let multiplier = (magnitude / two).sin() / magnitude;

    UnitQuaternion::new_unchecked(Quaternion::new(
        (magnitude / two).cos(),
        angular_velocity.x * multiplier,
        angular_velocity.y * multiplier,
        angular_velocity.z * multiplier,
    ))
}

#[derive(Debug, Clone, Copy)]
pub struct SteeringLerper {
    steering: &'static AtomicCell<Option<Steering>>,
}

impl SteeringLerper {
    pub fn new(mut on_calculated: impl FnMut(f64, f64) + Send + 'static) -> Self {
        let steering: &AtomicCell<Option<Steering>> = Box::leak(Box::new(AtomicCell::new(None)));

        std::thread::spawn(move || {
            let mut state = (0.0, 0.0);
            let mut target_steering = Steering::default();
            let sleeper = SpinSleeper::default();
            let mut empty_count = 0usize;
            loop {
                sleeper.sleep(std::time::Duration::from_millis(50));
                let weight = target_steering.get_weight();
                let (left, right) = target_steering.get_left_and_right();
                state = (
                    lerp(state.0, left, 0.05, weight),
                    lerp(state.1, right, 0.05, weight),
                );
                on_calculated(state.0, state.1);
                let Some(tmp) = steering.take() else {
                    empty_count += 1;
                    if empty_count > 4 {
                        on_calculated(0.0, 0.0);
                        state = (0.0, 0.0);
                    }
                    continue;
                };
                target_steering = tmp;
                empty_count = 0;
            }
        });

        Self { steering }
    }

    pub fn set_steering(&self, steering: Steering) {
        self.steering.store(Some(steering));
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{UnitQuaternion, Vector3};

    #[test]
    fn approx_invertibility_test01() {
        let mut q1 = UnitQuaternion::<f64>::identity();
        let angular_velocity = Vector3::new(1.0, 3.0, -2.3);
        q1 = super::apply_angular_velocity(q1, angular_velocity, 0.016);
        let actual_angular_velocity =
            super::quat_to_angular_velocity(UnitQuaternion::default(), q1, 0.016);
        assert!(
            (angular_velocity - actual_angular_velocity).magnitude() < 1e-2,
            "{:?}",
            actual_angular_velocity
        );
    }

    #[test]
    fn invertibility_test01() {
        let mut q1 = UnitQuaternion::<f64>::identity();
        let angular_velocity = Vector3::new(1.0, 3.0, -2.3);
        q1 = super::angular_velocity_to_quat(angular_velocity, 0.016) * q1;
        let actual_angular_velocity =
            super::quat_to_angular_velocity(UnitQuaternion::default(), q1, 0.016);
        assert!(
            (angular_velocity - actual_angular_velocity).magnitude() < 1e-2,
            "{:?}",
            actual_angular_velocity
        );
    }
}