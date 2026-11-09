//! Constraints and other types used for solving contacts.

mod normal_part;
mod tangent_part;

pub use normal_part::ContactNormalPart;
pub use tangent_part::ContactTangentPart;

use core::cmp::Ordering;

use crate::{
    collision::contact_types::ContactId, dynamics::solver::ContactSoftnessCoefficients, prelude::*,
};
#[cfg(feature = "serialize")]
use bevy::reflect::{ReflectDeserialize, ReflectSerialize};
use bevy::{prelude::Entity, reflect::Reflect, utils::default};

use tangent_part::TangentImpulse;

use super::solver_body::{
    SolverBodiesAccess, SolverBody, SolverBodyIndex, SolverBodyInertia,
};

/// Clamp the *applied* cached friction to the Coulomb limit of a capped
/// normal contact. Leave ordinary contacts' existing warm start unchanged.
#[inline]
fn warm_start_tangent_impulse(
    cached: TangentImpulse,
    coefficient: f32,
    friction: f32,
    applied_normal_impulse: f32,
    limited: bool,
) -> TangentImpulse {
    let impulse = coefficient * cached;
    if !limited {
        return impulse;
    }

    let limit = friction.max(0.0) * applied_normal_impulse;
    #[cfg(feature = "2d")]
    {
        impulse.clamp(-limit, limit)
    }
    #[cfg(feature = "3d")]
    {
        impulse.clamp_length_max(limit)
    }
}

// TODO: One-body constraint version
/// Data and logic for solving a single contact point for a [`ContactConstraint`].
#[derive(Clone, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct ContactConstraintPoint {
    /// The normal part of the contact constraint.
    pub normal_part: ContactNormalPart,

    /// The tangential friction part of the contact constraint.
    ///
    /// `None` if the coefficient of friction is zero.
    pub tangent_part: Option<ContactTangentPart>,

    /// The world-space contact point relative to the center of mass of the first body.
    pub anchor1: Vector,

    /// The world-space contact point relative to the center of mass of the second body.
    pub anchor2: Vector,

    /// The pre-solve relative velocity of the bodies along the normal at the contact point.
    pub normal_speed: f32,

    /// The pre-solve separation distance between the bodies.
    ///
    /// A negative separation indicates penetration.
    pub initial_separation: f32,

    /// Net impulse applied to this point during the current substep.
    /// Includes warm starting and every subsequent solver correction.
    pub(crate) substep_applied_normal_impulse: f32,

    /// Net friction impulse applied to this point during the current substep.
    pub(crate) substep_applied_tangent_impulse: TangentImpulse,
}

impl ContactConstraintPoint {
    /// Returns the net contact impulses applied during the current substep.
    ///
    /// The tangent impulse is expressed in the contact's tangent basis. In 3D,
    /// its two components correspond to the two orthonormal tangent directions.
    /// The ledger remains solver-owned and is exposed read-only for systems
    /// that evaluate material response from the actual solved contact.
    #[cfg(feature = "2d")]
    #[inline]
    pub fn substep_applied_impulses(&self) -> (f32, f32) {
        (
            self.substep_applied_normal_impulse,
            self.substep_applied_tangent_impulse,
        )
    }

    #[cfg(feature = "3d")]
    #[inline]
    pub fn substep_applied_impulses(&self) -> (f32, bevy_math::Vec2) {
        (
            self.substep_applied_normal_impulse,
            self.substep_applied_tangent_impulse,
        )
    }
}

/// A contact constraint used for resolving inter-penetration between two bodies.
///
/// Each constraint corresponds to a [`ContactManifold`] indicated by the `manifold_index`.
/// The contact points are stored in `points`, and they all share the same `normal`.
#[derive(Clone, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct ContactConstraint {
    /// The [`SolverBodyIndex`] of the first body in the contact,
    /// or [`SolverBodyIndex::INVALID`] if it is static or sleeping.
    pub body_index1: SolverBodyIndex,
    /// The [`SolverBodyIndex`] of the second body in the contact,
    /// or [`SolverBodyIndex::INVALID`] if it is static or sleeping.
    pub body_index2: SolverBodyIndex,
    /// Entity identities are retained so a contact can resolve fresh solver
    /// indices after another body is removed during a topology edit.
    pub(super) body_entity1: Option<Entity>,
    pub(super) body_entity2: Option<Entity>,
    /// The relative dominance of the bodies.
    ///
    /// If the relative dominance is positive, the first body is dominant
    /// and is considered to have infinite mass.
    pub relative_dominance: i16,
    /// The combined coefficient of dynamic [friction](Friction) of the bodies.
    pub friction: f32,
    /// The combined coefficient of [restitution](Restitution) of the bodies.
    pub restitution: f32,
    /// The desired relative linear speed of the bodies along the surface,
    /// expressed in world space as `tangent_speed2 - tangent_speed1`.
    ///
    /// Defaults to zero. If set to a non-zero value, this can be used to simulate effects
    /// such as conveyor belts.
    #[cfg(feature = "2d")]
    pub tangent_speed: f32,
    /// The desired relative linear velocity of the bodies along the surface,
    /// expressed in world space as `tangent_velocity2 - tangent_velocity1`.
    ///
    /// Defaults to zero. If set to a non-zero value, this can be used to simulate effects
    /// such as conveyor belts.
    #[cfg(feature = "3d")]
    pub tangent_velocity: Vector,
    /// The world-space contact normal shared by all points in the contact manifold.
    pub normal: Vector,
    /// The first world-space tangent direction shared by all points in the contact manifold.
    #[cfg(feature = "3d")]
    pub tangent1: Vector,
    /// The contact points in the manifold. Each point shares the same `normal`.
    // TODO: Use a `SmallVec`
    pub points: Vec<ContactConstraintPoint>,
    /// The stable identifier of the [`ContactEdge`] in the [`ContactGraph`].
    ///
    /// [`ContactEdge`]: crate::collision::contact_types::ContactEdge
    pub contact_id: ContactId,
    /// The index of the contact manifold in the [`ContactPair`].
    pub manifold_index: usize,
}

impl ContactConstraint {
    /// Generates a new [`ContactConstraint`] from the given bodies and contact manifold.
    pub(super) fn generate(
        body_index1: SolverBodyIndex,
        body_index2: SolverBodyIndex,
        body_entity1: Option<Entity>,
        body_entity2: Option<Entity>,
        inertia1: &SolverBodyInertia,
        inertia2: &SolverBodyInertia,
        linear_velocity1: Vector,
        linear_velocity2: Vector,
        contact_id: ContactId,
        manifold: &ContactManifold,
        manifold_index: usize,
        warm_start_enabled: bool,
        softness: &ContactSoftnessCoefficients,
    ) -> Self {
        // Compute the relative dominance of the bodies.
        let relative_dominance = inertia1.dominance() - inertia2.dominance();

        // Compute the inverse mass and angular inertia, taking into account the relative dominance.
        let (inv_mass1, i1, inv_mass2, i2) = match relative_dominance.cmp(&0) {
            Ordering::Equal => (
                inertia1.effective_inv_mass(),
                inertia1.effective_inv_angular_inertia(),
                inertia2.effective_inv_mass(),
                inertia2.effective_inv_angular_inertia(),
            ),
            Ordering::Greater => (
                Vector::ZERO,
                SymmetricTensor::ZERO,
                inertia2.effective_inv_mass(),
                inertia2.effective_inv_angular_inertia(),
            ),
            Ordering::Less => (
                inertia1.effective_inv_mass(),
                inertia1.effective_inv_angular_inertia(),
                Vector::ZERO,
                SymmetricTensor::ZERO,
            ),
        };

        let softness = if relative_dominance != 0 {
            softness.non_dynamic
        } else {
            softness.dynamic
        };

        let effective_inverse_mass_sum = inv_mass1 + inv_mass2;

        let tangents =
            compute_tangent_directions(manifold.normal, linear_velocity1, linear_velocity2);

        let mut points = Vec::with_capacity(manifold.points.len());

        for point in manifold.points.iter() {
            // Use fixed world-space anchors.
            // This improves rolling behavior for shapes like balls and capsules.
            let anchor1 = point.anchor1;
            let anchor2 = point.anchor2;

            let mut point = ContactConstraintPoint {
                // TODO: Apply warm starting scale here instead of in `warm_start`?
                normal_part: ContactNormalPart::generate(
                    effective_inverse_mass_sum,
                    &i1,
                    &i2,
                    anchor1,
                    anchor2,
                    manifold.normal,
                    warm_start_enabled.then_some(point.warm_start_normal_impulse),
                    softness,
                ),
                // There should only be a friction part if the coefficient of friction is non-negative.
                tangent_part: (manifold.friction > 0.0).then_some(ContactTangentPart::generate(
                    effective_inverse_mass_sum,
                    &i1,
                    &i2,
                    anchor1,
                    anchor2,
                    tangents,
                    warm_start_enabled.then_some(point.warm_start_tangent_impulse),
                )),
                anchor1,
                anchor2,
                normal_speed: point.normal_speed,
                initial_separation: -point.penetration - (anchor2 - anchor1).dot(manifold.normal),
                substep_applied_normal_impulse: 0.0,
                substep_applied_tangent_impulse: default(),
            };

            point
                .normal_part
                .set_maximum_normal_impulse(manifold.max_normal_impulse);

            points.push(point);
        }

        ContactConstraint {
            body_index1,
            body_index2,
            body_entity1,
            body_entity2,
            relative_dominance,
            friction: manifold.friction,
            restitution: manifold.restitution,
            #[cfg(feature = "2d")]
            tangent_speed: manifold.tangent_speed,
            #[cfg(feature = "3d")]
            tangent_velocity: manifold.tangent_velocity,
            normal: manifold.normal,
            #[cfg(feature = "3d")]
            tangent1: tangents[0],
            points,
            contact_id,
            manifold_index,
        }
    }

    /// Warm starts the contact constraint by applying the impulses from the previous frame or substep.
    pub fn warm_start(
        &mut self,
        body1: &mut SolverBody,
        body2: &mut SolverBody,
        inertia1: &SolverBodyInertia,
        inertia2: &SolverBodyInertia,
        warm_start_coefficient: f32,
    ) {
        let inv_mass1 = inertia1.effective_inv_mass();
        let inv_mass2 = inertia2.effective_inv_mass();
        let inv_angular_inertia1 = inertia1.effective_inv_angular_inertia();
        let inv_angular_inertia2 = inertia2.effective_inv_angular_inertia();

        let tangent_directions = self.tangent_directions();

        for point in self.points.iter_mut() {
            point.normal_part.reset_substep_state();
            // The ledger is local to this substep. It records the complete
            // impulse applied to the bodies, not only the solver's accumulated
            // lambda delta, so a topology edit can reverse a stale contact.
            point.substep_applied_normal_impulse = 0.0;
            point.substep_applied_tangent_impulse = default();

            // Fixed anchors
            let r1 = point.anchor1;
            let r2 = point.anchor2;

            let normal_impulse = point
                .normal_part
                .warm_start_impulse(warm_start_coefficient);
            let tangent_impulse = warm_start_tangent_impulse(
                point.tangent_part.as_ref().map_or(default(), |part| part.impulse),
                warm_start_coefficient,
                self.friction,
                normal_impulse,
                point.normal_part.max_impulse.is_finite(),
            );
            point.substep_applied_normal_impulse += normal_impulse;
            point.substep_applied_tangent_impulse += tangent_impulse;

            #[cfg(feature = "2d")]
            let p = normal_impulse * self.normal
                + tangent_impulse * tangent_directions[0];
            #[cfg(feature = "3d")]
            let p = normal_impulse * self.normal
                + tangent_impulse.x * tangent_directions[0]
                + tangent_impulse.y * tangent_directions[1];

            body1.linear_velocity -= p * inv_mass1;
            body1.angular_velocity -= inv_angular_inertia1 * cross(r1, p);

            body2.linear_velocity += p * inv_mass2;
            body2.angular_velocity += inv_angular_inertia2 * cross(r2, p);
        }
    }

    /// Reverses this contact's net impulse from the current substep.
    ///
    /// Reverses this contact's net impulse from the current substep.
    ///
    /// Earlier substeps have already integrated their positions and remain
    /// committed. Fracture rollback therefore reverses only the current
    /// substep's impulse ledger, before this substep's position integration.
    pub(crate) fn rollback_substep_impulses(&mut self, access: &SolverBodiesAccess) {
        let mut dummy_body1 = SolverBody::DUMMY;
        let mut dummy_body2 = SolverBody::DUMMY;
        let (mut body1, mut inertia1) = (&mut dummy_body1, &SolverBodyInertia::DUMMY);
        let (mut body2, mut inertia2) = (&mut dummy_body2, &SolverBodyInertia::DUMMY);

        // SAFETY: This method is called sequentially while the constraint graph
        // is being rebuilt. No other solver access is active at this barrier.
        let (b1, b2) =
            unsafe { access.get_pair_unchecked_mut(self.body_index1, self.body_index2) };
        if let Some((body, inertia)) = b1 {
            body1 = body;
            inertia1 = inertia;
        }
        if let Some((body, inertia)) = b2 {
            body2 = body;
            inertia2 = inertia;
        }

        match self.relative_dominance.cmp(&0) {
            Ordering::Greater => inertia1 = &SolverBodyInertia::DUMMY,
            Ordering::Less => inertia2 = &SolverBodyInertia::DUMMY,
            Ordering::Equal => {}
        }

        let inv_mass1 = inertia1.effective_inv_mass();
        let inv_mass2 = inertia2.effective_inv_mass();
        let inv_angular_inertia1 = inertia1.effective_inv_angular_inertia();
        let inv_angular_inertia2 = inertia2.effective_inv_angular_inertia();
        let tangent_directions = self.tangent_directions();

        for point in &mut self.points {
            let normal_impulse = point.substep_applied_normal_impulse;
            let tangent_impulse = point.substep_applied_tangent_impulse;
            #[cfg(feature = "2d")]
            let impulse = normal_impulse * self.normal
                + tangent_impulse * tangent_directions[0];
            #[cfg(feature = "3d")]
            let impulse = normal_impulse * self.normal
                + tangent_impulse.x * tangent_directions[0]
                + tangent_impulse.y * tangent_directions[1];

            let r1 = point.anchor1;
            let r2 = point.anchor2;

            body1.linear_velocity += impulse * inv_mass1;
            body1.angular_velocity += inv_angular_inertia1 * cross(r1, impulse);
            body2.linear_velocity -= impulse * inv_mass2;
            body2.angular_velocity -= inv_angular_inertia2 * cross(r2, impulse);

            point.substep_applied_normal_impulse = 0.0;
            point.substep_applied_tangent_impulse = default();
        }
    }

    fn solve_normal_block<const USE_BIAS: bool>(
        &mut self,
        body1: &mut SolverBody,
        body2: &mut SolverBody,
        inertia1: &SolverBodyInertia,
        inertia2: &SolverBodyInertia,
        delta_secs: f32,
        max_overlap_solve_speed: f32,
    ) -> bool {
        const MAX_POINTS: usize = 4;
        let count = self.points.len();
        if !(2..=MAX_POINTS).contains(&count) {
            return false;
        }

        let inv_mass_sum = inertia1.effective_inv_mass() + inertia2.effective_inv_mass();
        let inv_angular_inertia1 = inertia1.effective_inv_angular_inertia();
        let inv_angular_inertia2 = inertia2.effective_inv_angular_inertia();
        let safe_dt = if delta_secs.is_finite() && delta_secs > 1.0e-6 {
            delta_secs
        } else {
            1.0e-6
        };

        let mut anchors1 = [Vector::ZERO; MAX_POINTS];
        let mut anchors2 = [Vector::ZERO; MAX_POINTS];
        let mut old_impulses = [0.0_f64; MAX_POINTS];
        let mut lower = [0.0_f64; MAX_POINTS];
        let mut upper = [0.0_f64; MAX_POINTS];
        let mut normal_speeds = [0.0_f64; MAX_POINTS];
        let mut biases = [0.0_f64; MAX_POINTS];
        let mut mass_scales = [1.0_f64; MAX_POINTS];
        let mut impulse_scales = [0.0_f64; MAX_POINTS];
        let mut matrix = [[0.0_f64; MAX_POINTS]; MAX_POINTS];

        for i in 0..count {
            let point = &self.points[i];
            let r1 = point.anchor1;
            let r2 = point.anchor2;
            anchors1[i] = r1;
            anchors2[i] = r2;

            let rotated_r1 = body1.delta_rotation * r1;
            let rotated_r2 = body2.delta_rotation * r2;
            let separation = (
                body2.delta_position - body1.delta_position + rotated_r2 - rotated_r1
            ).dot(self.normal) + point.initial_separation;

            let relative_velocity = body2.velocity_at_point(r2) - body1.velocity_at_point(r1);
            normal_speeds[i] = f64::from(relative_velocity.dot(self.normal));

            let (mass_scale, impulse_scale, bias) = if separation > 0.0 {
                (1.0, 0.0, separation / safe_dt)
            } else if USE_BIAS {
                let softness = point.normal_part.softness;
                (
                    softness.mass_scale,
                    softness.impulse_scale,
                    (softness.bias * separation).max(-max_overlap_solve_speed),
                )
            } else {
                (1.0, 0.0, 0.0)
            };

            let old = point.normal_part.impulse;
            let capacity = point.normal_part.max_impulse;
            if !old.is_finite()
                || old < 0.0
                || !mass_scale.is_finite()
                || mass_scale < 0.0
                || !impulse_scale.is_finite()
                || !bias.is_finite()
                || capacity.is_nan()
                || capacity < 0.0
            {
                return false;
            }

            old_impulses[i] = f64::from(old);
            lower[i] = -old_impulses[i];
            upper[i] = if capacity.is_finite() {
                f64::from(capacity) - old_impulses[i]
            } else {
                f64::INFINITY
            };
            if upper[i] < lower[i] {
                if lower[i] - upper[i] <= 1.0e-8 {
                    upper[i] = lower[i];
                } else {
                    return false;
                }
            }

            biases[i] = f64::from(bias);
            mass_scales[i] = f64::from(mass_scale);
            impulse_scales[i] = f64::from(impulse_scale);
        }

        for i in 0..count {
            for j in i..count {
                let entry = normal_effective_mass_entry(
                    anchors1[i],
                    anchors2[i],
                    anchors1[j],
                    anchors2[j],
                    self.normal,
                    inv_mass_sum,
                    &inv_angular_inertia1,
                    &inv_angular_inertia2,
                );
                if !entry.is_finite() {
                    return false;
                }
                matrix[i][j] = entry;
                matrix[j][i] = entry;
            }
        }

        let mut linear = [0.0_f64; MAX_POINTS];
        for i in 0..count {
            let coupled_old_impulse = (0..count)
                .map(|j| matrix[i][j] * old_impulses[j])
                .sum::<f64>();
            linear[i] = mass_scales[i] * (normal_speeds[i] + biases[i])
                + impulse_scales[i] * coupled_old_impulse;
        }

        let Some(delta_impulses) =
            solve_bounded_normal_block(count, &matrix, &linear, &lower, &upper, &old_impulses)
        else {
            return false;
        };

        // All point impulses are solved before any body velocity is changed.
        // Applying the resulting deltas afterwards preserves the coupled
        // solution instead of reintroducing point-order dependence.
        for i in 0..count {
            let point = &mut self.points[i];
            let requested_accumulated = old_impulses[i] + delta_impulses[i];
            let gradient = linear[i]
                + (0..count)
                    .map(|j| matrix[i][j] * delta_impulses[j])
                    .sum::<f64>();
            let gradient_scale = 1.0
                + linear[i].abs()
                + (0..count)
                    .map(|j| (matrix[i][j] * delta_impulses[j]).abs())
                    .sum::<f64>();
            let gradient_tolerance = 1.0e-7 * gradient_scale;

            let capacity = point.normal_part.max_impulse;
            let at_capacity = capacity.is_finite()
                && requested_accumulated >= f64::from(capacity)
                    - 1.0e-7 * (1.0 + f64::from(capacity.abs()));

            let requested = if at_capacity && gradient < -gradient_tolerance {
                let diagonal = matrix[i][i];
                let excess = if diagonal > 1.0e-12 {
                    (-gradient / diagonal).max(0.0)
                } else {
                    0.0
                };
                f64::from(capacity) + excess
            } else {
                requested_accumulated
            };

            let requested_f32 = requested.min(f64::from(f32::MAX)) as f32;
            let applied_delta = point
                .normal_part
                .clamp_accumulated_impulse(requested_f32);
            if at_capacity && gradient < -gradient_tolerance {
                point.normal_part.capacity_exceeded = true;
            }
            point.normal_part.total_impulse += applied_delta;
            point.substep_applied_normal_impulse += applied_delta;

            let impulse = applied_delta * self.normal;
            let r1 = point.anchor1;
            let r2 = point.anchor2;
            body1.linear_velocity -= impulse * inertia1.effective_inv_mass();
            body1.angular_velocity -= inv_angular_inertia1 * cross(r1, impulse);
            body2.linear_velocity += impulse * inertia2.effective_inv_mass();
            body2.angular_velocity += inv_angular_inertia2 * cross(r2, impulse);
        }

        true
    }

    /// Solves the [`ContactConstraint`], applying an impulse to the given bodies.
    pub fn solve<const USE_BIAS: bool>(
        &mut self,
        body1: &mut SolverBody,
        body2: &mut SolverBody,
        inertia1: &SolverBodyInertia,
        inertia2: &SolverBodyInertia,
        delta_secs: f32,
        max_overlap_solve_speed: f32,
    ) {
        let inv_mass1 = inertia1.effective_inv_mass();
        let inv_mass2 = inertia2.effective_inv_mass();
        let inv_angular_inertia1 = inertia1.effective_inv_angular_inertia();
        let inv_angular_inertia2 = inertia2.effective_inv_angular_inertia();

        let delta_translation = body2.delta_position - body1.delta_position;

        // Solve the complete normal contact patch as one bounded system.
        // A sequential point solve is retained only for unusually large or
        // numerically degenerate manifolds.
        if !self.solve_normal_block::<USE_BIAS>(
            body1,
            body2,
            inertia1,
            inertia2,
            delta_secs,
            max_overlap_solve_speed,
        ) {
            for point in self.points.iter_mut() {
                let r1 = body1.delta_rotation * point.anchor1;
                let r2 = body2.delta_rotation * point.anchor2;

                let delta_separation = delta_translation + (r2 - r1);
                let separation = delta_separation.dot(self.normal) + point.initial_separation;

                let r1 = point.anchor1;
                let r2 = point.anchor2;
                let relative_velocity = body2.velocity_at_point(r2) - body1.velocity_at_point(r1);
                let impulse_magnitude = point.normal_part.solve_impulse::<USE_BIAS>(
                    separation,
                    relative_velocity,
                    self.normal,
                    max_overlap_solve_speed,
                    delta_secs,
                );

                let impulse = impulse_magnitude * self.normal;
                point.substep_applied_normal_impulse += impulse_magnitude;

                body1.linear_velocity -= impulse * inv_mass1;
                body1.angular_velocity -= inv_angular_inertia1 * cross(r1, impulse);
                body2.linear_velocity += impulse * inv_mass2;
                body2.angular_velocity += inv_angular_inertia2 * cross(r2, impulse);
            }
        }

        // Friction impulses, only during the relaxation stage.
        // Applying friction during the bias stage does not meaningfully improve quality for the cost.
        if !USE_BIAS {
            let tangent_directions = self.tangent_directions();

            for point in self.points.iter_mut() {
                let Some(ref mut friction_part) = point.tangent_part else {
                    continue;
                };

                // Fixed anchors
                let r1 = point.anchor1;
                let r2 = point.anchor2;

                // Relative velocity at contact point
                let relative_velocity = body2.velocity_at_point(r2) - body1.velocity_at_point(r1);

                // Compute the incremental impulse. The clamping and impulse accumulation is handled by the method.
                let impulse = friction_part.solve_impulse(
                    tangent_directions,
                    relative_velocity,
                    #[cfg(feature = "2d")]
                    self.tangent_speed,
                    #[cfg(feature = "3d")]
                    self.tangent_velocity,
                    self.friction,
                    point.normal_part.impulse,
                );

                #[cfg(feature = "2d")]
                let tangent_impulse = impulse.dot(tangent_directions[0]);
                #[cfg(feature = "3d")]
                let tangent_impulse = bevy_math::Vec2::new(
                    impulse.dot(tangent_directions[0]),
                    impulse.dot(tangent_directions[1]),
                );

                point.substep_applied_tangent_impulse += tangent_impulse;

                // Apply the impulse.
                body1.linear_velocity -= impulse * inv_mass1;
                body1.angular_velocity -= inv_angular_inertia1 * cross(r1, impulse);

                body2.linear_velocity += impulse * inv_mass2;
                body2.angular_velocity += inv_angular_inertia2 * cross(r2, impulse);
            }
        }
    }

    fn apply_restitution_block(
        &mut self,
        body1: &mut SolverBody,
        body2: &mut SolverBody,
        inertia1: &SolverBodyInertia,
        inertia2: &SolverBodyInertia,
        threshold: f32,
    ) -> bool {
        const MAX_POINTS: usize = 4;
        let count = self.points.len();
        if !(2..=MAX_POINTS).contains(&count) {
            return false;
        }

        let inv_mass_sum = inertia1.effective_inv_mass() + inertia2.effective_inv_mass();
        let inv_angular_inertia1 = inertia1.effective_inv_angular_inertia();
        let inv_angular_inertia2 = inertia2.effective_inv_angular_inertia();
        let mut anchors1 = [Vector::ZERO; MAX_POINTS];
        let mut anchors2 = [Vector::ZERO; MAX_POINTS];
        let mut old_impulses = [0.0_f64; MAX_POINTS];
        let mut lower = [0.0_f64; MAX_POINTS];
        let mut upper = [0.0_f64; MAX_POINTS];
        let mut linear = [0.0_f64; MAX_POINTS];
        let mut eligible = [false; MAX_POINTS];
        let mut matrix = [[0.0_f64; MAX_POINTS]; MAX_POINTS];

        for i in 0..count {
            let point = &self.points[i];
            anchors1[i] = point.anchor1;
            anchors2[i] = point.anchor2;
            let old = point.normal_part.impulse;
            let capacity = point.normal_part.max_impulse;
            if !old.is_finite()
                || old < 0.0
                || capacity.is_nan()
                || capacity < 0.0
            {
                return false;
            }
            old_impulses[i] = f64::from(old);

            if point.normal_speed > -threshold || point.normal_part.total_impulse == 0.0 {
                lower[i] = 0.0;
                upper[i] = 0.0;
                continue;
            }

            eligible[i] = true;
            let relative_velocity =
                body2.velocity_at_point(point.anchor2) - body1.velocity_at_point(point.anchor1);
            linear[i] = f64::from(
                relative_velocity.dot(self.normal) + self.restitution * point.normal_speed,
            );
            upper[i] = if capacity.is_finite() {
                (f64::from(capacity) - old_impulses[i]).max(0.0)
            } else {
                f64::INFINITY
            };
        }

        for i in 0..count {
            for j in i..count {
                let entry = normal_effective_mass_entry(
                    anchors1[i],
                    anchors2[i],
                    anchors1[j],
                    anchors2[j],
                    self.normal,
                    inv_mass_sum,
                    &inv_angular_inertia1,
                    &inv_angular_inertia2,
                );
                if !entry.is_finite() {
                    return false;
                }
                matrix[i][j] = entry;
                matrix[j][i] = entry;
            }
        }

        let Some(delta_impulses) =
            solve_bounded_normal_block(
                count,
                &matrix,
                &linear,
                &lower,
                &upper,
                &old_impulses,
            )
        else {
            return false;
        };

        for i in 0..count {
            if !eligible[i] {
                continue;
            }

            let point = &mut self.points[i];
            let requested_accumulated = old_impulses[i] + delta_impulses[i];
            let gradient = linear[i]
                + (0..count)
                    .map(|j| matrix[i][j] * delta_impulses[j])
                    .sum::<f64>();
            let gradient_scale = 1.0
                + linear[i].abs()
                + (0..count)
                    .map(|j| (matrix[i][j] * delta_impulses[j]).abs())
                    .sum::<f64>();
            let gradient_tolerance = 1.0e-7 * gradient_scale;
            let capacity = point.normal_part.max_impulse;
            let at_capacity = capacity.is_finite()
                && requested_accumulated >= f64::from(capacity)
                    - 1.0e-7 * (1.0 + f64::from(capacity.abs()));

            let requested = if at_capacity && gradient < -gradient_tolerance {
                let diagonal = matrix[i][i];
                let excess = if diagonal > 1.0e-12 {
                    (-gradient / diagonal).max(0.0)
                } else {
                    0.0
                };
                f64::from(capacity) + excess
            } else {
                requested_accumulated
            };

            let applied_delta = point.normal_part.clamp_accumulated_impulse(
                requested.min(f64::from(f32::MAX)) as f32,
            );
            if at_capacity && gradient < -gradient_tolerance {
                point.normal_part.capacity_exceeded = true;
            }
            point.normal_part.total_impulse += applied_delta;
            point.substep_applied_normal_impulse += applied_delta;

            let impulse = applied_delta * self.normal;
            body1.linear_velocity -= impulse * inertia1.effective_inv_mass();
            body1.angular_velocity -=
                inv_angular_inertia1 * cross(point.anchor1, impulse);
            body2.linear_velocity += impulse * inertia2.effective_inv_mass();
            body2.angular_velocity +=
                inv_angular_inertia2 * cross(point.anchor2, impulse);
        }

        true
    }

    /// Applies [restitution]([`Restitution`]) for the given bodies if the relative speed
    /// along the contact normal exceeds the given `threshold`.
    pub fn apply_restitution(
        &mut self,
        body1: &mut SolverBody,
        body2: &mut SolverBody,
        inertia1: &SolverBodyInertia,
        inertia2: &SolverBodyInertia,
        threshold: f32,
    ) {
        if self.apply_restitution_block(body1, body2, inertia1, inertia2, threshold) {
            return;
        }

        let inv_mass1 = inertia1.effective_inv_mass();
        let inv_mass2 = inertia2.effective_inv_mass();
        let inv_angular_inertia1 = inertia1.effective_inv_angular_inertia();
        let inv_angular_inertia2 = inertia2.effective_inv_angular_inertia();

        for point in self.points.iter_mut() {
            if point.normal_speed > -threshold || point.normal_part.total_impulse == 0.0 {
                continue;
            }

            let r1 = point.anchor1;
            let r2 = point.anchor2;
            let relative_velocity = body2.velocity_at_point(r2) - body1.velocity_at_point(r1);
            let normal_speed = relative_velocity.dot(self.normal);
            let mut impulse = -point.normal_part.effective_mass
                * (normal_speed + self.restitution * point.normal_speed);

            impulse = point.normal_part.clamp_accumulated_impulse(
                point.normal_part.impulse + impulse,
            );
            point.normal_part.total_impulse += impulse;
            point.substep_applied_normal_impulse += impulse;

            let impulse = impulse * self.normal;
            body1.linear_velocity -= impulse * inv_mass1;
            body1.angular_velocity -= inv_angular_inertia1 * cross(r1, impulse);
            body2.linear_velocity += impulse * inv_mass2;
            body2.angular_velocity += inv_angular_inertia2 * cross(r2, impulse);
        }
    }

    /// Returns the tangent directions for the contact constraint.
    #[inline(always)]
    pub fn tangent_directions(&self) -> [Vector; DIM - 1] {
        #[cfg(feature = "2d")]
        {
            [Vector::new(self.normal.y, -self.normal.x)]
        }
        #[cfg(feature = "3d")]
        {
            // Note: The order is flipped here so that we use `-normal`.
            [self.tangent1, self.tangent1.cross(self.normal)]
        }
    }
}


/// Computes one entry of the coupled normal effective-mass matrix.
#[inline]
fn normal_effective_mass_entry(
    r1_i: Vector,
    r2_i: Vector,
    r1_j: Vector,
    r2_j: Vector,
    normal: Vector,
    inv_mass_sum: Vector,
    inv_angular_inertia1: &SymmetricTensor,
    inv_angular_inertia2: &SymmetricTensor,
) -> f64 {
    let linear = normal.dot(inv_mass_sum * normal);
    let r1_i_cross_n = cross(r1_i, normal);
    let r1_j_cross_n = cross(r1_j, normal);
    let r2_i_cross_n = cross(r2_i, normal);
    let r2_j_cross_n = cross(r2_j, normal);

    #[cfg(feature = "2d")]
    let angular = inv_angular_inertia1 * r1_i_cross_n * r1_j_cross_n
        + inv_angular_inertia2 * r2_i_cross_n * r2_j_cross_n;

    #[cfg(feature = "3d")]
    let angular = r1_i_cross_n.dot(inv_angular_inertia1 * r1_j_cross_n)
        + r2_i_cross_n.dot(inv_angular_inertia2 * r2_j_cross_n);

    f64::from(linear + angular)
}

/// Solves a tiny convex quadratic program with box constraints. Active-set
/// enumeration is exact for the usual two-to-four-point contact patch; a
/// symmetric projected-gradient fallback handles rank-deficient manifolds.
fn solve_bounded_normal_block(
    count: usize,
    matrix: &[[f64; 4]; 4],
    linear: &[f64; 4],
    lower: &[f64; 4],
    upper: &[f64; 4],
    reference_impulses: &[f64; 4],
) -> Option<[f64; 4]> {
    if count < 2 || count > 4 {
        return None;
    }

    let matrix_scale = (0..count)
        .map(|i| matrix[i][i].abs())
        .fold(0.0_f64, f64::max);
    if !matrix_scale.is_finite() || matrix_scale <= 1.0e-14 {
        return None;
    }

    // Normal impulses in a planar 3D patch have a one-dimensional nullspace:
    // several point distributions can produce the same rigid-body wrench.
    // A vanishingly small diagonal term selects the minimum-norm distribution
    // instead of allowing manifold point order to choose one arbitrary corner.
    let mut hessian = *matrix;
    let regularizer = matrix_scale * 1.0e-10;
    let mut regularized_linear = *linear;
    for i in 0..count {
        hessian[i][i] += regularizer;
        // Select the minimum-norm final accumulated impulse, not merely the
        // minimum change from a potentially asymmetric warm-start cache.
        regularized_linear[i] += regularizer * reference_impulses[i];
    }

    let mut best: Option<([f64; 4], f64)> = None;
    let combinations = 3_usize.pow(count as u32);

    for mut encoded in 0..combinations {
        let mut state = [0_u8; 4];
        let mut value = [0.0_f64; 4];
        let mut free = [0_usize; 4];
        let mut free_count = 0_usize;
        let mut valid = true;

        for i in 0..count {
            let current = (encoded % 3) as u8;
            encoded /= 3;
            state[i] = current;

            let width = upper[i] - lower[i];
            let bound_tolerance = 1.0e-9
                * (1.0 + lower[i].abs() + if upper[i].is_finite() { upper[i].abs() } else { 0.0 });

            match current {
                0 => value[i] = lower[i],
                1 if width > bound_tolerance => {
                    free[free_count] = i;
                    free_count += 1;
                }
                2 if upper[i].is_finite() => {
                    value[i] = upper[i];
                }
                _ => {
                    valid = false;
                    break;
                }
            }
        }
        if !valid {
            continue;
        }

        if free_count > 0 {
            let mut augmented = [[0.0_f64; 5]; 4];
            for row in 0..free_count {
                let i = free[row];
                for column in 0..free_count {
                    let j = free[column];
                    augmented[row][column] = hessian[i][j];
                }
                let fixed_response = (0..count)
                    .filter(|j| state[*j] != 1)
                    .map(|j| hessian[i][j] * value[j])
                    .sum::<f64>();
                augmented[row][free_count] = -regularized_linear[i] - fixed_response;
            }

            let matrix_scale = (0..free_count)
                .map(|i| augmented[i][i].abs())
                .fold(0.0_f64, f64::max);
            let pivot_floor = (matrix_scale * 1.0e-12).max(1.0e-30);

            for column in 0..free_count {
                let pivot = (column..free_count)
                    .max_by(|a, b| {
                        augmented[*a][column]
                            .abs()
                            .total_cmp(&augmented[*b][column].abs())
                    })?;
                if augmented[pivot][column].abs() <= pivot_floor {
                    valid = false;
                    break;
                }
                if pivot != column {
                    augmented.swap(pivot, column);
                }

                let diagonal = augmented[column][column];
                for j in column..=free_count {
                    augmented[column][j] /= diagonal;
                }
                for row in 0..free_count {
                    if row == column {
                        continue;
                    }
                    let factor = augmented[row][column];
                    for j in column..=free_count {
                        augmented[row][j] -= factor * augmented[column][j];
                    }
                }
            }
            if !valid {
                continue;
            }

            for row in 0..free_count {
                value[free[row]] = augmented[row][free_count];
            }
        }

        for i in 0..count {
            let bound_tolerance = 1.0e-7
                * (1.0 + lower[i].abs() + if upper[i].is_finite() { upper[i].abs() } else { 0.0 });
            if value[i] < lower[i] - bound_tolerance
                || value[i] > upper[i] + bound_tolerance
                || !value[i].is_finite()
            {
                valid = false;
                break;
            }
            value[i] = value[i].clamp(lower[i], upper[i]);
        }
        if !valid {
            continue;
        }

        let mut objective = 0.0_f64;
        for i in 0..count {
            objective += regularized_linear[i] * value[i];
            for j in 0..count {
                objective += 0.5 * value[i] * hessian[i][j] * value[j];
            }
        }
        if !objective.is_finite() {
            continue;
        }

        let mut gradients = [0.0_f64; 4];
        for i in 0..count {
            gradients[i] = regularized_linear[i]
                + (0..count)
                    .map(|j| hessian[i][j] * value[j])
                    .sum::<f64>();
            let scale = 1.0
                + regularized_linear[i].abs()
                + (0..count)
                    .map(|j| (hessian[i][j] * value[j]).abs())
                    .sum::<f64>();
            let tolerance = 2.0e-7 * scale;
            let bound_tolerance = 1.0e-7
                * (1.0 + lower[i].abs() + if upper[i].is_finite() { upper[i].abs() } else { 0.0 });

            let fixed = (upper[i] - lower[i]).abs() <= bound_tolerance;
            let kkt_valid = if fixed {
                true
            } else {
                match state[i] {
                    0 => gradients[i] >= -tolerance,
                    1 => gradients[i].abs() <= 10.0 * tolerance,
                    2 => gradients[i] <= tolerance,
                    _ => false,
                }
            };
            let at_lower = (value[i] - lower[i]).abs() <= bound_tolerance;
            let at_upper = upper[i].is_finite()
                && (value[i] - upper[i]).abs() <= bound_tolerance;
            if !kkt_valid
                || (state[i] == 0 && !at_lower)
                || (state[i] == 2 && !at_upper)
                || (state[i] == 1 && (at_lower || at_upper))
            {
                valid = false;
                break;
            }
        }
        if !valid {
            continue;
        }

        if best.as_ref().is_none_or(|(_, best_objective)| objective < *best_objective) {
            best = Some((value, objective));
        }
    }

    if let Some((solution, _)) = best {
        return Some(solution);
    }

    // Rank-deficient contact patches have a non-unique impulse distribution.
    // Projected gradient uses the whole symmetric matrix at every iteration,
    // avoiding an arbitrary contact-point ordering as the tie-break.
    let row_sum = (0..count)
        .map(|i| (0..count).map(|j| hessian[i][j].abs()).sum::<f64>())
        .fold(0.0_f64, f64::max);
    if !row_sum.is_finite() || row_sum <= 1.0e-14 {
        return None;
    }

    let step = 0.95 / row_sum;
    let mut value = [0.0_f64; 4];
    for i in 0..count {
        value[i] = 0.0_f64.clamp(lower[i], upper[i]);
    }

    for _ in 0..512 {
        let mut next = value;
        let mut maximum_change = 0.0_f64;
        for i in 0..count {
            let gradient = regularized_linear[i]
                + (0..count)
                    .map(|j| hessian[i][j] * value[j])
                    .sum::<f64>();
            next[i] = (value[i] - step * gradient).clamp(lower[i], upper[i]);
            maximum_change = maximum_change.max((next[i] - value[i]).abs());
        }
        value = next;
        if maximum_change <= 1.0e-10 {
            break;
        }
    }

    value[..count].iter().all(|value| value.is_finite()).then_some(value)
}

/// Computes `DIM - 1` tangent directions.
#[allow(unused_variables)]
#[inline(always)]
fn compute_tangent_directions(
    normal: Vector,
    velocity1: Vector,
    velocity2: Vector,
) -> [Vector; DIM - 1] {
    #[cfg(feature = "2d")]
    {
        [Vector::new(normal.y, -normal.x)]
    }
    #[cfg(feature = "3d")]
    {
        let force_direction = -normal;
        let relative_velocity = velocity1 - velocity2;
        let tangent_velocity =
            relative_velocity - force_direction * force_direction.dot(relative_velocity);

        let tangent = tangent_velocity
            .try_normalize()
            .unwrap_or(force_direction.any_orthonormal_vector());
        let bitangent = force_direction.cross(tangent);
        [tangent, bitangent]
    }
}

#[cfg(test)]
mod material_warm_start_tests {
    use super::*;

    #[cfg(feature = "2d")]
    #[test]
    fn capped_2d_friction_warm_start_respects_applied_normal_impulse() {
        assert_eq!(warm_start_tangent_impulse(10.0, 2.0, 0.5, 4.0, true), 2.0);
        assert_eq!(warm_start_tangent_impulse(-10.0, 2.0, 0.5, 4.0, true), -2.0);
        assert_eq!(warm_start_tangent_impulse(10.0, 2.0, 0.5, 0.0, true), 0.0);
        assert_eq!(warm_start_tangent_impulse(10.0, 2.0, 0.5, 4.0, false), 20.0);
    }

    #[cfg(feature = "3d")]
    #[test]
    fn capped_3d_friction_warm_start_respects_applied_normal_impulse() {
        let cached = bevy_math::Vec2::new(3.0, 4.0);
        let capped = warm_start_tangent_impulse(cached, 2.0, 0.5, 4.0, true);
        assert!((capped.length() - 2.0).abs() < 1.0e-5);
        assert_eq!(
            warm_start_tangent_impulse(cached, 2.0, 0.5, 0.0, true),
            bevy_math::Vec2::ZERO
        );
        assert_eq!(
            warm_start_tangent_impulse(cached, 2.0, 0.5, 4.0, false),
            cached * 2.0
        );
    }
}
