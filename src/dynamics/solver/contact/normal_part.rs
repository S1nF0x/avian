use crate::{dynamics::solver::softness_parameters::SoftnessCoefficients, prelude::*};
use bevy::reflect::Reflect;
#[cfg(feature = "serialize")]
use bevy::reflect::{ReflectDeserialize, ReflectSerialize};

// TODO: Block solver for solving two contact points simultaneously
// TODO: One-body constraint version
/// The normal part of a [`ContactConstraintPoint`](super::ContactConstraintPoint).
/// Aims to resolve overlap.
#[derive(Clone, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct ContactNormalPart {
    /// The magnitude of the contact impulse along the contact normal.
    pub impulse: f32,

    /// Maximum accumulated normal impulse for the current substep.
    pub max_impulse: f32,

    /// Set when the requested normal impulse exceeds the material capacity.
    pub capacity_exceeded: bool,

    /// The total normal impulse applied across substeps and restitution.
    /// The unit is typically N⋅s or kg⋅m/s.
    ///
    /// This is used by the solver to identify speculative contacts that did not generate
    /// an impulse, but is also useful for users to determine how "strong" the contact is.
    /// This is stored for contact manifolds as [`ContactPoint::normal_impulse`].
    pub total_impulse: f32,

    /// The inertial properties of the bodies projected onto the contact normal,
    /// or in other words, the mass "seen" by the constraint along the normal.
    pub effective_mass: f32,

    /// The softness parameters used for tuning contact response.
    pub softness: SoftnessCoefficients,
}

impl ContactNormalPart {
    /// Generates a new [`ContactNormalPart`].
    #[allow(clippy::too_many_arguments)]
    pub fn generate(
        effective_inverse_mass_sum: Vector,
        inverse_angular_inertia1: &SymmetricTensor,
        inverse_angular_inertia2: &SymmetricTensor,
        r1: Vector,
        r2: Vector,
        normal: Vector,
        warm_start_impulse: Option<f32>,
        softness: SoftnessCoefficients,
    ) -> Self {
        let i1 = inverse_angular_inertia1;
        let i2 = inverse_angular_inertia2;

        // Derivation for the projected normal mass. This is for 3D, but the 2D version is basically equivalent.
        //
        // The penetration constraint function is the following:
        //
        // C(s) = dot(p2 - p1, n) = dot(x2 + r2 - x1 - r1, n)
        //
        // where
        // - p1 and p2 are world-space contact points for each body
        // - n is the surface normal pointing from the first body towards the second (the order matters)
        // - x1 and x2 are the centers of mass
        // - r1 and r2 are vectors from the centers of mass to the corresponding contact points
        //
        // The contact constraint is satisfied when the bodies are not penetrating:
        //
        // C(s) >= 0
        //
        // We can compute the velocity constraint by getting the time derivative:
        //
        // C_vel(s) = d/dt(dot(x2 + r2 - x1 - r1, n))
        //          = dot(d/dt(x2 + r2 - x1 - r1), n) + dot(x2 + r2 - x1 - r1, d/dt(n))
        //
        // The penetration is assumed to be small, so we can ignore the second term:
        //
        // C_vel(s) = dot(d/dt(x2 + r2 - x1 - r1), n)
        //          = dot(lin_vel2 + ang_vel2 x r2 - lin_vel1 - ang_vel1 x r1, n)
        //          = dot(lin_vel2, n) + dot(ang_vel2, r2 x n) - dot(v1, n) - dot(ang_vel1, r1 x n)
        //
        // By inspection, we can see that the Jacobian is the following:
        //
        //      linear1  angular1 linear2 angular2
        // J = [ -normal, -(r1 x n), n, r2 x n ]
        //
        // From this, we can derive the effective inverse mass:
        //
        // K = J_x * M^-1 * J_x^T
        //   = 1/m1 + 1/m2 + (r1 x n)^T * I1^-1 * (r1 x n) + (r2 x n)^T * I2^-1 * (r2 x n)
        //
        // See "Constraints Derivation for Rigid Body Simulation in 3D" section 2.1.3
        // by Daniel Chappuis for the full derivation of the effective inverse mass.
        //
        // Finally, the transposes can be simplified with dot products, because a^T * b = dot(a, b),
        // where a and b are two column vectors.
        //
        // K = 1/m1 + 1/m2 + dot(r1 x n, I1^-1 * (r1 x n)) + dot(r2 x n, I2^-1 * (r2 x n))

        let r1_cross_n = cross(r1, normal);
        let r2_cross_n = cross(r2, normal);

        let k_linear = normal.dot(effective_inverse_mass_sum * normal);
        #[cfg(feature = "2d")]
        let k = k_linear + i1 * r1_cross_n * r1_cross_n + i2 * r2_cross_n * r2_cross_n;
        #[cfg(feature = "3d")]
        let k = k_linear + r1_cross_n.dot(i1 * r1_cross_n) + r2_cross_n.dot(i2 * r2_cross_n);

        Self {
            impulse: warm_start_impulse.unwrap_or_default(),
            max_impulse: f32::INFINITY,
            capacity_exceeded: false,
            total_impulse: 0.0,
            effective_mass: k.recip_or_zero(),
            softness,
        }
    }

    /// Sets the maximum normal impulse for this contact point.
    ///
    /// This only limits the solver impulse; it does not change effective mass.
    pub fn set_maximum_normal_impulse(&mut self, max_impulse_ns: f32) {
        self.max_impulse = if max_impulse_ns.is_finite() {
            max_impulse_ns.max(0.0)
        } else {
            f32::INFINITY
        };
        self.impulse = self.impulse.clamp(0.0, self.max_impulse);
    }

    /// Returns the cached normal impulse used for warm starting, limited by material capacity.
    #[inline]
    pub fn warm_start_impulse(&self, coefficient: f32) -> f32 {
        let impulse = coefficient * self.impulse;
        if self.max_impulse.is_finite() {
            impulse.clamp(0.0, self.max_impulse)
        } else {
            impulse
        }
    }

    /// Clears transient failure state at the start of a substep.
    #[inline]
    pub(crate) fn reset_substep_state(&mut self) {
        self.capacity_exceeded = false;
    }

    /// Clamps a requested accumulated impulse and returns the actual increment.
    #[inline]
    pub fn clamp_accumulated_impulse(&mut self, requested_impulse: f32) -> f32 {
        if self.capacity_exceeded {
            return 0.0;
        }
        let requested = requested_impulse.max(0.0);
        // Reaching the exact material capacity is failure onset too. A
        // one-substep solve can land exactly on the cap, whereas additional
        // substeps may request a larger impulse on the following iteration.
        // Report the same material state in either case.
        if self.max_impulse.is_finite()
            && requested >= self.max_impulse
            && requested > self.impulse
        {
            self.capacity_exceeded = true;
        }
        let new_impulse = requested.min(self.max_impulse);
        let applied = new_impulse - self.impulse;
        self.impulse = new_impulse;
        applied
    }

    /// Solves the non-penetration constraint, updating the total impulse in `self` and returning
    /// the incremental impulse to apply to each body.
    pub fn solve_impulse<const USE_BIAS: bool>(
        &mut self,
        separation: f32,
        relative_velocity: Vector,
        normal: Vector,
        max_overlap_solve_speed: f32,
        delta_secs: f32,
    ) -> f32 {
        // Compute the relative velocity along the normal.
        let normal_speed = relative_velocity.dot(normal);

        // Compute the incremental normal impulse.
        let mut impulse = if separation > 0.0 {
            // Speculative contact: Push back the part of the velocity that would cause penetration.
            -self.effective_mass * (normal_speed + separation / delta_secs)
        } else if USE_BIAS {
            // Contact using bias: Incorporate softness parameters.
            //
            // 1. Velocity bias: Allows the constraint to solve overlap by boosting
            //    the constraint response a bit, taking into account the current overlap.
            //    This is known as Baumgarte stabilization.
            //
            // 2. Mass coefficient: Scales the effective mass "seen" by the constraint.
            //
            // 3. Impulse coefficient: Scales the accumulated impulse that is subtracted
            //    from the total impulse to prevent the total impulse from becoming too large.
            //
            // See the `softness_parameters` module for more details and references.

            // TODO: We might want optional slop, a small amount of allowed penetration.
            let bias = (self.softness.bias * separation).max(-max_overlap_solve_speed);
            let scaled_mass = self.softness.mass_scale * self.effective_mass;
            let scaled_impulse = self.softness.impulse_scale * self.impulse;

            -scaled_mass * (normal_speed + bias) - scaled_impulse
        } else {
            // Contact without bias: Solve normally without softness parameters.
            // This is useful for a "relaxation" phase which helps get rid of overshoot.
            -self.effective_mass * normal_speed
        };

        // Clamp the accumulated impulse to the material capacity.
        impulse = self.clamp_accumulated_impulse(self.impulse + impulse);
        self.total_impulse += impulse;

        // Return the clamped incremental normal impulse.
        impulse
    }
}

#[cfg(test)]
mod material_capacity_tests {
    use super::*;

    fn normal_part() -> ContactNormalPart {
        ContactNormalPart {
            impulse: 0.0,
            max_impulse: f32::INFINITY,
            capacity_exceeded: false,
            total_impulse: 0.0,
            effective_mass: 1.0,
            softness: SoftnessCoefficients {
                bias: 0.0,
                mass_scale: 1.0,
                impulse_scale: 0.0,
            },
        }
    }

    #[test]
    fn reaching_material_yield_impulse_is_not_substep_dependent() {
        let mut at_limit = normal_part();
        at_limit.set_maximum_normal_impulse(10.0);
        assert_eq!(at_limit.clamp_accumulated_impulse(10.0), 10.0);
        assert!(at_limit.capacity_exceeded);

        at_limit.reset_substep_state();
        at_limit.impulse = 0.0;
        assert_eq!(at_limit.clamp_accumulated_impulse(9.0), 9.0);
        assert!(!at_limit.capacity_exceeded);

        // Cached capacity with zero additional load is not a new failure.
        at_limit.impulse = 10.0;
        at_limit.reset_substep_state();
        assert_eq!(at_limit.clamp_accumulated_impulse(10.0), 0.0);
        assert!(!at_limit.capacity_exceeded);
        assert_eq!(at_limit.clamp_accumulated_impulse(11.0), 0.0);
        assert!(at_limit.capacity_exceeded);

        let mut ordinary_contact = normal_part();
        assert_eq!(ordinary_contact.clamp_accumulated_impulse(10.0), 10.0);
        assert!(!ordinary_contact.capacity_exceeded);
    }
}
