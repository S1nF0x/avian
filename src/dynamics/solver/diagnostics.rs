use bevy::{
    diagnostic::DiagnosticPath,
    prelude::{ReflectResource, Resource},
    reflect::Reflect,
};
use core::time::Duration;

use crate::diagnostics::{PhysicsDiagnostics, impl_diagnostic_paths};

/// Diagnostics for the physics solver.
#[derive(Resource, Debug, Default, Reflect)]
#[reflect(Resource, Debug)]
pub struct SolverDiagnostics {
    /// Time spent preparing constraints.
    pub prepare_constraints: Duration,
    /// Time spent preparing or clearing velocity increments in [`VelocityIntegrationData`]s.
    ///
    /// [`VelocityIntegrationData`]: crate::dynamics::integrator::VelocityIntegrationData
    pub update_velocity_increments: Duration,
    /// Time spent integrating velocities.
    pub integrate_velocities: Duration,
    /// Time spent warm starting the solver.
    pub warm_start: Duration,
    /// Time spent solving constraints with bias.
    pub solve_constraints: Duration,
    /// Time spent integrating positions.
    pub integrate_positions: Duration,
    /// Time spent relaxing velocities.
    pub relax_velocities: Duration,
    /// Time spent applying restitution.
    pub apply_restitution: Duration,
    /// Time spent writing the final results to the bodies.
    pub finalize: Duration,
    /// Time spent storing impulses for warm starting.
    pub store_impulses: Duration,
    /// Time spent on swept CCD.
    pub swept_ccd: Duration,
    /// The number of contact constraints generated.
    pub contact_constraint_count: u32,
    /// The number of contact capacity failures reported this physics step.
    pub contact_break_event_count: u32,
    /// The number of bounded same-substep contact re-solves completed.
    pub contact_re_solve_count: u32,
    /// Time spent rebuilding and re-solving breakage contacts.
    pub contact_re_solve: Duration,
    /// The number of re-solve requests dropped after reaching the pass limit.
    pub contact_re_solve_dropped: u32,
}

impl PhysicsDiagnostics for SolverDiagnostics {
    fn timer_paths(&self) -> Vec<(&'static DiagnosticPath, Duration)> {
        vec![
            (Self::PREPARE_CONSTRAINTS, self.prepare_constraints),
            (
                Self::UPDATE_VELOCITY_INCREMENTS,
                self.update_velocity_increments,
            ),
            (Self::INTEGRATE_VELOCITIES, self.integrate_velocities),
            (Self::WARM_START, self.warm_start),
            (Self::SOLVE_CONSTRAINTS, self.solve_constraints),
            (Self::CONTACT_RE_SOLVE, self.contact_re_solve),
            (Self::INTEGRATE_POSITIONS, self.integrate_positions),
            (Self::RELAX_VELOCITIES, self.relax_velocities),
            (Self::APPLY_RESTITUTION, self.apply_restitution),
            (Self::FINALIZE, self.finalize),
            (Self::STORE_IMPULSES, self.store_impulses),
            (Self::SWEPT_CCD, self.swept_ccd),
        ]
    }

    fn counter_paths(&self) -> Vec<(&'static DiagnosticPath, u32)> {
        vec![(
            Self::CONTACT_CONSTRAINT_COUNT,
            self.contact_constraint_count,
        ),
        (
            Self::CONTACT_BREAK_EVENT_COUNT,
            self.contact_break_event_count,
        ),
        (Self::CONTACT_RE_SOLVE_COUNT, self.contact_re_solve_count),
        (
            Self::CONTACT_RE_SOLVE_DROPPED,
            self.contact_re_solve_dropped,
        )]
    }
}

impl_diagnostic_paths! {
    impl SolverDiagnostics {
        PREPARE_CONSTRAINTS: "avian/solver/prepare_constraints",
        UPDATE_VELOCITY_INCREMENTS: "avian/solver/update_velocity_increments",
        INTEGRATE_VELOCITIES: "avian/solver/integrate_velocities",
        WARM_START: "avian/solver/warm_start",
        SOLVE_CONSTRAINTS: "avian/solver/solve_constraints",
        INTEGRATE_POSITIONS: "avian/solver/integrate_positions",
        RELAX_VELOCITIES: "avian/solver/relax_velocities",
        APPLY_RESTITUTION: "avian/solver/apply_restitution",
        FINALIZE: "avian/solver/finalize",
        STORE_IMPULSES: "avian/solver/store_impulses",
        SWEPT_CCD: "avian/solver/swept_ccd",
        CONTACT_CONSTRAINT_COUNT: "avian/solver/contact_constraint_count",
        CONTACT_BREAK_EVENT_COUNT: "avian/solver/contact_break_event_count",
        CONTACT_RE_SOLVE_COUNT: "avian/solver/contact_re_solve_count",
        CONTACT_RE_SOLVE: "avian/solver/contact_re_solve",
        CONTACT_RE_SOLVE_DROPPED: "avian/solver/contact_re_solve_dropped",
    }
}
