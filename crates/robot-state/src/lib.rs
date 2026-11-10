//! In-process shared state with separate state cells and narrow writer handles.
//! The snapshot version is a seqlock-style consistency marker: readers retry
//! when any state component changes during their read.

use robot_application::ports::outbound::{
    ControlStateWriter, RobotMotionStateWriter, RobotStateReader, SafetyStateWriter,
};
use robot_domain::{
    ControlMode, ControlState, MotionState, RobotState, SafetyState, StateSnapshot,
};
use std::sync::{
    Arc, Mutex, RwLock,
    atomic::{AtomicU64, Ordering},
};

#[derive(Default)]
struct SharedState {
    version: AtomicU64,
    writer: Mutex<()>,
    robot: RwLock<RobotState>,
    control: RwLock<ControlState>,
    safety: RwLock<SafetyState>,
}

impl SharedState {
    fn mutate<T>(&self, f: impl FnOnce() -> T) -> T {
        let _writer = self
            .writer
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        self.version.fetch_add(1, Ordering::AcqRel);
        let result = f();
        self.version.fetch_add(1, Ordering::Release);
        result
    }

    fn snapshot(&self) -> StateSnapshot {
        loop {
            let before = self.version.load(Ordering::Acquire);
            if before & 1 == 1 {
                std::thread::yield_now();
                continue;
            }

            let robot = *self
                .robot
                .read()
                .unwrap_or_else(std::sync::PoisonError::into_inner);
            let control = *self
                .control
                .read()
                .unwrap_or_else(std::sync::PoisonError::into_inner);
            let safety = self
                .safety
                .read()
                .unwrap_or_else(std::sync::PoisonError::into_inner)
                .clone();
            let after = self.version.load(Ordering::Acquire);

            if before == after {
                return StateSnapshot {
                    version: after / 2,
                    robot,
                    control,
                    safety,
                };
            }
        }
    }
}

pub struct RobotStateStore {
    shared: Arc<SharedState>,
}

impl Default for RobotStateStore {
    fn default() -> Self {
        Self::new()
    }
}

impl RobotStateStore {
    pub fn new() -> Self {
        Self {
            shared: Arc::new(SharedState::default()),
        }
    }

    pub fn reader(&self) -> StateReaderHandle {
        StateReaderHandle(Arc::clone(&self.shared))
    }

    pub fn motion_writer(&self) -> MotionStateWriterHandle {
        MotionStateWriterHandle(Arc::clone(&self.shared))
    }

    pub fn control_writer(&self) -> ControlStateWriterHandle {
        ControlStateWriterHandle(Arc::clone(&self.shared))
    }

    pub fn safety_writer(&self) -> SafetyStateWriterHandle {
        SafetyStateWriterHandle(Arc::clone(&self.shared))
    }
}

#[derive(Clone)]
pub struct StateReaderHandle(Arc<SharedState>);

impl RobotStateReader for StateReaderHandle {
    fn snapshot(&self) -> StateSnapshot {
        self.0.snapshot()
    }
}

#[derive(Clone)]
pub struct MotionStateWriterHandle(Arc<SharedState>);

impl RobotMotionStateWriter for MotionStateWriterHandle {
    fn report_motion_if_safety_clear(&self, motion: MotionState, generation: u64) -> bool {
        self.0.mutate(|| {
            let safety = self
                .0
                .safety
                .read()
                .unwrap_or_else(std::sync::PoisonError::into_inner);
            if safety.latched || safety.generation != generation {
                return false;
            }
            drop(safety);
            self.0
                .robot
                .write()
                .unwrap_or_else(std::sync::PoisonError::into_inner)
                .motion = motion;
            true
        })
    }

    fn report_motion_if_safety_latched(&self, motion: MotionState, generation: u64) -> bool {
        self.0.mutate(|| {
            let safety = self
                .0
                .safety
                .read()
                .unwrap_or_else(std::sync::PoisonError::into_inner);
            if !safety.latched || safety.generation != generation {
                return false;
            }
            drop(safety);
            self.0
                .robot
                .write()
                .unwrap_or_else(std::sync::PoisonError::into_inner)
                .motion = motion;
            true
        })
    }
}

#[derive(Clone)]
pub struct ControlStateWriterHandle(Arc<SharedState>);

impl ControlStateWriter for ControlStateWriterHandle {
    fn compare_and_set(&self, expected: ControlState, next: ControlState) -> bool {
        self.0.mutate(|| {
            let mut state = self
                .0
                .control
                .write()
                .unwrap_or_else(std::sync::PoisonError::into_inner);
            if *state == expected {
                *state = next;
                true
            } else {
                false
            }
        })
    }

    fn compare_and_set_if_safety_clear(
        &self,
        expected: ControlState,
        next: ControlState,
        safety_generation: u64,
    ) -> bool {
        self.0.mutate(|| {
            let safety = self
                .0
                .safety
                .read()
                .unwrap_or_else(std::sync::PoisonError::into_inner);
            if safety.latched || safety.generation != safety_generation {
                return false;
            }
            drop(safety);
            let mut state = self
                .0
                .control
                .write()
                .unwrap_or_else(std::sync::PoisonError::into_inner);
            if *state == expected {
                *state = next;
                true
            } else {
                false
            }
        })
    }

    fn force_safe_stop(&self) {
        self.0.mutate(|| {
            *self
                .0
                .control
                .write()
                .unwrap_or_else(std::sync::PoisonError::into_inner) = ControlState {
                mode: ControlMode::SafeStop,
                active_lease: None,
            };
        });
    }
}

#[derive(Clone)]
pub struct SafetyStateWriterHandle(Arc<SharedState>);

impl SafetyStateWriter for SafetyStateWriterHandle {
    fn latch(&self, reason: String) -> SafetyState {
        self.0.mutate(|| {
            let mut state = self
                .0
                .safety
                .write()
                .unwrap_or_else(std::sync::PoisonError::into_inner);
            state.generation = state.generation.wrapping_add(1);
            state.reason = Some(reason);
            state.latched = true;
            state.clone()
        })
    }

    fn clear_if_generation(&self, generation: u64) -> bool {
        self.0.mutate(|| {
            let mut state = self
                .0
                .safety
                .write()
                .unwrap_or_else(std::sync::PoisonError::into_inner);
            if state.latched && state.generation == generation {
                state.latched = false;
                state.reason = None;
                true
            } else {
                false
            }
        })
    }
}
