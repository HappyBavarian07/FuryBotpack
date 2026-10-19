use crate::{
    Arena, BallState, BoostPadState, CarBodyConfig, CarControls, CarState, GameMode,
    PhysState, RaycastHitInfo, Team, consts, init_from_default,
};
use glam::{Mat3A, Vec3A};
use std::cell::RefCell;

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct RCCarControls {
    pub throttle: f32,
    pub steer: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub roll: f32,
    pub jump: bool,
    pub boost: bool,
    pub handbrake: bool,
}

impl From<CarControls> for RCCarControls {
    fn from(controls: CarControls) -> Self {
        Self {
            throttle: controls.throttle,
            steer: controls.steer,
            pitch: controls.pitch,
            yaw: controls.yaw,
            roll: controls.roll,
            jump: controls.jump,
            boost: controls.boost,
            handbrake: controls.handbrake,
        }
    }
}

impl From<RCCarControls> for CarControls {
    fn from(controls: RCCarControls) -> Self {
        Self {
            throttle: controls.throttle,
            steer: controls.steer,
            pitch: controls.pitch,
            yaw: controls.yaw,
            roll: controls.roll,
            jump: controls.jump,
            boost: controls.boost,
            handbrake: controls.handbrake,
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct RCCarState {
    pub pos_x: f32, pub pos_y: f32, pub pos_z: f32,
    pub rot_forward_x: f32, pub rot_forward_y: f32, pub rot_forward_z: f32,
    pub rot_right_x: f32, pub rot_right_y: f32, pub rot_right_z: f32,
    pub rot_up_x: f32, pub rot_up_y: f32, pub rot_up_z: f32,
    pub vel_x: f32, pub vel_y: f32, pub vel_z: f32,
    pub ang_vel_x: f32, pub ang_vel_y: f32, pub ang_vel_z: f32,

    pub is_on_ground: bool,
    pub wheels_with_contact: [bool; 4],
    pub has_jumped: bool,
    pub has_double_jumped: bool,
    pub has_flipped: bool,
    pub flip_rel_torque_x: f32, pub flip_rel_torque_y: f32, pub flip_rel_torque_z: f32,
    pub jump_time: f32,
    pub flip_time: f32,
    pub is_flipping: bool,
    pub is_jumping: bool,
    pub air_time: f32,
    pub air_time_since_jump: f32,
    pub boost: f32,
    pub time_since_boosted: f32,
    pub is_boosting: bool,
    pub boosting_time: f32,
    pub is_supersonic: bool,
    pub supersonic_time: f32,
    pub handbrake_val: f32,
    pub is_auto_flipping: bool,
    pub auto_flip_timer: f32,
    pub auto_flip_torque_scale: f32,
    pub bump_cooldown_timer: f32,
    pub world_contact_normal_has_value: bool,
    pub world_contact_normal_x: f32, pub world_contact_normal_y: f32, pub world_contact_normal_z: f32,
    pub is_demoed: bool,
    pub demo_respawn_timer: f32,
}

impl From<CarState> for RCCarState {
    fn from(state: CarState) -> Self {
        let (wc_has_val, wc_x, wc_y, wc_z) = match state.world_contact_normal {
            Some(n) => (true, n.x, n.y, n.z),
            None => (false, 0.0, 0.0, 0.0),
        };
        Self {
            pos_x: state.phys.pos.x,
            pos_y: state.phys.pos.y,
            pos_z: state.phys.pos.z,
            rot_forward_x: state.phys.rot_mat.x_axis.x,
            rot_forward_y: state.phys.rot_mat.x_axis.y,
            rot_forward_z: state.phys.rot_mat.x_axis.z,
            rot_right_x: state.phys.rot_mat.y_axis.x,
            rot_right_y: state.phys.rot_mat.y_axis.y,
            rot_right_z: state.phys.rot_mat.y_axis.z,
            rot_up_x: state.phys.rot_mat.z_axis.x,
            rot_up_y: state.phys.rot_mat.z_axis.y,
            rot_up_z: state.phys.rot_mat.z_axis.z,
            vel_x: state.phys.vel.x,
            vel_y: state.phys.vel.y,
            vel_z: state.phys.vel.z,
            ang_vel_x: state.phys.ang_vel.x,
            ang_vel_y: state.phys.ang_vel.y,
            ang_vel_z: state.phys.ang_vel.z,

            is_on_ground: state.is_on_ground,
            wheels_with_contact: state.wheels_with_contact.map(|hit| hit.is_some()),
            has_jumped: state.has_jumped,
            has_double_jumped: state.has_double_jumped,
            has_flipped: state.has_flipped,
            flip_rel_torque_x: state.flip_rel_torque.x,
            flip_rel_torque_y: state.flip_rel_torque.y,
            flip_rel_torque_z: state.flip_rel_torque.z,
            jump_time: state.jump_time(),
            flip_time: state.flip_time,
            is_flipping: state.is_flipping,
            is_jumping: state.is_jumping,
            air_time: state.air_time,
            air_time_since_jump: state.air_time_since_jump,
            boost: state.boost,
            time_since_boosted: state.time_since_boosted,
            is_boosting: state.is_boosting,
            boosting_time: state.boosting_time,
            is_supersonic: state.is_supersonic,
            supersonic_time: state.supersonic_grace_timer,
            handbrake_val: state.handbrake_val,
            is_auto_flipping: state.is_auto_flipping,
            auto_flip_timer: state.auto_flip_timer,
            auto_flip_torque_scale: state.auto_flip_torque_scale,
            bump_cooldown_timer: state.bump_cooldown_timer,
            world_contact_normal_has_value: wc_has_val,
            world_contact_normal_x: wc_x,
            world_contact_normal_y: wc_y,
            world_contact_normal_z: wc_z,
            is_demoed: state.is_demoed,
            demo_respawn_timer: state.demo_respawn_timer,
        }
    }
}

impl From<RCCarState> for CarState {
    fn from(state: RCCarState) -> Self {
        Self {
            phys: PhysState {
                pos: Vec3A::new(state.pos_x, state.pos_y, state.pos_z),
                rot_mat: Mat3A::from_cols(
                    Vec3A::new(state.rot_forward_x, state.rot_forward_y, state.rot_forward_z),
                    Vec3A::new(state.rot_right_x, state.rot_right_y, state.rot_right_z),
                    Vec3A::new(state.rot_up_x, state.rot_up_y, state.rot_up_z),
                ),
                vel: Vec3A::new(state.vel_x, state.vel_y, state.vel_z),
                ang_vel: Vec3A::new(state.ang_vel_x, state.ang_vel_y, state.ang_vel_z),
            },
            controls: CarControls::DEFAULT,
            prev_controls: CarControls::DEFAULT,
            is_on_ground: state.is_on_ground,
            wheels_with_contact: state.wheels_with_contact.map(|contact| contact.then_some(RaycastHitInfo::default())),
            has_jumped: state.has_jumped,
            has_double_jumped: state.has_double_jumped,
            has_flipped: state.has_flipped,
            flip_rel_torque: Vec3A::new(
                state.flip_rel_torque_x,
                state.flip_rel_torque_y,
                state.flip_rel_torque_z,
            ),
            jump_ticks: (state.jump_time / consts::TICK_TIME).round() as u32,
            flip_time: state.flip_time,
            is_flipping: state.is_flipping,
            is_jumping: state.is_jumping,
            air_time: state.air_time,
            air_time_since_jump: state.air_time_since_jump,
            boost: state.boost,
            time_since_boosted: state.time_since_boosted,
            is_boosting: state.is_boosting,
            boosting_time: state.boosting_time,
            is_supersonic: state.is_supersonic,
            supersonic_grace_timer: state.supersonic_time,
            handbrake_val: state.handbrake_val,
            is_auto_flipping: state.is_auto_flipping,
            auto_flip_timer: state.auto_flip_timer,
            auto_flip_torque_scale: state.auto_flip_torque_scale,
            bump_cooldown_timer: state.bump_cooldown_timer,
            last_bump_victim: None,
            last_extra_hit_tick: None,
            world_contact_normal: if state.world_contact_normal_has_value {
                Some(Vec3A::new(
                    state.world_contact_normal_x,
                    state.world_contact_normal_y,
                    state.world_contact_normal_z,
                ))
            } else {
                None
            },
            is_demoed: state.is_demoed,
            demo_respawn_timer: state.demo_respawn_timer,
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct RCBallState {
    pub pos_x: f32, pub pos_y: f32, pub pos_z: f32,
    pub rot_forward_x: f32, pub rot_forward_y: f32, pub rot_forward_z: f32,
    pub rot_right_x: f32, pub rot_right_y: f32, pub rot_right_z: f32,
    pub rot_up_x: f32, pub rot_up_y: f32, pub rot_up_z: f32,
    pub vel_x: f32, pub vel_y: f32, pub vel_z: f32,
    pub ang_vel_x: f32, pub ang_vel_y: f32, pub ang_vel_z: f32,
}

impl From<BallState> for RCBallState {
    fn from(state: BallState) -> Self {
        Self {
            pos_x: state.phys.pos.x,
            pos_y: state.phys.pos.y,
            pos_z: state.phys.pos.z,
            rot_forward_x: state.phys.rot_mat.x_axis.x,
            rot_forward_y: state.phys.rot_mat.x_axis.y,
            rot_forward_z: state.phys.rot_mat.x_axis.z,
            rot_right_x: state.phys.rot_mat.y_axis.x,
            rot_right_y: state.phys.rot_mat.y_axis.y,
            rot_right_z: state.phys.rot_mat.y_axis.z,
            rot_up_x: state.phys.rot_mat.z_axis.x,
            rot_up_y: state.phys.rot_mat.z_axis.y,
            rot_up_z: state.phys.rot_mat.z_axis.z,
            vel_x: state.phys.vel.x,
            vel_y: state.phys.vel.y,
            vel_z: state.phys.vel.z,
            ang_vel_x: state.phys.ang_vel.x,
            ang_vel_y: state.phys.ang_vel.y,
            ang_vel_z: state.phys.ang_vel.z,
        }
    }
}

impl From<RCBallState> for BallState {
    fn from(state: RCBallState) -> Self {
        let mut bs = BallState::DEFAULT;
        bs.phys = PhysState {
            pos: Vec3A::new(state.pos_x, state.pos_y, state.pos_z),
            rot_mat: Mat3A::from_cols(
                Vec3A::new(state.rot_forward_x, state.rot_forward_y, state.rot_forward_z),
                Vec3A::new(state.rot_right_x, state.rot_right_y, state.rot_right_z),
                Vec3A::new(state.rot_up_x, state.rot_up_y, state.rot_up_z),
            ),
            vel: Vec3A::new(state.vel_x, state.vel_y, state.vel_z),
            ang_vel: Vec3A::new(state.ang_vel_x, state.ang_vel_y, state.ang_vel_z),
        };
        bs
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct RCBoostPadState {
    pub is_active: bool,
    pub cooldown: f32,
}

impl From<BoostPadState> for RCBoostPadState {
    fn from(state: BoostPadState) -> Self {
        Self {
            is_active: state.is_active(),
            cooldown: state.cooldown,
        }
    }
}

impl From<RCBoostPadState> for BoostPadState {
    fn from(state: RCBoostPadState) -> Self {
        Self {
            cooldown: state.cooldown,
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rocketsim_set_version(version: i32) {
    // SAFETY: called during single-threaded FFI setup before any Arena stepping.
    unsafe {
        if version == 2 {
            std::env::set_var("RS_TUNE_SUSP_SUBTRACT", "0.05");
            std::env::set_var("RS_TUNE_SUSP_RELVEL_LIVE", "0");
            std::env::set_var("RS_TUNE_PUSHBACK_AT_APPLY", "0");
            std::env::set_var("RS_TUNE_FRICTION_AT_APPLY", "0");
            std::env::set_var("RS_TUNE_STEER_AT_APPLY", "0");
        } else {
            std::env::set_var("RS_TUNE_SUSP_SUBTRACT", "0.0");
            std::env::set_var("RS_TUNE_SUSP_RELVEL_LIVE", "1");
            std::env::set_var("RS_TUNE_PUSHBACK_AT_APPLY", "1");
            std::env::set_var("RS_TUNE_FRICTION_AT_APPLY", "1");
            std::env::set_var("RS_TUNE_STEER_AT_APPLY", "1");
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rocketsim_init(silent: bool) -> bool {
    init_from_default(silent).is_ok()
}

#[unsafe(no_mangle)]
pub extern "C" fn arena_create(game_mode: i32) -> *mut Arena {
    let mode = match game_mode {
        0 => GameMode::Soccar,
        1 => GameMode::Hoops,
        2 => GameMode::Heatseeker,
        3 => GameMode::Snowday,
        4 => GameMode::Dropshot,
        _ => GameMode::TheVoid,
    };
    let arena = Arena::new(mode);
    Box::into_raw(Box::new(arena))
}

#[unsafe(no_mangle)]
pub extern "C" fn arena_destroy(arena: *mut Arena) {
    if !arena.is_null() {
        unsafe {
            let _ = Box::from_raw(arena);
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn arena_add_car(arena: *mut Arena, team: i32, car_body_config: i32) -> usize {
    let t = match team {
        0 => Team::Blue,
        _ => Team::Orange,
    };
    let config = match car_body_config {
        0 => CarBodyConfig::OCTANE,
        1 => CarBodyConfig::DOMINUS,
        2 => CarBodyConfig::PLANK,
        3 => CarBodyConfig::BREAKOUT,
        4 => CarBodyConfig::HYBRID,
        5 => CarBodyConfig::MERC,
        _ => CarBodyConfig::PSYCLOPS,
    };
    unsafe { (*arena).add_car(t, config) }
}

#[unsafe(no_mangle)]
pub extern "C" fn arena_step_tick(arena: *mut Arena) {
    unsafe {
        STEP_EVENTS_BUF.with(|buf| {
            let mut b = buf.borrow_mut();
            b.clear();
            let events = (*arena).step_tick();
            record_events(&mut b, events, 0);
        });
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn arena_step(arena: *mut Arena, ticks: u32) {
    unsafe {
        STEP_EVENTS_BUF.with(|buf| {
            let mut b = buf.borrow_mut();
            b.clear();
            for t in 0..ticks {
                let events = (*arena).step_tick();
                record_events(&mut b, events, t);
            }
        });
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn arena_reset_to_random_kickoff(arena: *mut Arena, seed: i64) {
    let rng_seed = if seed == -1 { None } else { Some(seed as u64) };
    unsafe {
        (*arena).reset_to_random_kickoff(rng_seed);
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn arena_get_car_state(arena: *const Arena, car_idx: usize, out_state: *mut RCCarState) {
    unsafe {
        let state = (*arena).get_car_state(car_idx);
        *out_state = RCCarState::from(*state);
    }
}

const LARGE_TELEPORT_UU: f32 = 500.0;

fn is_large_teleport(from: Vec3A, to: Vec3A) -> bool {
    from.distance_squared(to) > LARGE_TELEPORT_UU * LARGE_TELEPORT_UU
}

/// Sets a car state. Teleports beyond `LARGE_TELEPORT_UU` (kickoffs, demo respawns,
/// state setters) also drop cached contacts so the car and ball behave like fresh
/// instances; small per-tick corrections keep contact continuity.
#[unsafe(no_mangle)]
pub extern "C" fn arena_set_car_state(arena: *mut Arena, car_idx: usize, in_state: *const RCCarState) {
    unsafe {
        let arena = &mut *arena;
        let mut state = CarState::from(*in_state);
        let previous = arena.get_car_state(car_idx);
        state.last_extra_hit_tick = previous.last_extra_hit_tick;
        state.last_bump_victim = previous.last_bump_victim;
        let far = is_large_teleport(previous.phys.pos, state.phys.pos);
        arena.set_car_state(car_idx, state);
        if far {
            arena.reset_car_transient_contacts(car_idx);
            arena.clear_persistent_manifolds();
            arena.refresh_car_sticky_gate(car_idx);
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn arena_set_car_controls(arena: *mut Arena, car_idx: usize, in_controls: *const RCCarControls) {
    unsafe {
        let controls = CarControls::from(*in_controls);
        (*arena).set_car_controls(car_idx, controls);
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn arena_get_ball_state(arena: *const Arena, out_state: *mut RCBallState) {
    unsafe {
        let state = (*arena).get_ball_state();
        *out_state = RCBallState::from(*state);
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn arena_set_ball_state(arena: *mut Arena, in_state: *const RCBallState) {
    unsafe {
        let arena = &mut *arena;
        let state = BallState::from(*in_state);
        let far = is_large_teleport(arena.get_ball_state().phys.pos, state.phys.pos);
        arena.set_ball_state(state);
        if far {
            arena.clear_persistent_manifolds();
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn arena_num_boost_pads(arena: *const Arena) -> usize {
    unsafe { (*arena).num_boost_pads() }
}

#[unsafe(no_mangle)]
pub extern "C" fn arena_get_boost_pad_state(arena: *const Arena, pad_idx: usize, out_state: *mut RCBoostPadState) {
    unsafe {
        let state = (*arena).get_boost_pad_state(pad_idx);
        *out_state = RCBoostPadState::from(state);
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn arena_set_boost_pad_state(arena: *mut Arena, pad_idx: usize, in_state: *const RCBoostPadState) {
    unsafe {
        let state = BoostPadState::from(*in_state);
        (*arena).set_boost_pad_state(pad_idx, state);
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn arena_is_ball_scored(arena: *const Arena) -> bool {
    unsafe { (*arena).is_ball_scored() }
}

#[unsafe(no_mangle)]
pub extern "C" fn arena_get_boost_pad_config(
    arena: *const Arena,
    pad_idx: usize,
    out_x: *mut f32,
    out_y: *mut f32,
    out_z: *mut f32,
    out_is_big: *mut bool,
) {
    unsafe {
        let config = (*arena).get_boost_pad_config(pad_idx);
        *out_x = config.pos.x;
        *out_y = config.pos.y;
        *out_z = config.pos.z;
        *out_is_big = config.is_big;
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct RCCarHitBallInfo {
    pub contact_point_x: f32,
    pub contact_point_y: f32,
    pub contact_point_z: f32,
    pub extra_hit_vel_x: f32,
    pub extra_hit_vel_y: f32,
    pub extra_hit_vel_z: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct RCCarHitCarInfo {
    pub bumper_car_idx: usize,
    pub victim_car_idx: usize,
    pub contact_point_x: f32,
    pub contact_point_y: f32,
    pub contact_point_z: f32,
    pub is_demo: bool,
}

#[unsafe(no_mangle)]
pub extern "C" fn arena_get_car_hit_ball(
    arena: *const Arena,
    car_idx: usize,
    out_info: *mut RCCarHitBallInfo,
) -> bool {
    unsafe {
        for event in (*arena).get_last_step_events() {
            if let crate::ArenaEvent::CarHitBall(e) = event {
                if e.car_idx == car_idx {
                    (*out_info).contact_point_x = e.contact_point.x;
                    (*out_info).contact_point_y = e.contact_point.y;
                    (*out_info).contact_point_z = e.contact_point.z;
                    (*out_info).extra_hit_vel_x = e.extra_hit_vel.x;
                    (*out_info).extra_hit_vel_y = e.extra_hit_vel.y;
                    (*out_info).extra_hit_vel_z = e.extra_hit_vel.z;
                    return true;
                }
            }
        }
        false
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn arena_get_num_car_hit_car(arena: *const Arena) -> usize {
    unsafe {
        (*arena)
            .get_last_step_events()
            .iter()
            .filter(|e| matches!(e, crate::ArenaEvent::CarHitCar(_)))
            .count()
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn arena_get_car_hit_car(
    arena: *const Arena,
    event_idx: usize,
    out_info: *mut RCCarHitCarInfo,
) -> bool {
    unsafe {
        let mut idx = 0;
        for event in (*arena).get_last_step_events() {
            if let crate::ArenaEvent::CarHitCar(e) = event {
                if idx == event_idx {
                    (*out_info).bumper_car_idx = e.bumper_car_idx;
                    (*out_info).victim_car_idx = e.victim_car_idx;
                    (*out_info).contact_point_x = e.contact_point.x;
                    (*out_info).contact_point_y = e.contact_point.y;
                    (*out_info).contact_point_z = e.contact_point.z;
                    (*out_info).is_demo = e.is_demo;
                    return true;
                }
                idx += 1;
            }
        }
        false
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct RCStepEvent {
    pub kind: u32,
    pub tick_offset: u32,
    pub car_idx: u32,
    pub other_idx: u32,
    pub contact_point_x: f32,
    pub contact_point_y: f32,
    pub contact_point_z: f32,
    pub contact_normal_x: f32,
    pub contact_normal_y: f32,
    pub contact_normal_z: f32,
    pub extra_hit_vel_x: f32,
    pub extra_hit_vel_y: f32,
    pub extra_hit_vel_z: f32,
    pub is_demo: bool,
}

thread_local! {
    static STEP_EVENTS_BUF: RefCell<Vec<RCStepEvent>> = const { RefCell::new(Vec::new()) };
}

fn record_events(buf: &mut Vec<RCStepEvent>, events: &[crate::ArenaEvent], tick_offset: u32) {
    for event in events {
        match event {
            // The legacy C ABI has no landing event; contact booleans still report landing state.
            crate::ArenaEvent::CarLanded(_) => {}
            crate::ArenaEvent::BallHitWorld(e) => {
                buf.push(RCStepEvent {
                    kind: 0,
                    tick_offset,
                    car_idx: u32::MAX,
                    other_idx: u32::MAX,
                    contact_point_x: e.contact_point.x,
                    contact_point_y: e.contact_point.y,
                    contact_point_z: e.contact_point.z,
                    contact_normal_x: e.contact_normal.x,
                    contact_normal_y: e.contact_normal.y,
                    contact_normal_z: e.contact_normal.z,
                    extra_hit_vel_x: 0.0,
                    extra_hit_vel_y: 0.0,
                    extra_hit_vel_z: 0.0,
                    is_demo: false,
                });
            }
            crate::ArenaEvent::CarHitBall(e) => {
                buf.push(RCStepEvent {
                    kind: 1,
                    tick_offset,
                    car_idx: e.car_idx as u32,
                    other_idx: u32::MAX,
                    contact_point_x: e.contact_point.x,
                    contact_point_y: e.contact_point.y,
                    contact_point_z: e.contact_point.z,
                    contact_normal_x: 0.0,
                    contact_normal_y: 0.0,
                    contact_normal_z: 0.0,
                    extra_hit_vel_x: e.extra_hit_vel.x,
                    extra_hit_vel_y: e.extra_hit_vel.y,
                    extra_hit_vel_z: e.extra_hit_vel.z,
                    is_demo: false,
                });
            }
            crate::ArenaEvent::CarHitCar(e) => {
                buf.push(RCStepEvent {
                    kind: 2,
                    tick_offset,
                    car_idx: e.bumper_car_idx as u32,
                    other_idx: e.victim_car_idx as u32,
                    contact_point_x: e.contact_point.x,
                    contact_point_y: e.contact_point.y,
                    contact_point_z: e.contact_point.z,
                    contact_normal_x: 0.0,
                    contact_normal_y: 0.0,
                    contact_normal_z: 0.0,
                    extra_hit_vel_x: 0.0,
                    extra_hit_vel_y: 0.0,
                    extra_hit_vel_z: 0.0,
                    is_demo: e.is_demo,
                });
            }
            crate::ArenaEvent::CarHitWorld(e) => {
                buf.push(RCStepEvent {
                    kind: 3,
                    tick_offset,
                    car_idx: e.car_idx as u32,
                    other_idx: u32::MAX,
                    contact_point_x: e.contact_point.x,
                    contact_point_y: e.contact_point.y,
                    contact_point_z: e.contact_point.z,
                    contact_normal_x: e.contact_normal.x,
                    contact_normal_y: e.contact_normal.y,
                    contact_normal_z: e.contact_normal.z,
                    extra_hit_vel_x: 0.0,
                    extra_hit_vel_y: 0.0,
                    extra_hit_vel_z: 0.0,
                    is_demo: false,
                });
            }
            crate::ArenaEvent::CarPickupBoost(e) => {
                buf.push(RCStepEvent {
                    kind: 4,
                    tick_offset,
                    car_idx: e.car_idx as u32,
                    other_idx: e.boost_pad_idx as u32,
                    contact_point_x: 0.0,
                    contact_point_y: 0.0,
                    contact_point_z: 0.0,
                    contact_normal_x: 0.0,
                    contact_normal_y: 0.0,
                    contact_normal_z: 0.0,
                    extra_hit_vel_x: 0.0,
                    extra_hit_vel_y: 0.0,
                    extra_hit_vel_z: 0.0,
                    is_demo: false,
                });
            }
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn arena_get_num_step_events(_arena: *const Arena) -> usize {
    STEP_EVENTS_BUF.with(|buf| buf.borrow().len())
}

#[unsafe(no_mangle)]
pub extern "C" fn arena_get_step_event(_arena: *const Arena, idx: usize, out_event: *mut RCStepEvent) -> bool {
    STEP_EVENTS_BUF.with(|buf| {
        let b = buf.borrow();
        if idx < b.len() {
            unsafe {
                *out_event = b[idx];
            }
            true
        } else {
            false
        }
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn arena_get_all_step_events(_arena: *const Arena, out_events: *mut RCStepEvent, max_events: usize) -> usize {
    STEP_EVENTS_BUF.with(|buf| {
        let b = buf.borrow();
        let count = b.len().min(max_events);
        unsafe {
            for i in 0..count {
                *out_events.add(i) = b[i];
            }
        }
        count
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn arena_get_all_boost_pad_states(arena: *const Arena, out_states: *mut RCBoostPadState) {
    unsafe {
        let num_pads = (*arena).num_boost_pads();
        for p in 0..num_pads {
            let p_state = (*arena).get_boost_pad_state(p);
            *out_states.add(p) = RCBoostPadState::from(p_state);
        }
    }
}

static BALL_SYM_STANDARD: std::sync::OnceLock<(rl_ball_sym::Game, rl_ball_sym::Ball)> =
    std::sync::OnceLock::new();

/// Ball-only prediction with `rl_ball_sym`: starting from the given state, steps `ticks`
/// ticks of 1/120 s and writes position/velocity/angular velocity after each tick into
/// `out_states` (length `ticks`). The rotation fields are left untouched.
#[unsafe(no_mangle)]
pub extern "C" fn ball_sym_predict(in_state: *const RCBallState, ticks: u32, out_states: *mut RCBallState) {
    use rl_ball_sym::Vec3A;
    let (game, template) = BALL_SYM_STANDARD.get_or_init(rl_ball_sym::load_standard);
    unsafe {
        let s = &*in_state;
        let mut ball = *template;
        ball.update(
            0.0,
            Vec3A::new(s.pos_x, s.pos_y, s.pos_z),
            Vec3A::new(s.vel_x, s.vel_y, s.vel_z),
            Vec3A::new(s.ang_vel_x, s.ang_vel_y, s.ang_vel_z),
        );
        let out = std::slice::from_raw_parts_mut(out_states, ticks as usize);
        for slot in out.iter_mut() {
            ball.step(game, 1.0 / 120.0);
            slot.pos_x = ball.location.x;
            slot.pos_y = ball.location.y;
            slot.pos_z = ball.location.z;
            slot.vel_x = ball.velocity.x;
            slot.vel_y = ball.velocity.y;
            slot.vel_z = ball.velocity.z;
            slot.ang_vel_x = ball.angular_velocity.x;
            slot.ang_vel_y = ball.angular_velocity.y;
            slot.ang_vel_z = ball.angular_velocity.z;
        }
    }
}

/// Steps `ticks` ticks and writes the ball state after each tick into `out_states`
/// (length `ticks`). Events are discarded. Intended for car-less prediction arenas.
#[unsafe(no_mangle)]
pub extern "C" fn arena_predict_ball(arena: *mut Arena, ticks: u32, out_states: *mut RCBallState) {
    unsafe {
        let arena_ref = &mut *arena;
        let out = std::slice::from_raw_parts_mut(out_states, ticks as usize);
        for slot in out.iter_mut() {
            arena_ref.step_tick();
            *slot = RCBallState::from(*arena_ref.get_ball_state());
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn arena_step_all(
    arena: *mut Arena,
    in_controls: *const RCCarControls,
    num_cars: usize,
    ticks: u32,
    out_car_states: *mut RCCarState,
    out_ball_state: *mut RCBallState,
    out_boost_states: *mut RCBoostPadState,
) {
    unsafe {
        let arena_ref = &mut *arena;

        if !in_controls.is_null() && num_cars > 0 {
            let controls_slice = std::slice::from_raw_parts(in_controls, num_cars);
            for (i, ctrl) in controls_slice.iter().enumerate() {
                if i < arena_ref.cars.len() {
                    arena_ref.set_car_controls(i, CarControls::from(*ctrl));
                }
            }
        }

        STEP_EVENTS_BUF.with(|buf| {
            let mut b = buf.borrow_mut();
            b.clear();
            for t in 0..ticks {
                let events = arena_ref.step_tick();
                record_events(&mut b, events, t);
            }
        });

        if !out_ball_state.is_null() {
            let b_state = arena_ref.get_ball_state();
            *out_ball_state = RCBallState::from(*b_state);
        }

        if !out_car_states.is_null() && num_cars > 0 {
            for i in 0..num_cars {
                if i < arena_ref.cars.len() {
                    let c_state = arena_ref.get_car_state(i);
                    *out_car_states.add(i) = RCCarState::from(*c_state);
                }
            }
        }

        if !out_boost_states.is_null() {
            let num_pads = arena_ref.num_boost_pads();
            for p in 0..num_pads {
                let p_state = arena_ref.get_boost_pad_state(p);
                *out_boost_states.add(p) = RCBoostPadState::from(p_state);
            }
        }
    }
}

thread_local! {
    static BALL_PRED_ARENA: RefCell<Option<Arena>> = const { RefCell::new(None) };
}

#[unsafe(no_mangle)]
pub extern "C" fn ball_pred_predict(
    game_mode: i32,
    start_state: *const RCBallState,
    num_ticks: u32,
    out_states: *mut RCBallState,
) -> bool {
    if start_state.is_null() || out_states.is_null() || num_ticks == 0 {
        return false;
    }

    let gm = match game_mode {
        0 => GameMode::Soccar,
        1 => GameMode::Hoops,
        2 => GameMode::Dropshot,
        3 => GameMode::TheVoid,
        _ => GameMode::Soccar,
    };

    BALL_PRED_ARENA.with(|cell| {
        let mut opt = cell.borrow_mut();
        if opt.is_none() {
            *opt = Some(Arena::new(gm));
        }
        let arena = opt.as_mut().unwrap();

        unsafe {
            let start = *start_state;
            arena.set_ball_state(BallState::from(start));

            for t in 0..num_ticks {
                arena.step_tick();
                let b_state = arena.get_ball_state();
                *out_states.add(t as usize) = RCBallState::from(*b_state);
            }
        }
    });

    true
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn legacy_wheel_state_and_events_keep_their_abi() {
        assert_eq!(std::mem::size_of::<RCCarState>(), 180);
        assert_eq!(std::mem::offset_of!(RCCarState, wheels_with_contact), 73);
        for mask in 0..16u8 {
            let mut state = CarState::default();
            state.boost = 52.0;
            state.is_supersonic = true;
            state.wheels_with_contact = std::array::from_fn(|i|
                (mask & (1 << i) != 0).then_some(RaycastHitInfo::default()));
            let wire = RCCarState::from(state);
            assert_eq!(wire.wheels_with_contact, std::array::from_fn(|i| mask & (1 << i) != 0));
            let restored = CarState::from(wire);
            assert_eq!(restored.wheels_with_contact, state.wheels_with_contact);
            assert_eq!(restored.boost, 52.0);
            assert!(restored.is_supersonic);
        }
        let events = [
            crate::ArenaEvent::CarLanded(crate::CarLandedEvent { car_idx: 0, wheels: [None; 4] }),
            crate::ArenaEvent::CarHitCar(crate::CarHitCarEvent {
                bumper_car_idx: 0, victim_car_idx: 1, contact_point: Vec3A::ZERO, is_demo: true,
            }),
            crate::ArenaEvent::CarPickupBoost(crate::CarPickupBoostEvent { car_idx: 0, boost_pad_idx: 3 }),
        ];
        let mut buf = Vec::new();
        record_events(&mut buf, &events, 5);
        assert_eq!(buf.len(), 2);
        assert_eq!((buf[0].kind, buf[0].tick_offset, buf[0].other_idx), (2, 5, 1));
        assert!(buf[0].is_demo);
        assert_eq!((buf[1].kind, buf[1].tick_offset, buf[1].other_idx), (4, 5, 3));
    }
}
#[unsafe(no_mangle)]
pub extern "C" fn ball_pred_create(game_mode: i32) -> *mut Arena {
    let gm = match game_mode {
        0 => GameMode::Soccar,
        1 => GameMode::Hoops,
        2 => GameMode::Dropshot,
        3 => GameMode::TheVoid,
        _ => GameMode::Soccar,
    };
    Box::into_raw(Box::new(Arena::new(gm)))
}

#[unsafe(no_mangle)]
pub extern "C" fn ball_pred_destroy(arena: *mut Arena) {
    if !arena.is_null() {
        unsafe {
            drop(Box::from_raw(arena));
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn ball_pred_predict_arena(
    arena: *mut Arena,
    start_state: *const RCBallState,
    num_ticks: u32,
    out_states: *mut RCBallState,
) -> bool {
    if arena.is_null() || start_state.is_null() || out_states.is_null() || num_ticks == 0 {
        return false;
    }
    unsafe {
        let arena_ref = &mut *arena;
        arena_ref.set_ball_state(BallState::from(*start_state));
        for t in 0..num_ticks {
            arena_ref.step_tick();
            let b_state = arena_ref.get_ball_state();
            *out_states.add(t as usize) = RCBallState::from(*b_state);
        }
    }
    true
}
