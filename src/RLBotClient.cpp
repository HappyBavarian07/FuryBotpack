#include "RLBotClient.h"
#include "obs/KickoffFunctions.h"
#include <algorithm>

using namespace RLGC;

namespace
{
    Vec ToVec(const rlbot::flat::Vector3 rlbotVec) {
        return Vec(rlbotVec.x(), rlbotVec.y(), rlbotVec.z());
    }

    PhysState ToPhysObj(const rlbot::flat::Physics* phys) {
        PhysState obj = {};
        obj.pos = ToVec(phys->location());

        Angle ang = Angle(phys->rotation().yaw(), phys->rotation().pitch(), phys->rotation().roll());
        obj.rotMat = ang.ToRotMat();

        obj.vel = ToVec(phys->velocity());
        obj.angVel = ToVec(phys->angular_velocity());
        return obj;
    }

    Player ToPlayer(const rlbot::flat::PlayerInfo* playerInfo, float dtSec, PlayerTimingState& timing)
    {
        Player pd = {};

        static_cast<PhysState&>(pd) = ToPhysObj(playerInfo->physics());

        pd.carId = playerInfo->player_id();
        pd.team = (Team)playerInfo->team();

        pd.boost = playerInfo->boost();

        pd.isOnGround = (playerInfo->air_state() == rlbot::flat::AirState::OnGround);
        pd.hasDoubleJumped = playerInfo->has_double_jumped();
        pd.hasFlipped = playerInfo->has_dodged();
        pd.isJumping = playerInfo->air_state() == rlbot::flat::AirState::Jumping;
        pd.isFlipping = playerInfo->air_state() == rlbot::flat::AirState::Dodging;
        pd.hasJumped = playerInfo->has_jumped() || pd.hasFlipped || pd.hasDoubleJumped || pd.isJumping;
        pd.isDemoed = playerInfo->demolished_timeout() > 0.f;

        // Approximate airtime timers (used for HasFlipOrJump behavior)
        if (pd.isOnGround) {
            timing.airTime = 0.f;
            timing.airTimeSinceJump = 0.f;
        }
        else {
            timing.airTime += dtSec;

            timing.airTimeSinceJump = pd.hasJumped && !pd.isJumping ? timing.airTimeSinceJump + dtSec : 0.f;
            if (pd.hasJumped && !pd.hasFlipped && !pd.hasDoubleJumped && !pd.isJumping &&
                playerInfo->dodge_timeout() > 0.f) {
                timing.airTimeSinceJump = std::clamp(
                    RLConst::DOUBLEJUMP_MAX_DELAY - playerInfo->dodge_timeout(),
                    0.f, RLConst::DOUBLEJUMP_MAX_DELAY);
            }
        }

        pd.airTime = timing.airTime;
        pd.airTimeSinceJump = timing.airTimeSinceJump;

        return pd;
    }

    GameState ToGameState(rlbot::flat::GamePacket const* packet, float dtSec, std::vector<PlayerTimingState>& playerTiming) {
        GameState gs = {};

        auto players = packet->players();
        if (players) {
            const int n = (int)players->size();
            if ((int)playerTiming.size() < n)
                playerTiming.resize(n);

            gs.players.reserve(n);
            for (int i = 0; i < n; i++) {
                gs.players.push_back(ToPlayer(players->Get(i), dtSec, playerTiming[i]));
                gs.players.back().index = i;
            }
        }

        static_cast<PhysState&>(gs.ball) = ToPhysObj(packet->balls()->Get(0)->physics());

        auto boostPadStates = packet->boost_pads();
        if (boostPadStates->size() != CommonValues::BOOST_LOCATIONS_AMOUNT) {
            if (rand() % 20 == 0) { // Don't spam-log as that will lag the bot
                RG_LOG(
                    "RLBotClient ToGameState(): Bad boost pad amount, expected "
                    << CommonValues::BOOST_LOCATIONS_AMOUNT << " but got " << boostPadStates->size()
                );
            }

            // Just set all boost pads to on
            std::fill(gs.boostPads.begin(), gs.boostPads.end(), 1);
        }
        else {
            for (int i = 0; i < CommonValues::BOOST_LOCATIONS_AMOUNT; i++) {
                gs.boostPads[i] = boostPadStates->Get(i)->is_active();
                gs.boostPadsInv[CommonValues::BOOST_LOCATIONS_AMOUNT - i - 1] = gs.boostPads[i];

                gs.boostPadTimers[i] = boostPadStates->Get(i)->timer();
                gs.boostPadTimersInv[CommonValues::BOOST_LOCATIONS_AMOUNT - i - 1] = gs.boostPadTimers[i];
            }
        }

        return gs;
    }
} // anonymous namespace

RLBotBot::RLBotBot(std::unordered_set<unsigned> indices_,
    unsigned const team_,
    std::string name_,
    std::shared_ptr<const SharedBotContext> ctx) noexcept
    : rlbot::Bot(std::move(indices_), team_, std::move(name_))
    , ctx_(std::move(ctx))
{
    std::set<unsigned> sorted(std::begin(indices), std::end(indices));
    for (auto const& index : sorted)
        std::printf("Team %u Index %u: %s created\n", team_, index, name_.c_str());
}

RLBotBot::~RLBotBot() {}

void RLBotBot::update(rlbot::flat::GamePacket const* packet,
    rlbot::flat::BallPrediction const* ballPrediction_) noexcept
{
    if (!packet || !packet->match_info() || !packet->players() ||
        !packet->balls() || packet->balls()->size() == 0) {
        return;
    }

    float curTime = packet->match_info()->seconds_elapsed();
    const uint32_t frame = packet->match_info()->frame_num();
    const uint32_t elapsedFrames = frame - prevFrame;
    if (ticks >= 0 && (curTime < prevTime || elapsedFrames > 0x80000000u)) {
        ticks = -1;
        totalTicks = 0;
        m_botState.clear();
        m_playerTiming.clear();
        RLGC::KickoffFunctions::ResetArenaCache(nullptr);
    }
    const bool firstPacket = ticks < 0;
    if (!firstPacket && elapsedFrames == 0) return;
    const int elapsed = firstPacket ? 0 : static_cast<int>(std::min(elapsedFrames, 65536u));
    const float deltaTime = elapsed / 120.f;
    prevTime = curTime;
    prevFrame = frame;
    ticks = firstPacket ? ctx_->params.tickSkip : ticks + elapsed;
    totalTicks += elapsed;

    const auto matchPhase = packet->match_info()->match_phase();
    if (matchPhase != rlbot::flat::MatchPhase::Active &&
        matchPhase != rlbot::flat::MatchPhase::Kickoff &&
        matchPhase != rlbot::flat::MatchPhase::GoalScored) {
        ticks = ctx_->params.tickSkip;
        m_botState.clear();
        m_playerTiming.clear();
        for (const auto index : indices) setOutput(index, {});
        return;
    }
    // On GoalScored the phase is often shorter than one tickSkip window, so
    // infer and apply immediately so the bot keeps reacting after a goal
    // instead of holding the last input until Replay clears it.
    const bool isGoalScored = matchPhase == rlbot::flat::MatchPhase::GoalScored;
    const bool inferNow = isGoalScored || ticks >= ctx_->params.tickSkip;
    const bool applyPrevious = inferNow && (isGoalScored || ticks >= ctx_->params.actionDelay);
    if (inferNow) ticks = 0;

    GameState gs = ToGameState(packet, deltaTime, m_playerTiming);
    gs.deltaTime = deltaTime;
    gs.lastTickCount = totalTicks;
    if (matchPhase == rlbot::flat::MatchPhase::GoalScored) {
        gs.ball.pos = Vec{0.f, 0.f, 900.f};
        gs.ball.vel = Vec{};
        gs.ball.angVel = Vec{};
        gs.ball.rotMat = RotMat::GetIdentity();
    }

    for (auto const& index : this->indices)
    {
        if (index >= gs.players.size()) continue;
        auto& st = m_botState[index];
        if (!st.initialized) {
            st.initialized = true;

            st.action = RLGC::Action{};
            st.controls = RLGC::Action{};
        }

        auto& localPlayer = gs.players[index];
        if (applyPrevious) st.controls = st.action;
        localPlayer.prevAction = st.controls;

        if (inferNow) {
            st.action = ctx_->inferUnit->InferAction(localPlayer, gs, true);
        }

        if (isGoalScored || ticks >= ctx_->params.actionDelay) {
            // Apply new action (immediately on GoalScored)
            st.controls = st.action;
        }

        const auto& c = st.controls;
        setOutput(index, {
            c.throttle,
            c.steer,
            c.pitch,
            c.yaw,
            c.roll,
            c.jump > 0.5f,
            c.boost > 0.5f,
            c.handbrake > 0.5f,
            false,
            });
    }

}

void RLBotBot::SelfTest(std::shared_ptr<const SharedBotContext> context) {
    using namespace rlbot::flat;
    auto check = [](bool condition, const char* message) {
        if (!condition) throw std::runtime_error(message);
    };
    GamePacketT packet;
    packet.match_info = std::make_unique<MatchInfoT>();
    packet.match_info->match_phase = MatchPhase::Active;
    packet.players.push_back(std::make_unique<PlayerInfoT>());
    packet.players[0]->physics = std::make_unique<Physics>(
        Vector3(0, -3000, 17), Rotator(0, 0, 0), Vector3(0, 0, 0), Vector3(0, 0, 0));
    packet.balls.push_back(std::make_unique<BallInfoT>());
    packet.balls[0]->physics = std::make_unique<Physics>(
        Vector3(100, 200, 500), Rotator(0, 0, 0), Vector3(200, 300, 100), Vector3(0, 0, 0));
    packet.boost_pads.assign(34, rlbot::flat::BoostPadState(true, 0));
    RLBotBot bot({0}, 0, "Fury packet check", context);
    auto updateFrame = [&](uint32_t frame) {
        packet.match_info->frame_num = frame;
        // Frozen seconds test: cadence must follow frame_num, including countdown.
        flatbuffers::FlatBufferBuilder builder;
        builder.Finish(GamePacket::Pack(builder, &packet));
        bot.update(flatbuffers::GetRoot<GamePacket>(builder.GetBufferPointer()), nullptr);
    };
    auto sameAction = [](const Action& a, const Action& b) {
        return std::equal(a.begin(), a.end(), b.begin());
    };
    updateFrame(100);
    const Action pending = bot.m_botState.at(0).action;
    for (uint32_t frame = 101; frame < 107; ++frame) {
        updateFrame(frame);
        check(sameAction(bot.m_botState.at(0).controls, Action{}), "Action applied before delay 7");
    }
    updateFrame(107);
    check(sameAction(bot.m_botState.at(0).controls, pending), "Action missing at delay 7");
    updateFrame(108);
    check(bot.ticks == 0, "Decision interval is not 8 frames");
    updateFrame(108);
    check(bot.ticks == 0, "Duplicate frame advanced cadence");
    updateFrame(114);
    const Action pendingAfterDrop = bot.m_botState.at(0).action;
    updateFrame(116);
    check(sameAction(bot.m_botState.at(0).controls, pendingAfterDrop), "Dropped frame lost pending action");
    packet.match_info->match_phase = MatchPhase::Countdown;
    updateFrame(117);
    check(bot.m_botState.empty() && bot.getOutput(0).throttle() == 0.f,
          "Countdown did not clear controls");

    packet.players[0]->boost = 33.f;
    packet.players[0]->physics = std::make_unique<Physics>(
        Vector3(0, -4608, 17), Rotator(0, 1.5707963f, 0), Vector3(0, 0, 0), Vector3(0, 0, 0));
    packet.balls[0]->physics = std::make_unique<Physics>(
        Vector3(0, 0, RLConst::BALL_REST_Z), Rotator(0, 0, 0), Vector3(0, 0, 0), Vector3(0, 0, 0));
    packet.match_info->match_phase = MatchPhase::Kickoff;
    updateFrame(118);
    check(bot.m_botState.contains(0), "Playable Kickoff phase suppressed inference");
    const Action kickoffAction = bot.m_botState.at(0).action;
    for (uint32_t frame = 119; frame < 125; ++frame) {
        updateFrame(frame);
        check(sameAction(bot.m_botState.at(0).controls, Action{}), "Kickoff bypassed action delay 7");
    }
    updateFrame(125);
    check(sameAction(bot.m_botState.at(0).controls, kickoffAction), "Kickoff action was not applied");
    check(bot.getOutput(0).throttle() > 0.f, "Fury did not drive forward during playable kickoff");
    packet.match_info->match_phase = MatchPhase::Active;
    updateFrame(126);
    check(bot.ticks == 0 && sameAction(bot.m_botState.at(0).controls, kickoffAction),
          "Kickoff-to-Active transition reset action timing");

    packet.match_info->match_phase = MatchPhase::GoalScored;
    updateFrame(127);
    // GoalScored infers and applies immediately (brief phase before Replay)
    check(bot.m_botState.contains(0) && bot.ticks == 0, "GoalScored suppressed immediate inference");
    check(sameAction(bot.m_botState.at(0).controls, bot.m_botState.at(0).action),
          "GoalScored did not apply action immediately");
    updateFrame(128);
    check(bot.ticks == 0, "GoalScored does not infer every frame");
    packet.match_info->match_phase = MatchPhase::Replay;
    updateFrame(129);
    check(bot.m_botState.empty() && bot.getOutput(0).throttle() == 0.f, "Replay did not clear controls");
    packet.match_info->match_phase = MatchPhase::Countdown;
    updateFrame(130);
    check(bot.m_botState.empty() && bot.getOutput(0).throttle() == 0.f,
          "Post-goal Countdown did not clear controls");

    auto firstAction = [&](MatchPhase phase, Vector3 ballPos) {
        packet.balls[0]->physics = std::make_unique<Physics>(
            ballPos, Rotator(0, 0, 0), Vector3(0, 0, 0), Vector3(0, 0, 0));
        packet.match_info->match_phase = phase;
        RLBotBot probe({0}, 0, "Fury ball check", context);
        flatbuffers::FlatBufferBuilder probeBuilder;
        probeBuilder.Finish(GamePacket::Pack(probeBuilder, &packet));
        probe.update(flatbuffers::GetRoot<GamePacket>(probeBuilder.GetBufferPointer()), nullptr);
        return probe.m_botState.at(0).action;
    };
    const Vector3 farBall(3000, 4800, 1800), midBall(0, 0, 900);
    check(!sameAction(firstAction(MatchPhase::Active, farBall), firstAction(MatchPhase::Active, midBall)),
          "Ball-override check has no power: Active actions match for both balls");
    check(sameAction(firstAction(MatchPhase::GoalScored, farBall), firstAction(MatchPhase::GoalScored, midBall)),
          "GoalScored did not replace the ball with the fixed post-goal ball");
    check(sameAction(firstAction(MatchPhase::GoalScored, farBall), firstAction(MatchPhase::Active, midBall)),
          "Post-goal ball is not (0, 0, 900)");

    auto& player = *packet.players[0];
    player.air_state = AirState::InAir;
    player.has_jumped = true;
    player.dodge_timeout = 0.2f;
    flatbuffers::FlatBufferBuilder builder;
    builder.Finish(GamePacket::Pack(builder, &packet));
    PlayerTimingState timing;
    const auto converted = ToPlayer(flatbuffers::GetRoot<GamePacket>(
        builder.GetBufferPointer())->players()->Get(0), 1.f / 120.f, timing);
    check(!converted.isDemoed && std::abs(converted.airTimeSinceJump -
          (RLConst::DOUBLEJUMP_MAX_DELAY - 0.2f)) < 0.0001f, "Dodge timeout translation failed");
    player.has_jumped = false;
    player.dodge_timeout = 0.f;
    builder.Clear();
    builder.Finish(GamePacket::Pack(builder, &packet));
    const auto reset = ToPlayer(flatbuffers::GetRoot<GamePacket>(
        builder.GetBufferPointer())->players()->Get(0), 1.f / 120.f, timing);
    check(reset.HasFlipReset() && reset.airTimeSinceJump == 0.f, "Flip reset availability translation failed");
    context->obs->Reset(GameState{});
    KickoffFunctions::ResetArenaCache(nullptr);
    std::cout << "PASS: v5 packet flip availability, frame cadence 8/7, duplicate/dropped frames, neutral countdown/replay, playable kickoff/post-goal\n";
}
