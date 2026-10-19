#include "FuryOBSPadded200B.h"
#include <algorithm>
#include <mutex>
#include <random>
#include <ranges>
#include <utility>

#include "KickoffFunctions.h"
#include "RLGymCPP/GameStates/StateUtil.h"

namespace RLGC {
    static inline float SafeValue(const float v) {
        return std::isfinite(v) ? v : 0.0f;
    }

    static inline Vec SafeVec(Vec v) {
        v.x = SafeValue(v.x);
        v.y = SafeValue(v.y);
        v.z = SafeValue(v.z);
        return v;
    }

    static inline Vec SafeUnit(const Vec &v) {
        const float lenSq = v.LengthSq();
        if (!std::isfinite(lenSq) || lenSq <= 0.000001f) {
            return Vec(0, 0, 0);
        }
        const float invLen = 1.0f / std::sqrt(lenSq);
        if (!std::isfinite(invLen)) {
            return Vec(0, 0, 0);
        }
        return v * invLen;
    }

    FuryOBSPadded200B::FuryOBSPadded200B(
        const int maxPlayers,
        const Vec posCoef,
        const float velCoef,
        const float angVelCoef,
        const bool includeKickoffData,
        const bool includePredictionData,
        const int predictionSteps,
        const float predictionStepTime,
        const bool includeGoalData
    ) : posCoef(posCoef),
        velCoef(velCoef),
        angVelCoef(angVelCoef),
        maxPlayers(maxPlayers),
        includeKickoffData(includeKickoffData),
        includePredictionData(includePredictionData),
        predictionSteps(predictionSteps),
        predictionStepTime(predictionStepTime),
        includeGoalData(includeGoalData),
        includeModeContextData(false) {
    }

    void FuryOBSPadded200B::Reset(const GameState &initialState) {
        ObsBuilder::Reset(initialState);
        localTrajectoryValid = false;
        cachedLocalTrajectoryTick = 0;
        cachedLocalTrajectoryHorizon = 0.0f;
    }

    void FuryOBSPadded200B::AddKickoffData(FList &obs, const Player &player, const GameState &state,
                                       const bool inv) const {
        // Add kickoff active flag (1.0 = kickoff, 0.0 = normal) or just the states normalized
        obs += KickoffFunctions::IsKickoffResult(state).NormalizeKickoffState();

        const Vec ball = state.ball.pos;
        const float distToBall = SafeValue((player.pos - ball).Length());
        obs += SafeValue(distToBall * posCoef.x);

        // Determine if this player is designated kickoff player and give the model the role
        // Leftmost = closest to ball, tie-breaker goes to player with lower X coordinate
        obs += KickoffFunctions::GetPlayerKickoffRoleInt(state, player.team, player) / 3.0f;

        //if (KickoffFunctions::IsKickoffResult(state) != KickoffState::Inactive) {
        const Vec playerInvPos = SafeVec(InvertVec(player.pos, inv));
        float minSpawnDist = FLT_MAX;
        for (const auto &spawn: RLConst::CAR_SPAWN_LOCATIONS_SOCCAR) {
            const auto spawnVec = Vec(spawn.x, spawn.y, RLConst::CAR_SPAWN_REST_Z);
            if (const float dist = SafeValue((playerInvPos - spawnVec).Length()); dist < minSpawnDist) {
                minSpawnDist = dist;
            }
        }
        obs += SafeValue(minSpawnDist * posCoef.x);
        //} else {
        //    obs += 0.0f;
        //}
    }

    void FuryOBSPadded200B::AddGoalData(FList &obs, const Player &player, const GameState &state, bool inv) const {
        const Vec playerPos = SafeVec(InvertVec(player.pos, inv));
        const Vec ballPos = SafeVec(InvertVec(state.ball.pos, inv));
        const Vec ballVel = SafeVec(InvertVec(state.ball.vel, inv));

        constexpr Vec ownGoalPos = CommonValues::BLUE_GOAL_BACK;
        constexpr Vec enemyGoalPos = CommonValues::ORANGE_GOAL_BACK;

        const float distToOwnGoal = SafeValue((playerPos - ownGoalPos).Length());
        obs += SafeValue(distToOwnGoal * posCoef.x);

        const float distToEnemyGoal = SafeValue((playerPos - enemyGoalPos).Length());
        obs += SafeValue(distToEnemyGoal * posCoef.x);

        const float ballDistToEnemyGoal = SafeValue((ballPos - enemyGoalPos).Length());
        obs += SafeValue(ballDistToEnemyGoal * posCoef.x);

        const Vec ballToEnemyGoal = SafeUnit(enemyGoalPos - ballPos);
        const float ballSpeedTowardsGoal = SafeValue(ballVel.Dot(ballToEnemyGoal));
        obs += SafeValue(ballSpeedTowardsGoal * velCoef);

        const Vec ballToOwnGoal = SafeUnit(ownGoalPos - ballPos);
        const float ballSpeedTowardsOwnGoal = SafeValue(ballVel.Dot(ballToOwnGoal));
        obs += SafeValue(ballSpeedTowardsOwnGoal * velCoef);
    }

    void FuryOBSPadded200B::AddPredictionData(FList &obs, const GameState &state, const bool inv) const {
        const BallTrajectory *trajectory = nullptr;
        const float requestedHorizon = predictionStepTime * static_cast<float>(std::max(1, predictionSteps));
        {

            const bool needsRefresh = !localTrajectoryValid ||
                                      cachedLocalTrajectory.slices.empty() ||
                                      cachedLocalTrajectoryTick != state.lastTickCount ||
                                      cachedLocalTrajectoryHorizon + 0.0001f < requestedHorizon;
            if (needsRefresh) {
                localBallPredictor.Predict(
                    cachedLocalTrajectory,
                    state.ball.pos,
                    state.ball.vel,
                    state.ball.angVel,
                    requestedHorizon
                );
                cachedLocalTrajectoryTick = state.lastTickCount;
                cachedLocalTrajectoryHorizon = cachedLocalTrajectory.validUntil;
                localTrajectoryValid = true;
            }
            trajectory = &cachedLocalTrajectory;
        }

        for (int step = 0; step < predictionSteps; ++step) {
            const float targetTime = predictionStepTime * static_cast<float>(step + 1);
            Vec predPos = state.ball.pos;
            Vec predVel = state.ball.vel;

            if (trajectory && !trajectory->slices.empty()) {
                const BallSlice &slice = trajectory->GetSliceAtTime(targetTime);
                predPos = slice.pos;
                predVel = slice.vel;
            }

            const Vec safePos = SafeVec(InvertVec(predPos, inv));
            const Vec safeVel = SafeVec(InvertVec(predVel, inv));
            obs += safePos * posCoef;
            obs += safeVel * velCoef;
        }
    }

    Vec FuryOBSPadded200B::GetNearestTeammateDirection(const Player &player, const GameState &state) {
        float minDistSq = FLT_MAX;
        Vec minDirection(0, 0, 0);

        // find nearest teammate and the direciton
        for (const auto &other: state.players) {
            if (other.carId == player.carId || other.team != player.team) {
                continue;
            }

            const Vec direction = other.pos - player.pos;
            const float distSq = direction.LengthSq();
            if (distSq < minDistSq) {
                minDistSq = distSq;
                minDirection = direction;
            }
        }

        if (minDistSq > 0.000001f) {
            const float invDist = 1.0f / std::sqrt(minDistSq);
            return minDirection * invDist;
        }

        return Vec(0, 0, 0);
    }


    void FuryOBSPadded200B::AddPlayerToObs(FList &obs, const Player &player, const bool inv, const PhysState &ball,
                                       const GameState &state) {
        // Get player physics state (inverted for orange team)
        auto [pos, rotMat, vel, angVel] = InvertPhys(player, inv);
        pos = SafeVec(pos);
        vel = SafeVec(vel);
        angVel = SafeVec(angVel);
        rotMat.forward = SafeVec(rotMat.forward);
        rotMat.up = SafeVec(rotMat.up);

        // Basic player state
        obs += pos * posCoef;
        obs += rotMat.forward;
        obs += rotMat.up;
        obs += vel * velCoef;
        obs += angVel * angVelCoef;
        obs += SafeVec(rotMat.Dot(angVel)) * angVelCoef;

        // Ball position/velocity relative to player (in player's local space)
        // NOTE: Training used rotMat (inverted for orange), so we must keep that for consistency
        Vec toBallWorld = ball.pos - pos;
        Vec localBallPos = SafeVec(rotMat.Dot(toBallWorld));
        Vec localBallVel = SafeVec(rotMat.Dot(ball.vel - vel));

        obs += localBallPos * posCoef;
        obs += localBallVel * velCoef;

        // Player status flags
        float boostNormalized = SafeValue(player.boost) / 100.0f;
        boostNormalized = std::clamp(boostNormalized, 0.0f, 1.0f);
        obs += boostNormalized;
        obs += player.isOnGround ? 1.0f : 0.0f;
        obs += player.HasFlipOrJump() ? 1.0f : 0.0f;
        obs += player.HasFlipReset() ? 1.0f : 0.0f;
        obs += player.isDemoed ? 1.0f : 0.0f;
        obs += player.hasJumped ? 1.0f : 0.0f;

        // Direction vectors to goals (for offensive/defensive awareness)

        constexpr Vec ownGoal = CommonValues::BLUE_GOAL_BACK;
        constexpr Vec enemyGoal = CommonValues::ORANGE_GOAL_BACK;

        const Vec toOwnGoalVec = ownGoal - pos;
        const Vec toEnemyGoalVec = enemyGoal - pos;

        obs += SafeUnit(toOwnGoalVec);
        obs += SafeUnit(toEnemyGoalVec);

        Vec teammateDir = GetNearestTeammateDirection(player, state);
        teammateDir = SafeVec(InvertVec(teammateDir, inv));
        obs += teammateDir;
    }

    void FuryOBSPadded200B::BuildObsInto(const Player &player, const GameState &state, FList &result) {
        result.clear();
        result.reserve(static_cast<size_t>(GetObsSize()));

        const bool inv = player.team == Team::ORANGE;

        auto ball = InvertPhys(state.ball, inv);
        auto &pads = state.GetBoostPads(inv);
        auto &padTimers = state.GetBoostPadTimers(inv);

        result += SafeVec(ball.pos) * posCoef;
        result += SafeVec(ball.vel) * velCoef;
        result += SafeVec(ball.angVel) * angVelCoef;

        for (int i = 0; i < RLGC::Action::ELEM_AMOUNT; ++i) {
            result += SafeValue(player.prevAction[i]);
        }

        for (int i = 0; i < CommonValues::BOOST_LOCATIONS_AMOUNT; ++i) {
            if (pads[i]) {
                result += 1.f;
            } else {
                float timer = SafeValue(padTimers[i]);
                timer = std::clamp(timer, 0.0f, 100.0f);
                result += 1.f / (1.f + timer);
            }
        }

        if (includeKickoffData) {
            AddKickoffData(result, player, state, inv);
        }

        if (includePredictionData) {
            AddPredictionData(result, state, inv);
        }

        if (includeGoalData) {
            AddGoalData(result, player, state, inv);
        }

        FList selfObs = {};
        selfObs.reserve(64);
        AddPlayerToObs(selfObs, player, inv, ball, state);
        result += selfObs;

        std::vector<std::tuple<float, uint32_t, FList> > teammates, opponents;
        teammates.reserve(static_cast<size_t>(std::max(0, maxPlayers - 1)));
        opponents.reserve(static_cast<size_t>(std::max(0, maxPlayers)));

        const Vec ballPosInv = SafeVec(InvertVec(state.ball.pos, inv));

        const auto makeDistanceKey = [&](const Player &other) {
            Vec p = SafeVec(InvertVec(other.pos, inv));
            float distSq = (p - ballPosInv).LengthSq();
            if (!std::isfinite(distSq)) distSq = FLT_MAX;
            return std::pair<float, uint32_t>(distSq, other.carId);
        };

        for (auto &otherPlayer: state.players) {
            if (otherPlayer.carId == player.carId) {
                continue;
            }

            FList playerObs = {};
            AddPlayerToObs(playerObs, otherPlayer, inv, ball, state);
            auto key = makeDistanceKey(otherPlayer);
            ((otherPlayer.team == player.team) ? teammates : opponents)
                    .emplace_back(key.first, key.second, std::move(playerObs));
        }

        const size_t maxTeammates = static_cast<size_t>(std::max(0, maxPlayers - 1));
        const size_t maxOpponents = static_cast<size_t>(std::max(0, maxPlayers));

        if (teammates.size() > maxTeammates) {
            static std::once_flag teammatesOverflowWarned;
            std::call_once(teammatesOverflowWarned, [maxTeammates]() {
                RG_LOG("FuryOBSPaddedOld: Too many teammates for Obs, truncating to " << maxTeammates);
            });
            teammates.resize(maxTeammates);
        }

        if (opponents.size() > maxOpponents) {
            static std::once_flag opponentsOverflowWarned;
            std::call_once(opponentsOverflowWarned, [maxOpponents]() {
                RG_LOG("FuryOBSPaddedOld: Too many opponents for Obs, truncating to " << maxOpponents);
            });
            opponents.resize(maxOpponents);
        }

        auto sorter = [](const auto &a, const auto &b) {
            if (std::get<0>(a) != std::get<0>(b))
                return std::get<0>(a) < std::get<0>(b);
            return std::get<1>(a) < std::get<1>(b);
        };

        std::ranges::sort(teammates, sorter);
        std::ranges::sort(opponents, sorter);

        FList emptyPlayerObs(PLAYER_OBS_SIZE, 0.0f);

        for (const auto &t: teammates) {
            result += std::get<2>(t);
        }
        for (size_t i = teammates.size(); i < maxTeammates; ++i) {
            result += emptyPlayerObs;
        }

        for (const auto &o: opponents) {
            result += std::get<2>(o);
        }
        for (size_t i = opponents.size(); i < maxOpponents; ++i) {
            result += emptyPlayerObs;
        }
    }

    FList FuryOBSPadded200B::BuildObs(const Player &player, const GameState &state) {
        FList result;
        BuildObsInto(player, state, result);
        return result;
    }

    int FuryOBSPadded200B::GetObsSize() const {
        // Calculate size based on components
        int size = 0;

        // Global ball state (position, velocity, angular velocity)
        size += 3 + 3 + 3;

        // Previous action (8 floats)
        size += 8;

        // Boost pad availability (34 pads)
        size += CommonValues::BOOST_LOCATIONS_AMOUNT;

        // Optional kickoff data (4 floats)
        if (includeKickoffData)
            size += 4;

        // Optional prediction data (predictionSteps +� 6 floats)
        if (includePredictionData)
            size += predictionSteps * 6;

        // Optional goal data (5 floats)
        if (includeGoalData)
            size += 5;

        // PLAYER_OBS_SIZE should be like 39 here so just use a constant ig ??!
        const int teammateCount = std::max(0, maxPlayers - 1);
        const int opponentCount = std::max(0, maxPlayers);

        // self
        size += PLAYER_OBS_SIZE;

        // max teammates - 1 * player obs size
        size += teammateCount * PLAYER_OBS_SIZE;

        // max opponents * player obs size
        size += opponentCount * PLAYER_OBS_SIZE;

        return size;
    }
}
