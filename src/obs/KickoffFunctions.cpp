#include "KickoffFunctions.h"
#include <RLGymCPP/CommonValues.h>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <mutex>
#include <unordered_set>
#include <vector>

namespace RLGC {
    struct KickoffArenaCache {
        static constexpr int MAX_PLAYERS = 6;

        mutable std::recursive_mutex mutex;

        KickoffResult cachedResult{};
        uint64_t cachedTick = UINT64_MAX;

        bool wasKickoff = false;
        uint64_t lastPlayersLeftSpawnTick = 0;
        uint64_t lastBallLeftSpawnTick = 0;
        uint64_t kickoffStartedBeginTick = 0;
        uint64_t movementCandidateTick = 0;
        bool kickoffTimedOut = false;

        uint32_t designatedBlue = UINT32_MAX;
        uint32_t designatedOrange = UINT32_MAX;

        bool wasDesignatedPlayer[MAX_PLAYERS] = {};
        uint8_t roleAtStart[MAX_PLAYERS] = {};
        bool touchedDuringKickoff[MAX_PLAYERS] = {};
        float startDistToBall[MAX_PLAYERS] = {};
        float startBoost[MAX_PLAYERS] = {};
        float endDistToBall[MAX_PLAYERS] = {};
        float endBoost[MAX_PLAYERS] = {};
        bool startCaptured = false;
        float outcomeRewardCache[MAX_PLAYERS] = {};
        bool outcomeRewardCached[MAX_PLAYERS] = {};
        bool outcomeRewardPaid[MAX_PLAYERS] = {};
        bool kickoffJustEnded = false;
        uint64_t lastKickoffEndTick = 0;
        uint64_t outcomeEvaluationReadyTick = 0;
        KickoffState outcomePrevState = KickoffState::Inactive;
        KickoffState prevKickoffState = KickoffState::Inactive;
        uint32_t designatedFirstTouch = UINT32_MAX;
        bool firstTouchResolved = false;
        bool firstTouchTied = false;
        uint32_t designatedMaskBlue = UINT32_MAX;
        uint32_t designatedMaskOrange = UINT32_MAX;
        bool designatedMaskValid = false;
        float ballSpeedAtKickoffEnd = 0.0f;
        uint64_t followUpWindowEnd = 0;
        bool followUpClaimed = false;
        uint32_t followUpRecipient = UINT32_MAX;
        int postKickoffWinner = -1;
        bool postKickoffOppTouched = false;
        bool outcomePending = false;
        int outcomeWinnerTeam = -1;
        bool outcomeWinnerDesignated = false;
        bool outcomeByGoal = false;
        bool postKickoffPenaltyPending[MAX_PLAYERS] = {};
        uint64_t postKickoffTick = 0;
        uint64_t possessionConfirmTick = 0;
        bool possessionConfirmPending = false;
        bool possessionConfirmPaid[MAX_PLAYERS] = {};
        int possessionConfirmTeam = -1;

        uint64_t lastPreStepTick = 0;

        void Reset() {
            cachedResult = KickoffResult{};
            cachedTick = UINT64_MAX;
            wasKickoff = false;
            lastPlayersLeftSpawnTick = 0;
            lastBallLeftSpawnTick = 0;
            kickoffStartedBeginTick = 0;
            movementCandidateTick = 0;
            kickoffTimedOut = false;
            designatedBlue = UINT32_MAX;
            designatedOrange = UINT32_MAX;
            std::memset(wasDesignatedPlayer, 0, sizeof(wasDesignatedPlayer));
            std::memset(roleAtStart, 0, sizeof(roleAtStart));
            std::memset(touchedDuringKickoff, 0, sizeof(touchedDuringKickoff));
            std::memset(startDistToBall, 0, sizeof(startDistToBall));
            std::memset(startBoost, 0, sizeof(startBoost));
            std::memset(endDistToBall, 0, sizeof(endDistToBall));
            std::memset(endBoost, 0, sizeof(endBoost));
            startCaptured = false;
            std::memset(outcomeRewardCache, 0, sizeof(outcomeRewardCache));
            std::memset(outcomeRewardCached, 0, sizeof(outcomeRewardCached));
            std::memset(outcomeRewardPaid, 0, sizeof(outcomeRewardPaid));
            kickoffJustEnded = false;
            lastKickoffEndTick = 0;
            outcomeEvaluationReadyTick = 0;
            outcomePrevState = KickoffState::Inactive;
            prevKickoffState = KickoffState::Inactive;
            designatedFirstTouch = UINT32_MAX;
            firstTouchResolved = false;
            firstTouchTied = false;
            designatedMaskBlue = UINT32_MAX;
            designatedMaskOrange = UINT32_MAX;
            designatedMaskValid = false;
            ballSpeedAtKickoffEnd = 0.0f;
            followUpWindowEnd = 0;
            followUpClaimed = false;
            followUpRecipient = UINT32_MAX;
            postKickoffWinner = -1;
            postKickoffOppTouched = false;
            outcomePending = false;
            outcomeWinnerTeam = -1;
            outcomeWinnerDesignated = false;
            outcomeByGoal = false;
            std::memset(postKickoffPenaltyPending, 0, sizeof(postKickoffPenaltyPending));
            postKickoffTick = 0;
            possessionConfirmTick = 0;
            possessionConfirmPending = false;
            std::memset(possessionConfirmPaid, 0, sizeof(possessionConfirmPaid));
            possessionConfirmTeam = -1;
            lastPreStepTick = 0;
        }
    };

    static constexpr int MAX_ARENAS = 16384;
    static constexpr uint32_t ARENA_HASH_MASK = MAX_ARENAS - 1;
    static std::atomic<const Arena *> g_arenas[MAX_ARENAS];
    static KickoffArenaCache g_arenaCaches[MAX_ARENAS];

    static KickoffArenaCache &GetArenaCache(const Arena *arenaKey) {
        if (!arenaKey) {
            static KickoffArenaCache fallback;
            return fallback;
        }

        thread_local const Arena *lastArena = nullptr;
        thread_local KickoffArenaCache *lastCache = nullptr;
        if (lastArena == arenaKey) {
            return *lastCache;
        }

        // Fast open-addressed hash lookup with linear probing (Fibonacci pointer hash)
        uint32_t h = static_cast<uint32_t>(((reinterpret_cast<uintptr_t>(arenaKey) >> 6) * 0x9E3779B97F4A7C15ULL) & ARENA_HASH_MASK);
        for (int probe = 0; probe < MAX_ARENAS; probe++) {
            uint32_t idx = (h + probe) & ARENA_HASH_MASK;
            const Arena *a = g_arenas[idx].load(std::memory_order_relaxed);
            if (a == arenaKey) {
                lastArena = arenaKey;
                lastCache = &g_arenaCaches[idx];
                return g_arenaCaches[idx];
            }
            if (a == nullptr) {
                const Arena *expected = nullptr;
                if (g_arenas[idx].compare_exchange_strong(expected, arenaKey)) {
                    std::lock_guard<std::recursive_mutex> lock(g_arenaCaches[idx].mutex);
                    g_arenaCaches[idx].Reset();
                    lastArena = arenaKey;
                    lastCache = &g_arenaCaches[idx];
                    return g_arenaCaches[idx];
                }
                a = g_arenas[idx].load(std::memory_order_relaxed);
                if (a == arenaKey) {
                    lastArena = arenaKey;
                    lastCache = &g_arenaCaches[idx];
                    return g_arenaCaches[idx];
                }
            }
        }
        static KickoffArenaCache fallback;
        return fallback;
    }

    void KickoffFunctions::ResetArenaCache(const Arena *arenaKey) {
        auto &cache = GetArenaCache(arenaKey);
        std::lock_guard<std::recursive_mutex> lock(cache.mutex);
        cache.Reset();
    }

    static KickoffResult make_result(KickoffState s, uint64_t remaining = 0, bool timedOut = false) {
        KickoffResult r{};
        r.state = s;
        r.remainingHoldTicks = remaining;
        r.timedOut = timedOut;
        return r;
    }

    KickoffResult KickoffFunctions::IsKickoffResult(const GameState &state) {
        const uint64_t curTick = state.lastTickCount;
        const Arena *arenaKey = state.lastArena;

        auto &cache = GetArenaCache(arenaKey);
        std::lock_guard<std::recursive_mutex> lock(cache.mutex);

        auto resetKickoffCache = [&cache]() {
            cache.wasKickoff = false;
            cache.lastPlayersLeftSpawnTick = 0;
            cache.lastBallLeftSpawnTick = 0;
            cache.kickoffStartedBeginTick = 0;
            cache.movementCandidateTick = 0;
            cache.kickoffTimedOut = false;
        };

        if (cache.cachedTick != UINT64_MAX && curTick < cache.cachedTick) {
            resetKickoffCache();
        }

        if (cache.cachedTick == curTick) {
            return cache.cachedResult;
        }

        auto &arenaCache = cache;

        constexpr float PLAYER_SPAWN_TOLERANCE = 50.0f;
        constexpr float PLAYER_SPAWN_TOLERANCE_SQ = PLAYER_SPAWN_TOLERANCE * PLAYER_SPAWN_TOLERANCE;
        constexpr float BALL_POS_TOLERANCE = 5.0f;
        constexpr float BALL_VEL_MOVE_THRESHOLD = 20.0f;
        constexpr float BALL_VEL_MOVE_THRESHOLD_SQ = BALL_VEL_MOVE_THRESHOLD * BALL_VEL_MOVE_THRESHOLD;
        constexpr float PLAYER_HORIZ_VEL_THRESHOLD = 110.0f;
        constexpr float PLAYER_HORIZ_VEL_THRESHOLD_SQ = PLAYER_HORIZ_VEL_THRESHOLD * PLAYER_HORIZ_VEL_THRESHOLD;
        constexpr uint64_t PLAYER_MOVEMENT_CONFIRM_TICKS = 8;
        constexpr uint64_t GRACE_HOLD_TICKS = 90;
        constexpr float GRACE_CENTER_EXIT_RADIUS = 1400.0f;
        constexpr float GRACE_CENTER_EXIT_RADIUS_SQ =
                GRACE_CENTER_EXIT_RADIUS * GRACE_CENTER_EXIT_RADIUS;
        constexpr uint64_t KICKOFF_TIMEOUT_TICKS = 540;

        const Vec &b = state.ball.pos;
        const bool ballAtSpawn = (std::abs(b.x) <= BALL_POS_TOLERANCE && std::abs(b.y) <= BALL_POS_TOLERANCE &&
                                  std::abs(b.z - RLConst::BALL_REST_Z) <= BALL_POS_TOLERANCE);
        const bool ballIsMoving = state.ball.vel.LengthSq() > BALL_VEL_MOVE_THRESHOLD_SQ;

        bool allPlayersNearSpawn = true;
        bool allPlayersOnSpawn = true;
        for (const auto &player: state.players) {
            const float px = player.pos.x;
            const float py = player.pos.y;
            const float pz = player.pos.z;
            const float teamFlip = (player.team == Team::ORANGE) ? -1.0f : 1.0f;

            bool nearSomeSpawn = false;
            for (const auto &spawn: RLConst::CAR_SPAWN_LOCATIONS_SOCCAR) {
                const float sx = spawn.x * teamFlip;
                const float sy = spawn.y * teamFlip;

                const float dx = px - sx;
                const float dy = py - sy;
                const float dz = pz - RLConst::CAR_SPAWN_REST_Z;

                if (dx * dx + dy * dy + dz * dz < PLAYER_SPAWN_TOLERANCE_SQ) {
                    nearSomeSpawn = true;
                    break;
                }
            }
            if (!nearSomeSpawn) {
                allPlayersNearSpawn = false;
                allPlayersOnSpawn = false;
                break;
            }

            const float playerHorizSpeedSq = player.vel.x * player.vel.x + player.vel.y * player.vel.y;
            if (playerHorizSpeedSq > PLAYER_HORIZ_VEL_THRESHOLD_SQ) {
                allPlayersOnSpawn = false;
            }
        }

        const bool kickoffLikeFrame = allPlayersNearSpawn && ballAtSpawn && !ballIsMoving;

        if (kickoffLikeFrame) {
            if (!arenaCache.wasKickoff || arenaCache.lastPlayersLeftSpawnTick != 0) {
                arenaCache.wasKickoff = true;
                arenaCache.lastPlayersLeftSpawnTick = 0;
                arenaCache.lastBallLeftSpawnTick = 0;
                arenaCache.kickoffStartedBeginTick = 0;
                arenaCache.movementCandidateTick = 0;
                arenaCache.kickoffTimedOut = false;
            }
        }

        const bool initialDetected = allPlayersOnSpawn && ballAtSpawn;

        KickoffResult result = make_result(KickoffState::Inactive, 0, false);

        if (initialDetected && (!arenaCache.wasKickoff || arenaCache.lastPlayersLeftSpawnTick == 0)) {
            arenaCache.wasKickoff = true;
            arenaCache.lastPlayersLeftSpawnTick = 0;
            arenaCache.lastBallLeftSpawnTick = 0;
            arenaCache.kickoffStartedBeginTick = 0;
            arenaCache.movementCandidateTick = 0;
            arenaCache.kickoffTimedOut = false;
            result = make_result(KickoffState::KickoffRest, 0, false);
        } else if (!arenaCache.wasKickoff) {
            result = make_result(KickoffState::Inactive, 0, false);
        } else {
            const bool ballLeftSpawn = !ballAtSpawn || ballIsMoving;

            if (!ballLeftSpawn) {
                if (!allPlayersOnSpawn && arenaCache.lastPlayersLeftSpawnTick == 0) {
                    if (arenaCache.movementCandidateTick == 0) {
                        arenaCache.movementCandidateTick = curTick;
                    }
                    if (curTick - arenaCache.movementCandidateTick >= PLAYER_MOVEMENT_CONFIRM_TICKS) {
                        arenaCache.lastPlayersLeftSpawnTick = curTick;
                        if (arenaCache.kickoffStartedBeginTick == 0) {
                            arenaCache.kickoffStartedBeginTick = curTick;
                        }
                    }
                } else if (allPlayersOnSpawn) {
                    arenaCache.movementCandidateTick = 0;
                }

                if (arenaCache.lastPlayersLeftSpawnTick == 0) {
                    result = make_result(KickoffState::KickoffRest, 0, false);
                } else {
                    if (arenaCache.kickoffStartedBeginTick == 0) {
                        arenaCache.kickoffStartedBeginTick = arenaCache.lastPlayersLeftSpawnTick;
                    }

                    const uint64_t kickoffDuration = curTick - arenaCache.kickoffStartedBeginTick;
                    if (kickoffDuration >= KICKOFF_TIMEOUT_TICKS) {
                        arenaCache.kickoffTimedOut = true;
                        if (arenaCache.lastBallLeftSpawnTick == 0) {
                            arenaCache.lastBallLeftSpawnTick = curTick;
                        }
                        uint64_t elapsed = curTick - arenaCache.lastBallLeftSpawnTick;
                        const float ballCenterDistSq = b.x * b.x + b.y * b.y;
                        if (elapsed < GRACE_HOLD_TICKS &&
                            ballCenterDistSq <= GRACE_CENTER_EXIT_RADIUS_SQ) {
                            result = make_result(KickoffState::KickoffGrace, GRACE_HOLD_TICKS - elapsed, true);
                        } else {
                            resetKickoffCache();
                            result = make_result(KickoffState::Inactive, 0, false);
                        }
                    } else {
                        result = make_result(KickoffState::KickoffStarted, 0, false);
                    }
                }
            } else {
                if (arenaCache.lastBallLeftSpawnTick == 0) {
                    arenaCache.lastBallLeftSpawnTick = curTick;
                    result = make_result(KickoffState::KickoffGrace, GRACE_HOLD_TICKS, arenaCache.kickoffTimedOut);
                } else {
                    uint64_t elapsed = curTick - arenaCache.lastBallLeftSpawnTick;
                    const float ballCenterDistSq = b.x * b.x + b.y * b.y;
                    if (elapsed < GRACE_HOLD_TICKS &&
                        ballCenterDistSq <= GRACE_CENTER_EXIT_RADIUS_SQ) {
                        result = make_result(KickoffState::KickoffGrace, GRACE_HOLD_TICKS - elapsed,
                                             arenaCache.kickoffTimedOut);
                    } else {
                        resetKickoffCache();
                        result = make_result(KickoffState::Inactive, 0, false);
                    }
                }
            }
        }

        cache.prevKickoffState = result.state;
        cache.cachedTick = curTick;
        cache.cachedResult = result;


        if (result.state != KickoffState::Inactive &&
            (cache.designatedBlue == UINT32_MAX || cache.designatedOrange == UINT32_MAX)) {
            cache.designatedBlue = GetDesignatedKickoffPlayer(state, Team::BLUE);
            cache.designatedOrange = GetDesignatedKickoffPlayer(state, Team::ORANGE);
        } else if (result.state == KickoffState::Inactive) {
            cache.designatedBlue = UINT32_MAX;
            cache.designatedOrange = UINT32_MAX;
        }

        return result;
    }

    struct KickoffSortEntry {
        uint32_t carId;
        float absCenterDist;
        float teamLocalX;
    };

    static bool KickoffPriorityBetter(const KickoffSortEntry &a, const KickoffSortEntry &b) {
        constexpr float MARGIN = 40.0f;
        if (std::abs(a.absCenterDist - b.absCenterDist) > MARGIN) {
            return a.absCenterDist < b.absCenterDist;
        }
        return a.teamLocalX > b.teamLocalX;
    }

    static KickoffSortEntry BuildKickoffSortEntry(const Player &player, Team team) {
        const float teamFlip = (team == Team::ORANGE) ? -1.0f : 1.0f;
        float bestSpawnDist = FLT_MAX;
        float bestSpawnY = 0.0f;
        for (const auto &spawn: RLConst::CAR_SPAWN_LOCATIONS_SOCCAR) {
            const float sx = spawn.x * teamFlip;
            const float sy = spawn.y * teamFlip;
            const float dx = player.pos.x - sx;
            const float dy = player.pos.y - sy;
            const float d = dx * dx + dy * dy;
            if (d < bestSpawnDist) {
                bestSpawnDist = d;
                bestSpawnY = std::abs(sy);
            }
        }

        float localX = player.pos.x;
        if (team == Team::ORANGE) {
            localX = -localX;
        }

        return {player.carId, bestSpawnY, localX};
    }

    static uint32_t GetTopKickoffPriorityPlayer(const GameState &state, Team team) {
        bool hasBest = false;
        KickoffSortEntry best{};

        for (const auto &p: state.players) {
            if (p.team != team) {
                continue;
            }

            const KickoffSortEntry current = BuildKickoffSortEntry(p, team);
            if (!hasBest || KickoffPriorityBetter(current, best)) {
                best = current;
                hasBest = true;
            }
        }

        return hasBest ? best.carId : UINT32_MAX;
    }

    uint32_t KickoffFunctions::GetDesignatedKickoffPlayer(const GameState &state, Team team) {
        return GetTopKickoffPriorityPlayer(state, team);
    }

    std::pair<uint32_t, uint32_t> KickoffFunctions::GetDesignatedKickoffPlayersCached(const GameState &state) {
        const uint64_t curTick = state.lastTickCount;
        const Arena *arenaKey = state.lastArena;

        auto &cache = GetArenaCache(arenaKey);
        std::lock_guard<std::recursive_mutex> lock(cache.mutex);

        if (cache.cachedTick != curTick) {
            IsKickoffResult(state);
        }

        return {cache.designatedBlue, cache.designatedOrange};
    }

    static uint8_t ComputePlayerKickoffRoleInt(const GameState &state,
                                               Team team,
                                               uint32_t playerCarId,
                                               uint32_t designatedCarId) {
        int teammateCount = 0;
        for (const auto &p: state.players) {
            if (p.team == team) {
                teammateCount++;
            }
        }

        if (teammateCount <= 1) {
            return 1;
        }

        if (playerCarId == designatedCarId) {
            return 1;
        }

        static thread_local std::vector<KickoffSortEntry> nonDesignated;
        nonDesignated.clear();
        nonDesignated.reserve(static_cast<size_t>(std::max(teammateCount - 1, 0)));
        for (const auto &p: state.players) {
            if (p.team != team || p.carId == designatedCarId) {
                continue;
            }
            nonDesignated.push_back(BuildKickoffSortEntry(p, team));
        }

        std::sort(nonDesignated.begin(), nonDesignated.end(), KickoffPriorityBetter);

        for (size_t i = 0; i < nonDesignated.size(); i++) {
            if (nonDesignated[i].carId == playerCarId) {
                return i == 0 ? 2 : 3;
            }
        }

        return 3;
    }

    std::string KickoffFunctions::GetPlayerKickoffRole(const GameState &state, const Team team, const Player &player) {
        const uint8_t role = GetPlayerKickoffRoleInt(state, team, player);
        if (role == 1) return "First-Man";
        if (role == 2) return "Second-Man";
        if (role == 3) return "Third-Man";
        return "Inactive";
    }

    uint8_t KickoffFunctions::GetPlayerKickoffRoleInt(const GameState &state, const Team team, const Player &player) {
        const uint64_t curTick = state.lastTickCount;
        const Arena *arenaKey = state.lastArena;

        auto &cache = GetArenaCache(arenaKey);
        std::lock_guard<std::recursive_mutex> lock(cache.mutex);

        if (cache.cachedTick != curTick) {
            IsKickoffResult(state);
        }

        if (!cache.cachedResult.isActive() || cache.prevKickoffState == KickoffState::Inactive) {
            return 0; // Inactive
        }

        int blueCount = 0;
        int orangeCount = 0;
        for (const auto &p: state.players) {
            if (p.team == Team::BLUE) blueCount++;
            else if (p.team == Team::ORANGE) orangeCount++;
        }
        const int myTeamSize = (team == Team::BLUE) ? blueCount : orangeCount;
        const int oppTeamSize = (team == Team::BLUE) ? orangeCount : blueCount;
        if (myTeamSize == 1 && oppTeamSize >= 2) {
            return 0; // Inactive
        }

        auto [designatedBlue, designatedOrange] = GetDesignatedKickoffPlayersCached(state);
        const uint32_t designated = (team == Team::BLUE) ? designatedBlue : designatedOrange;
        return ComputePlayerKickoffRoleInt(state, team, player.carId, designated);
    }

    bool KickoffFunctions::IsPlayerDesignated(const GameState &state, const Player &player) {
        int blueCount = 0;
        int orangeCount = 0;
        for (const auto &p: state.players) {
            if (p.team == Team::BLUE) blueCount++;
            else if (p.team == Team::ORANGE) orangeCount++;
        }

        const int myTeamSize = (player.team == Team::BLUE) ? blueCount : orangeCount;
        const int oppTeamSize = (player.team == Team::BLUE) ? orangeCount : blueCount;

        if (myTeamSize == 1 && oppTeamSize >= 2) {
            return false;
        }

        auto [designatedBlue, designatedOrange] = GetDesignatedKickoffPlayersCached(state);
        return (player.carId == designatedBlue || player.carId == designatedOrange);
    }
}
