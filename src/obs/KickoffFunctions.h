#pragma once
#include <RLGymCPP/GameStates/GameState.h>
#include <cstdint>
#include <string>
#include <utility>

namespace RLGC {
    enum class KickoffState {
        Inactive = 0,
        KickoffRest = 1,
        KickoffStarted = 2,
        KickoffGrace = 3
    };

    inline std::string KickoffStateToString(KickoffState s) {
        switch (s) {
            case KickoffState::Inactive: return "Inactive";
            case KickoffState::KickoffRest: return "KickoffRest";
            case KickoffState::KickoffStarted: return "KickoffStarted";
            case KickoffState::KickoffGrace: return "KickoffGrace";
        }
        return "Unknown";
    }

    struct KickoffResult {
        KickoffState state;
        uint64_t remainingHoldTicks;
        bool timedOut;

        bool operator==(const KickoffState kickoff_state) const { return state == kickoff_state; }
        bool isActive() const { return state != KickoffState::Inactive; }

        float NormalizeKickoffState() const {
            return static_cast<float>(state) / 3.0f;
        }
    };

    struct KickoffTimingStats {
        uint64_t restStartTick = 0;
        uint64_t movementStartTick = 0;
        uint64_t firstTouchTick = 0;
        uint64_t endTick = 0;

        float restDuration = 0.0f;
        float approachDuration = 0.0f;
        float contactToEndDuration = 0.0f;
        float totalDuration = 0.0f;

        bool hasMovementStart = false;
        bool hasFirstTouch = false;
        bool hasEnded = false;

        void Reset() {
            restStartTick = 0;
            movementStartTick = 0;
            firstTouchTick = 0;
            endTick = 0;
            restDuration = 0.0f;
            approachDuration = 0.0f;
            contactToEndDuration = 0.0f;
            totalDuration = 0.0f;
            hasMovementStart = false;
            hasFirstTouch = false;
            hasEnded = false;
        }
    };

    class KickoffFunctions {
    public:
        static KickoffResult IsKickoffResult(const GameState &state);

        /// Clears the per-arena kickoff tracking state. Offline tools that reuse one arena key
        /// across many replays call this at each replay boundary.
        static void ResetArenaCache(const Arena *arenaKey);

        static uint32_t GetDesignatedKickoffPlayer(const GameState &state, Team team);

        static std::pair<uint32_t, uint32_t> GetDesignatedKickoffPlayersCached(const GameState &state);

        static std::string GetPlayerKickoffRole(const GameState &state, Team team, const Player &player);

        static uint8_t GetPlayerKickoffRoleInt(const GameState &state, Team team, const Player &player);

        static bool IsPlayerDesignated(const GameState &state, const Player &player);
    };
}
