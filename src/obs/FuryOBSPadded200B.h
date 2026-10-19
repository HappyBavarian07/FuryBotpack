#pragma once
#include "RLGymCPP/ObsBuilders/ObsBuilder.h"
#include "RLGymCPP/Possession/BallPredictor.h"

namespace RLGC {
    class FuryOBSPadded200B : public ObsBuilder {
    public:
        explicit FuryOBSPadded200B(
            int maxPlayers,
            Vec posCoef = Vec(1 / CommonValues::SIDE_WALL_X, 1 / CommonValues::BACK_WALL_Y,
                              1 / CommonValues::CEILING_Z),
            float velCoef = 1 / CommonValues::CAR_MAX_SPEED,
            float angVelCoef = 1 / CommonValues::CAR_MAX_ANG_VEL,
            bool includeKickoffData = true,
            bool includePredictionData = true,
            int predictionSteps = 4,
            float predictionStepTime = 0.5f,
            bool includeGoalData = true
        );

        void Reset(const GameState &initialState) override;

        void BuildObsInto(const Player &player, const GameState &state, FList &result);

        FList BuildObs(const Player &player, const GameState &state) override;

        int GetObsSize() const;

    private:
        static constexpr int PLAYER_OBS_SIZE = 39;

        void AddKickoffData(FList &obs, const Player &player, const GameState &state, bool inv) const;

        void AddGoalData(FList &obs, const Player &player, const GameState &state, bool inv) const;

        void AddPredictionData(FList &obs, const GameState &state, bool inv) const;

        Vec GetNearestTeammateDirection(const Player &player, const GameState &state);

        void AddPlayerToObs(FList &obs, const Player &player, bool inv, const PhysState &ball, const GameState &state);

    private:
        Vec posCoef;
        float velCoef;
        float angVelCoef;
        int maxPlayers;
        bool includeKickoffData;
        bool includePredictionData;
        int predictionSteps;
        float predictionStepTime;
        bool includeGoalData;
        bool includeModeContextData;

        mutable BallPredictor localBallPredictor;
        mutable BallTrajectory cachedLocalTrajectory;
        mutable uint64_t cachedLocalTrajectoryTick = 0;
        mutable float cachedLocalTrajectoryHorizon = 0.0f;
        mutable bool localTrajectoryValid = false;
    };
}
