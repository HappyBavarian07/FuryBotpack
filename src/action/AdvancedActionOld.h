#pragma once
#include <vector>
#include <utility>

#include "RLGymCPP/ActionParsers/ActionParser.h"

namespace RLGC {
    class AdvancedActionOld final : public ActionParser {
    public:
        explicit AdvancedActionOld(int throttle_bins = 3,
                                   int steer_bins = 3,
                                   int torque_subdivisions = 2,
                                   int flip_bins = 8,
                                   bool include_stalls = false);

        void EnableXMirror(bool enable);

        Action ParseAction(int actionIdx, const Player &player, const GameState &state) override;

        int GetActionAmount() override;

        int FindClosestActionIndex(const Action &action) const;

    private:
        bool xMirrorEnabled = false;
        std::vector<RLGC::Action> _lookup_table;
        std::vector<RLGC::Action> _mirror_lookup_table;

        static std::vector<float> parse_bin(int bins, bool endpoint = true);

        static std::vector<std::pair<float, float> > make_torque_face(int subdivisions);

        static std::vector<Action> make_lookup_table(int throttle_bins,
                                                     int steer_bins,
                                                     int torque_subdivisions,
                                                     int flip_bins,
                                                     bool include_stalls);
    };
} // namespace RLGSC
