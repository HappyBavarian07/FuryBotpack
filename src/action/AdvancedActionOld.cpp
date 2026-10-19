#include "AdvancedActionOld.h"

#include "RLGymCPP/ActionParsers/ActionParser.h"
#include <array>
#include <algorithm>
#include <limits>

#include <cmath>
#include <iostream>
#include <set>
#include <mutex>
#include <ostream>

#include "RLGymCPP/GameStates/GameState.h"

namespace RLGC {
    std::vector<float> AdvancedActionOld::parse_bin(const int bins, const bool endpoint) {
        std::vector<float> v(bins);

        for (int i = 0; i < bins; ++i) {
            const float t = (endpoint ? static_cast<float>(i) / (bins - 1) : static_cast<float>(i) / bins);
            v[i] = -1.f + 2.f * t;
        }
        return v;
    }

    // Build one face of torque grid
    std::vector<std::pair<float, float> > AdvancedActionOld::make_torque_face(const int subdivisions) {
        const int count = 1 + static_cast<int>(std::pow(2, subdivisions / 2));
        auto bins = parse_bin(count);
        const float delta = (bins.size() > 1 ? bins[1] - bins[0] : 2.f / count);
        std::vector<std::pair<float, float> > face;
        // square step
        for (float x: bins)
            for (float y: bins)
                face.emplace_back(x, y);
        // diamond step
        if (subdivisions % 2 == 1) {
            std::vector<float> db(bins.begin(), bins.end() - 1);
            for (auto &u: db) u += delta / 2.f;

            for (float x: db)
                for (float y: db)
                    face.emplace_back(x, y);
        }

        return face;
    }

    // Full lookup‐table builder
    std::vector<Action> AdvancedActionOld::make_lookup_table(const int throttle_bins,
                                                             const int steer_bins,
                                                             const int torque_subdivisions,
                                                             const int flip_bins,
                                                             const bool include_stalls) {
        std::vector<Action> acts;
        // ground
        const auto thr = parse_bin(throttle_bins);
        auto str = parse_bin(steer_bins);
        for (float t: thr)
            for (float s: str)
                for (int b = 0; b < 2; ++b)
                    for (int h = 0; h < 2; ++h) {
                        if (b == 1 && t != 1.f) continue;

                        acts.emplace_back(
                            t,
                            s,
                            0.f,
                            s,
                            0.f,
                            0.f,
                            static_cast<float>(b),
                            static_cast<float>(h)
                        );
                    }

        // aerial
        auto face = make_torque_face(torque_subdivisions);
        std::set<std::array<float, 3> > uniques;
        std::vector<std::array<float, 3> > points;

        for (auto &[fst, snd]: face) {
            for (int axis = 0; axis < 3; ++axis) {
                for (const int side: {-1, 1}) {
                    std::array<float, 3> arr;
                    // fill pitch/yaw/roll
                    for (int d = 0; d < 3; ++d) {
                        if (d == axis)
                            arr[d] = static_cast<float>(side);
                        else
                            arr[d] = (d == 0 ? fst : snd);
                    }

                    if (uniques.insert(arr).second)
                        points.push_back(arr);
                }
            }
        }

        for (auto &p: points) {
            float pitch = p[0], yaw = p[1], roll = p[2];
            // skip duplicates with ground
            if (pitch == 0 && roll == 0
                && std::ranges::find(str, yaw) != str.end())
                continue;
            for (int b = 0; b < 2; ++b) {
                acts.push_back({static_cast<float>(b), yaw, pitch, yaw, roll, 0.f, static_cast<float>(b), 0.f});
            }
        }

        // flips and jumps
        auto flips = parse_bin(flip_bins, false);
        flips.insert(flips.begin(), std::nanf("")); // empty jump
        for (const float ang: flips) {
            float p = 0.f, r = 0.f;

            if (!std::isnan(ang)) {
                p = std::sin(ang);
                r = std::cos(ang);

                const float mag = std::max(std::abs(p), std::abs(r));
                p /= mag;
                r /= mag;
            }

            for (int b = 0; b < 2; ++b) {
                acts.emplace_back(
                    static_cast<float>(b),
                    0.f,
                    p,
                    0.f,
                    r,
                    1.f,
                    static_cast<float>(b),
                    1.f
                );
            }
        }

        if (include_stalls) {
            acts.emplace_back(0, 0, 0, 1, -1, 1, 0, 1);
            acts.emplace_back(0, 0, 0, -1, 1, 1, 0, 1);
        }

        return acts;
    }

    AdvancedActionOld::AdvancedActionOld(const int throttle_bins, const int steer_bins, const int torque_subdivisions,
                                         const int flip_bins, const bool include_stalls) {
        _lookup_table = make_lookup_table(
            throttle_bins, steer_bins, torque_subdivisions,
            flip_bins, include_stalls
        );

        static std::once_flag flag;
        std::call_once(flag, [&] {
            RG_LOG("AdvancedActionOld(): Lookup table built, action count: "
                << _lookup_table.size());
        });
    }

    void AdvancedActionOld::EnableXMirror(const bool enable) {
        this->xMirrorEnabled = enable;

        if (enable) {
            _mirror_lookup_table.resize(_lookup_table.size());

            for (size_t i = 0; i < _lookup_table.size(); ++i) {
                const Action original = _lookup_table[i];
                Action copy{};

                copy.throttle = original.throttle;
                copy.steer = -original.steer;
                copy.pitch = original.pitch;
                copy.yaw = -original.yaw;
                copy.roll = -original.roll;
                copy.jump = original.jump;
                copy.boost = original.boost;
                copy.handbrake = original.handbrake;

                _mirror_lookup_table[i] = copy;
            }
        }
    }

    Action AdvancedActionOld::ParseAction(const int actionIdx, const Player &player, const GameState &state) {
        if (actionIdx < 0 || actionIdx >= static_cast<int>(_lookup_table.size())) {
            Action result;
            result.throttle = 0.f;
            result.steer = 0.f;
            result.pitch = 0.f;
            result.yaw = 0.f;
            result.roll = 0.f;
            result.jump = 0.f;
            result.boost = 0.f;
            result.handbrake = 0.f;
            return result;
        }
        if (this->xMirrorEnabled) {
            float x = player.pos.x;

            if (player.team == Team::ORANGE)
                x = -x;

            if (x < 0.f) {
                return _mirror_lookup_table[actionIdx];
            } else {
                return _lookup_table[actionIdx];
            }
        }
        return _lookup_table[actionIdx];
    }

    int AdvancedActionOld::GetActionAmount() {
        return static_cast<int>(_lookup_table.size());
    }

    int AdvancedActionOld::FindClosestActionIndex(const Action &action) const {
        int bestIndex = 0;
        float bestDistance = std::numeric_limits<float>::max();

        for (size_t i = 0; i < _lookup_table.size(); ++i) {
            const Action &tableAction = _lookup_table[i];

            // Calculate squared Euclidean distance
            float distance = 0.0f;
            distance += (action.throttle - tableAction.throttle) * (action.throttle - tableAction.throttle);
            distance += (action.steer - tableAction.steer) * (action.steer - tableAction.steer);
            distance += (action.pitch - tableAction.pitch) * (action.pitch - tableAction.pitch);
            distance += (action.yaw - tableAction.yaw) * (action.yaw - tableAction.yaw);
            distance += (action.roll - tableAction.roll) * (action.roll - tableAction.roll);
            distance += (action.jump - tableAction.jump) * (action.jump - tableAction.jump);
            distance += (action.boost - tableAction.boost) * (action.boost - tableAction.boost);
            distance += (action.handbrake - tableAction.handbrake) * (action.handbrake - tableAction.handbrake);

            if (distance < bestDistance) {
                bestDistance = distance;
                bestIndex = static_cast<int>(i);
            }
        }

        return bestIndex;
    }
} // namespace RLGSC
