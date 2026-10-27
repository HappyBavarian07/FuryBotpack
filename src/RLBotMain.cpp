#include "RLBotClient.h"
#include "obs/KickoffFunctions.h"
#include <GigaLearnCPP/Models.h>
#include <c10/core/InferenceMode.h>
#include <rlbot/BotManager.h>
#include <torch/torch.h>
#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <string_view>

namespace {
std::shared_ptr<const SharedBotContext> spawnContext;

std::unique_ptr<rlbot::Bot> SpawnBot(std::unordered_set<unsigned> indices,
                                  unsigned team, std::string name) noexcept {
    return std::make_unique<RLBotBot>(std::move(indices), team, std::move(name), spawnContext);
}

class FuryManager final : public rlbot::BotManagerBase {
public:
    FuryManager() noexcept : rlbot::BotManagerBase(false, SpawnBot) {}
};

void WarmUp(const SharedBotContext& context) {
    RLGC::GameState state;
    RLGC::Player player{};
    player.carId = 0;
    player.team = Team::BLUE;
    player.pos = Vec(0, -4608, 17);
    player.boost = 33;
    state.ball.pos = Vec(0, 0, RLConst::BALL_REST_Z);
    state.players.push_back(player);
    for (int i = 0; i < 3; ++i) context.inferUnit->InferAction(player, state, true);
    context.obs->Reset(state);
    RLGC::KickoffFunctions::ResetArenaCache(nullptr);
    std::cout << "CPU warm-up complete (3 inference passes; controls discarded)\n";
}

void SelfTest(const SharedBotContext& context) {
    RLGC::BallPredictor predictor;
    RLGC::BallTrajectory trajectory;
    predictor.Predict(trajectory, Vec(3900, 0, 500), Vec(1500, 0, 0), Vec(0, 0, 0), 0.5f);
    if (trajectory.GetSliceAtTime(0.5f).vel.x >= 0.f)
        throw std::runtime_error("RocketSim v3 wall bounce failed");
    for (int teamSize = 1; teamSize <= 3; ++teamSize) {
        RLGC::GameState state;
        state.lastTickCount = teamSize;
        state.ball.pos = Vec(100, 200, 500);
        state.ball.vel = Vec(200, 300, 100);
        for (int team = 0; team < 2; ++team) {
            for (int i = 0; i < teamSize; ++i) {
                RLGC::Player player{};
                player.carId = static_cast<uint32_t>(state.players.size());
                player.index = static_cast<int>(state.players.size());
                player.team = static_cast<Team>(team);
                player.pos = Vec(i * 1000 - 1000, team ? 3000 : -3000, 17);
                player.boost = 33;
                state.players.push_back(player);
            }
        }
        context.obs->Reset(state);
        for (const auto& player : state.players) {
            const auto observation = context.obs->BuildObs(player, state);
            if (observation.size() != 318 || !std::all_of(observation.begin(), observation.end(),
                    [](float value) { return std::isfinite(value); }))
                throw std::runtime_error("Fury observation check failed");
            const auto action = context.inferUnit->InferAction(player, state, true);
            const auto reference = context.inferUnit->BatchInferActions({player}, {state}, true).front();
            if (!std::equal(action.begin(), action.end(), reference.begin()))
                throw std::runtime_error("Fury single/batch inference parity check failed");
            for (float value : action)
                if (!std::isfinite(value) || std::abs(value) > 1.f)
                    throw std::runtime_error("Fury CPU inference check failed");
        }
    }
    std::cout << "PASS: RocketSim v3 wall bounce, 318 observations, CPU policy, both teams in 1v1/2v2/3v3\n";
}

void Benchmark(const SharedBotContext& context, int iterations) {
    struct Sample {
        RLGC::GameState state;
        size_t playerIndex;
    };
    std::vector<Sample> samples;
    samples.reserve(96);
    for (int n = 0; n < 96; ++n) {
        const int teamSize = 1 + (n / 2) % 3;
        const int scenario = (n / 6) % 4;
        Sample sample;
        sample.playerIndex = (n % 2) * teamSize + (n / 24) % teamSize;
        auto& state = sample.state;
        state.deltaTime = 8.f / 120.f;
        state.lastTickCount = (n + 1) * 8;
        state.ball.pos = scenario == 0 ? Vec(0, 0, RLConst::BALL_REST_Z)
            : Vec(scenario == 1 ? 3900.f : (n % 9 - 4) * 300.f, (n % 7 - 3) * 500.f, 300.f + n * 11.f);
        state.ball.vel = scenario == 0 ? Vec(0, 0, 0) : Vec(1300 - n * 13.f, 700 - n * 17.f, 350 - n * 5.f);
        state.ball.angVel = scenario < 2 ? Vec(0, 0, 0) : Vec(2.f, -3.f, 1.f);
        for (int team = 0; team < 2; ++team) {
            const float side = team ? -1.f : 1.f;
            for (int p = 0; p < teamSize; ++p) {
                RLGC::Player player{};
                player.index = static_cast<int>(state.players.size());
                player.carId = static_cast<uint32_t>(player.index);
                player.team = static_cast<Team>(team);
                if (scenario == 0) {
                    const auto& spawn = RLConst::CAR_SPAWN_LOCATIONS_SOCCAR[p];
                    player.pos = Vec(spawn.x * side, spawn.y * side, RLConst::CAR_SPAWN_REST_Z);
                } else {
                    player.pos = Vec((p * 1100 - 900 + n * 3) * side, (-2800 + p * 650) * side,
                                     scenario == 2 ? 500.f + p * 150.f : 17.f);
                    player.vel = Vec(300.f * side, 700.f * side, scenario == 2 ? 250.f : 0.f);
                    player.angVel = scenario == 2 ? Vec(1.f, -0.5f, 0.3f) : Vec(0, 0, 0);
                    player.prevAction = {1, 0.5f, -0.5f, 0.5f, 0, 0, 0, 0};
                }
                player.rotMat = Angle(side * 1.5707963f, scenario == 2 ? 0.3f : 0.f, 0.f).ToRotMat();
                player.boost = scenario == 3 ? static_cast<float>((n + p) % 2) : 33.f + p * 15.f;
                player.isOnGround = scenario != 2;
                player.hasJumped = scenario == 2 && (n + p) % 2 == 0;
                player.airTime = scenario == 2 ? 0.8f : 0.f;
                player.airTimeSinceJump = player.hasJumped ? 0.6f : 0.f;
                state.players.push_back(player);
            }
        }
        for (size_t pad = 0; pad < state.boostPads.size(); ++pad) {
            state.boostPads[pad] = scenario == 0 || (pad + n) % 3 != 0;
            state.boostPadTimers[pad] = state.boostPads[pad] ? 0.f : 0.5f + (pad % 5);
            const size_t inverted = state.boostPads.size() - 1 - pad;
            state.boostPadsInv[inverted] = state.boostPads[pad];
            state.boostPadTimersInv[inverted] = state.boostPadTimers[pad];
        }
        samples.push_back(std::move(sample));
    }
    auto reset = [&] {
        context.obs->Reset(RLGC::GameState{});
        RLGC::KickoffFunctions::ResetArenaCache(nullptr);
    };
    auto forward = [&](torch::Tensor input) {
        torch::NoGradGuard noGrad;
        auto& models = *context.inferUnit->models;
        if (auto* shared = models["shared_head"]) input = shared->Forward(input, false);
        return models["policy"]->Forward(input, false);
    };
    auto hashBytes = [](uint64_t& hash, const void* data, size_t length) {
        const auto* bytes = static_cast<const unsigned char*>(data);
        for (size_t i = 0; i < length; ++i) hash = (hash ^ bytes[i]) * 1099511628211ULL;
    };
    uint64_t obsHash = 14695981039346656037ULL;
    uint64_t logitsHash = obsHash, actionsHash = obsHash;
    std::vector<torch::Tensor> inputs;
    inputs.reserve(samples.size());
    reset();
    for (auto& sample : samples) {
        const auto& player = sample.state.players[sample.playerIndex];
        auto observation = context.obs->BuildObs(player, sample.state);
        if (observation.size() != 318 || !std::all_of(observation.begin(), observation.end(),
                [](float value) { return std::isfinite(value); }))
            throw std::runtime_error("Benchmark observation check failed");
        hashBytes(obsHash, observation.data(), observation.size() * sizeof(float));
        inputs.push_back(torch::tensor(observation).view({1, 318}));
        auto logits = forward(inputs.back()).contiguous();
        {
            c10::InferenceMode inferenceMode;
            const auto guardedLogits = forward(inputs.back());
            if (!torch::equal(logits, guardedLogits))
                throw std::runtime_error("Benchmark NoGrad/InferenceMode logits differ");
        }
        hashBytes(logitsHash, logits.data_ptr<float>(), logits.numel() * sizeof(float));
        const auto action = context.inferUnit->InferAction(player, sample.state, true);
        hashBytes(actionsHash, action.begin(), RLGC::Action::ELEM_AMOUNT * sizeof(float));
    }
    std::cout << "Parity corpus: 96 samples, observations=0x" << std::hex << obsHash
              << " logits=0x" << logitsHash << " actions=0x" << actionsHash << std::dec << '\n';

    auto measure = [&](const char* label, auto&& run) {
        reset();
        std::vector<double> timings;
        timings.reserve(iterations);
        for (int i = 0; i < iterations + 96; ++i) {
            const size_t index = static_cast<size_t>(i) % samples.size();
            auto& sample = samples[index];
            sample.state.lastTickCount = static_cast<uint64_t>(i + 1) * 8;
            const auto start = std::chrono::steady_clock::now();
            run(sample, index);
            const auto finish = std::chrono::steady_clock::now();
            if (i >= 96)
                timings.push_back(std::chrono::duration<double, std::micro>(finish - start).count());
        }
        const double mean = std::accumulate(timings.begin(), timings.end(), 0.0) / timings.size();
        std::sort(timings.begin(), timings.end());
        const size_t middle = timings.size() / 2;
        const double median = timings.size() % 2 ? timings[middle] : (timings[middle - 1] + timings[middle]) / 2;
        const size_t p95 = static_cast<size_t>(std::ceil(timings.size() * 0.95)) - 1;
        std::cout << label << " (us): median=" << std::fixed << std::setprecision(3) << median
                  << " p95=" << timings[p95] << " mean=" << mean << '\n';
    };
    std::cout << "Benchmark: " << iterations << " timed samples/stage, 96 warm-ups, 1 CPU thread; prediction refreshed per decision\n";
    measure("Full InferAction", [&](Sample& sample, size_t) {
        context.inferUnit->InferAction(sample.state.players[sample.playerIndex], sample.state, true);
    });
    measure("Observation + RocketSim v3", [&](Sample& sample, size_t) {
        context.obs->BuildObs(sample.state.players[sample.playerIndex], sample.state);
    });
    measure("Raw shared + policy (NoGrad)", [&](Sample&, size_t index) {
        forward(inputs[index]);
    });
    reset();
}
}

int main(int argc, char** argv) {
    try {
        const bool benchmark = argc >= 2 && std::string_view(argv[1]) == "--benchmark";
        int benchmarkIterations = 1200;
        if (benchmark) {
            if (argc > 3) throw std::invalid_argument("Usage: --benchmark [iterations: 100..1000000]");
            if (argc == 3) {
                const std::string_view count(argv[2]);
                const auto parsed = std::from_chars(count.data(), count.data() + count.size(), benchmarkIterations);
                if (parsed.ec != std::errc{} || parsed.ptr != count.data() + count.size() ||
                    benchmarkIterations < 100 || benchmarkIterations > 1000000)
                    throw std::invalid_argument("Benchmark iterations must be an integer from 100 to 1000000");
            }
        }
        torch::set_num_threads(1);
        torch::set_num_interop_threads(1);
        auto modelDir = std::filesystem::absolute(argv[0]).parent_path();
        if (modelDir.filename() == "000-runtime") modelDir = modelDir.parent_path();
        if (!std::filesystem::is_regular_file(modelDir / "POLICY.lt") ||
            !std::filesystem::is_regular_file(modelDir / "SHARED_HEAD.lt"))
            throw std::runtime_error("Fury POLICY.lt and SHARED_HEAD.lt must be beside bot.toml");
        if (!RLGC::BallPredictor::InitV3Backend(modelDir))
            throw std::runtime_error("RocketSim v3 initialization failed; check collision_meshes/soccar");

        auto context = std::make_shared<SharedBotContext>();
        context->obs = std::make_shared<RLGC::FuryOBSPadded200B>(3,
            Vec(1 / RLGC::CommonValues::SIDE_WALL_X, 1 / RLGC::CommonValues::BACK_NET_Y,
                1 / RLGC::CommonValues::CEILING_Z));
        context->act = std::make_shared<RLGC::AdvancedActionOld>(4, 5, 5, 7, true);
        context->params = {8, 7};

        GGL::InferPartialModelConfig sharedHead;
        sharedHead.layerSizes = {1536, 1536};
        sharedHead.addOutputLayer = false;
        GGL::InferPartialModelConfig policy;
        policy.layerSizes = {1024, 1024, 1024};
        // Both use the default ReLU + LayerNorm, exactly as Fury 200B was trained.
        context->inferUnit = std::make_shared<GGL::InferUnit>(context->obs.get(), 318,
            context->act.get(), sharedHead, policy, modelDir, false);
        WarmUp(*context);
        std::cout << "Fury 200B | CPU PyTorch | RocketSim v3 ball prediction | tickSkip 8 / delay 7\n";
        if (argc == 2 && std::string(argv[1]) == "--self-test") {
            SelfTest(*context);
            RLBotBot::SelfTest(context);
            return EXIT_SUCCESS;
        }
        if (benchmark) {
            Benchmark(*context, benchmarkIterations);
            return EXIT_SUCCESS;
        }
        spawnContext = context;
        const char* host = std::getenv("RLBOT_SERVER_IP");
        const char* port = std::getenv("RLBOT_SERVER_PORT");
        const char* agentId = std::getenv("RLBOT_AGENT_ID");
        FuryManager manager;
        if (!manager.connect(host ? host : "127.0.0.1", port ? port : "23234",
                             agentId ? agentId : "happybavarian07/fury", false))
            return EXIT_FAILURE;
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "Fury: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
