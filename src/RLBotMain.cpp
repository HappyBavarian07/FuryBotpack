#include "RLBotClient.h"
#include "obs/KickoffFunctions.h"
#include <rlbot/BotManager.h>
#include <torch/torch.h>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>

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
            for (float value : action)
                if (!std::isfinite(value) || std::abs(value) > 1.f)
                    throw std::runtime_error("Fury CPU inference check failed");
        }
    }
    std::cout << "PASS: RocketSim v3 wall bounce, 318 observations, CPU policy, both teams in 1v1/2v2/3v3\n";
}
}

int main(int argc, char** argv) {
    try {
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
        spawnContext = context;
        const char* host = std::getenv("RLBOT_SERVER_IP");
        const char* port = std::getenv("RLBOT_SERVER_PORT");
        const char* agentId = std::getenv("RLBOT_AGENT_ID");
        FuryManager manager;
        if (!manager.connect(host ? host : "127.0.0.1", port ? port : "23234",
                             agentId ? agentId : "happybavarian07/fury-200b", false))
            return EXIT_FAILURE;
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "Fury: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
