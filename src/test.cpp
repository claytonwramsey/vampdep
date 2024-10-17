#include <array>
#include <vamp/collision/environment.hh>
#include <vamp/planning/plan.hh>
#include <vamp/planning/rrtc.hh>
#include <vamp/planning/rrtc_settings.hh>
#include <vamp/random/halton.hh>
#include <vamp/robots/sphere.hh>
#include <vamp/vector.hh>

int main(void) {

    const vamp::collision::Environment<vamp::FloatVector<8>> env;
    vamp::planning::RRTC<vamp::robots::Sphere, vamp::rng::Halton<3>, 8, 8> rrtc;
    vamp::FloatVector<3> start;
    vamp::FloatVector<3> goal(std::array<float, 3> { 1.0, 1.0, 1.0 });
    vamp::planning::RRTCSettings settings;
    const vamp::planning::PlanningResult<3> res
        = rrtc.solve(vamp::robots::Sphere::Configuration(start),
            vamp::robots::Sphere::Configuration(goal), env, settings);

    if (res.path.size() > 1) {
        std::cout << "found path! [";
        for (const auto &cfg : res.path) {
            std::cout << cfg << ", ";
        }
        std::cout << "]" << std::endl;
    } else {
        std::cout << "failed to find a path :(" << std::endl;
    }
    return 0;
}
