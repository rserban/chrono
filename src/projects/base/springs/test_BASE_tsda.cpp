// ======================================================================
// Last update by Saman Nezami at 05/12/2021
// - Testing the TSDA link motion
//=======================================================================

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkTSDA.h"
#include "chrono/physics/ChSystemNSC.h"

// Use the namespaces of Chrono
using namespace chrono;
// ===== the main code =========
int main(int, char*[]) {
    ChSystemNSC my_system;

    std::shared_ptr<ChBody> anchor;
    std::shared_ptr<ChLinkTSDA> link_shock;
    std::shared_ptr<ChBody> mass;

    my_system.SetGravitationalAcceleration(ChVector3d(0., 0., 0.));
    double weight = 1.;  // it seems 2.*M_PI/1000 gives a close to 1 hz, but why!?  kg
    double Shock_SpringCoeff = 4. * CH_PI * CH_PI;
    double Shock_DampingCoeff = 0.;

    // --- spring, mass and anchor ---
    ChVector3d anchor_initPos(0., 0., -1.);
    ChVector3d mass_initPos(0., 0., 0.);
    ChVector3d mass_intiVel(0., 0., 2. * CH_PI);

    anchor = chrono_types::make_shared<ChBody>();
    anchor->SetName("Anchor");
    anchor->SetPos(anchor_initPos);
    anchor->SetMass(1.);
    anchor->SetFixed(true);
    my_system.AddBody(anchor);

    mass = chrono_types::make_shared<ChBody>();
    mass->SetName("Mass");
    mass->SetPos(mass_initPos);
    mass->SetPosDt(mass_intiVel);
    mass->SetMass(weight);
    my_system.AddBody(mass);

    // Massless, linear spring-damper shock
    link_shock = chrono_types::make_shared<ChLinkTSDA>();
    link_shock->SetName("Shock");
    link_shock->Initialize(anchor, mass, false, anchor->GetPos(), mass->GetPos());
    link_shock->SetRestLength(0.0);
    link_shock->SetSpringCoefficient(Shock_SpringCoeff);
    link_shock->SetDampingCoefficient(Shock_DampingCoeff);
    my_system.AddLink(link_shock);

    std::ofstream io("tsda.out");

    my_system.GetSolver()->AsIterative()->SetMaxIterations(1000);
    ChRealtimeStepTimer realtime_timer;
    const double simulation_step = 1e-3;
    while (my_system.GetChTime() < 4) {
        my_system.DoStepDynamics(simulation_step);
        realtime_timer.Spin(simulation_step);

        io << my_system.GetChTime()  << " " << mass->GetCoordsys().pos.z() << "\n";
    }

    return 0;
}
