#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChSystemNSC.h"

#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/solver/ChIterativeSolverVI.h"
#include "chrono/solver/ChDirectSolverLS.h"
#include "chrono/solver/ChSolverPSOR.h"
#include "chrono/solver/ChSolverBB.h"

#include "chrono_thirdparty/filesystem/path.h"

#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif
#ifdef CHRONO_MUMPS
    #include "chrono_mumps/ChSolverMumps.h"
#endif

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "chrono_vehicle/chassis/ChRigidChassis.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicle.h"
#include "chrono_vehicle/tracked_vehicle/track_assembly/ChTrackAssemblySegmented.h"

#include "chrono_models/vehicle/m113/track_shoe/M113_TrackShoeDoublePin.h"
#include "chrono_models/vehicle/m113/track_shoe/M113_TrackShoeSinglePin.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::m113;

using std::cout;
using std::endl;

// =============================================================================

////TrackShoeType shoe_type = TrackShoeType::SINGLE_PIN;
TrackShoeType shoe_type = TrackShoeType::DOUBLE_PIN;
int num_shoes = 3;
bool RSDA = true;
int fixed_shoe = 0;

double step_size = 5e-4;
double g = -10;

////ChSolver::Type solver_type = ChSolver::Type::BARZILAIBORWEIN;
////ChSolver::Type solver_type = ChSolver::Type::MINRES;
////ChSolver::Type solver_type = ChSolver::Type::GMRES;
////ChSolver::Type solver_type = ChSolver::Type::SPARSE_LU;
ChSolver::Type solver_type = ChSolver::Type::SPARSE_QR;
////ChSolver::Type solver_type = ChSolver::Type::PARDISO_MKL;
////ChSolver::Type solver_type = ChSolver::Type::MUMPS;

////ChTimestepper::Type integrator_type = ChTimestepper::Type::EULER_IMPLICIT;
////ChTimestepper::Type integrator_type = ChTimestepper::Type::EULER_IMPLICIT_PROJECTED;
ChTimestepper::Type integrator_type = ChTimestepper::Type::HHT;

bool verbose_solver = false;
bool verbose_integrator = true;

const std::string out_dir = GetChronoOutputPath() + "TRACK_TEST";

// =============================================================================

class Chassis : public ChRigidChassis {
  public:
    Chassis() : ChRigidChassis("Ground") {}
    virtual double GetBodyMass() const override { return 1; }
    virtual ChMatrix33<> GetBodyInertia() const override { return ChMatrix33<>(1); }
    virtual ChFrame<> GetBodyCOMFrame() const override { return ChFrame<>(VNULL, QUNIT); }
    virtual ChCoordsys<> GetLocalDriverCoordsys() const override { return ChCoordsys<>(); }
};

class Assembly : public ChTrackAssemblySegmented {
  public:
    Assembly(int nshoes, bool add_RSDA)
        : ChTrackAssemblySegmented("Assembly", VehicleSide::LEFT), m_nshoes(nshoes), m_radius(0), m_fixed(false) {
        if (add_RSDA) {
            double k = 100;
            double c = 10;
            m_torque_funct = chrono_types::make_shared<ChTrackAssemblySegmented::TrackBendingFunctor>(k, c);
        }
    }

    void SetFixed(int val) { m_fixed = val; }
    void Initialize(std::shared_ptr<ChChassis> chassis);
    double GetRadius() const { return m_radius; }
    virtual void CreateShoes() = 0;

  protected:
    int m_nshoes;
    int m_fixed;
    double m_radius;
    ChTrackShoeList m_shoes;

  private:
    virtual std::string GetTemplateName() const override { return "None"; }
    virtual size_t GetNumTrackShoes() const override { return m_nshoes; }
    virtual std::shared_ptr<ChSprocket> GetSprocket() const override { return nullptr; }
    virtual std::shared_ptr<ChTrackShoe> GetTrackShoe(size_t id) const override { return m_shoes[id]; }
    virtual const ChVector3d GetSprocketLocation() const override { return ChVector3d(0); }
    virtual const ChVector3d GetIdlerLocation() const override { return ChVector3d(0); }
    virtual const ChVector3d GetRoadWhelAssemblyLocation(int which) const override { return ChVector3d(0); }
    virtual const ChVector3d GetRollerLocation(int which) const override { return ChVector3d(0); }
    virtual bool Assemble(std::shared_ptr<ChBodyAuxRef> chassis) override { return true; }
    virtual void RemoveTrackShoes() override {}
};

void Assembly::Initialize(std::shared_ptr<ChChassis> chassis) {
    // Create shoes
    CreateShoes();

    // Place track shoes in a closed circle
    double del_angle = CH_2PI / m_nshoes;
    double length = m_shoes[0]->GetPitch();
    m_radius = (length / 2) / std::tan(del_angle / 2);

    ChVector3d loc;
    ChQuaternion<> rot;
    double angle = 0;

    for (int i = 0; i < m_nshoes; i++) {
        loc.x() = m_radius * std::sin(angle);
        loc.z() = m_radius * std::cos(angle);
        rot = QuatFromAngleY(angle);

        m_shoes[i]->SetIndex(i);
        m_shoes[i]->Initialize(chassis->GetBody(), loc, rot);
        m_shoes[i]->SetVisualizationType(VisualizationType::PRIMITIVES);
        angle += del_angle;
    }

    // Connect shoes to each other
    for (size_t i = 0; i < num_shoes; ++i) {
        auto next = (i == num_shoes - 1) ? GetTrackShoe(0) : GetTrackShoe(i + 1);
        GetTrackShoe(i)->Connect(next, this, chassis.get(), true);
    }

    if (m_fixed >= 0)
        m_shoes[m_fixed]->GetShoeBody()->SetFixed(true);
}

class AssemblySP : public Assembly {
  public:
    AssemblySP(int nshoes, bool add_RSDA) : Assembly(nshoes, add_RSDA) {}

    virtual void CreateShoes() override {
        for (int i = 0; i < m_nshoes; i++) {
            m_shoes.push_back(chrono_types::make_shared<M113_TrackShoeSinglePin>("shoe_" + std::to_string(i)));
        }
    }
};

class AssemblyDP : public Assembly {
  public:
    AssemblyDP(int nshoes, bool add_RSDA) : Assembly(nshoes, add_RSDA) {}

    virtual void CreateShoes() override {
        for (int i = 0; i < m_nshoes; i++) {
            m_shoes.push_back(chrono_types::make_shared<M113_TrackShoeDoublePin>(
                "shoe_" + std::to_string(i), DoublePinTrackShoeType::ONE_CONNECTOR));
        }
    }
};

// =============================================================================

int main(int argc, char* argv[]) {
    // System, solver and integrator
    ChSystemSMC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, g));

    if (solver_type == ChSolver::Type::PARDISO_MKL) {
#ifndef CHRONO_PARDISO_MKL
        solver_type = ChSolver::Type::SPARSE_QR;
#endif
    } else if (solver_type == ChSolver::Type::MUMPS) {
#ifndef CHRONO_MUMPS
        solver_type = ChSolver::Type::SPARSE_QR;
#endif
    }

    if (solver_type == ChSolver::Type::PARDISO_MKL) {
#ifdef CHRONO_PARDISO_MKL
        std::cout << "Using Pardiso MKL solver" << std::endl;
        auto solver = chrono_types::make_shared<ChSolverPardisoMKL>();
        solver->LockSparsityPattern(true);
        sys.SetSolver(solver);
#endif
    } else if (solver_type == ChSolver::Type::MUMPS) {
#ifdef CHRONO_MUMPS
        std::cout << "Using MUMPS solver" << std::endl;
        auto solver = chrono_types::make_shared<ChSolverMumps>();
        solver->LockSparsityPattern(true);
        solver->EnableNullPivotDetection(true);
        solver->GetMumpsEngine().SetICNTL(14, 50);
        sys.SetSolver(solver);
#endif
    } else {
        sys.SetSolverType(solver_type);
        switch (solver_type) {
            case ChSolver::Type::SPARSE_LU:
            case ChSolver::Type::SPARSE_QR: {
                std::cout << "Using a direct sparse LS solver" << std::endl;
                auto solver = std::static_pointer_cast<ChDirectSolverLS>(sys.GetSolver());
                solver->LockSparsityPattern(false);
                solver->UseSparsityPatternLearner(false);
                break;
            }
            case ChSolver::Type::BARZILAIBORWEIN:
            case ChSolver::Type::APGD:
            case ChSolver::Type::PSOR: {
                std::cout << "Using an iterative VI solver" << std::endl;
                auto solver = std::static_pointer_cast<ChIterativeSolverVI>(sys.GetSolver());
                solver->SetMaxIterations(100);
                solver->SetOmega(0.8);
                solver->SetSharpnessLambda(1.0);

                ////sys.SetMaxPenetrationRecoverySpeed(1.5);
                break;
            }
            case ChSolver::Type::BICGSTAB:
            case ChSolver::Type::MINRES:
            case ChSolver::Type::GMRES: {
                std::cout << "Using an iterative LS solver" << std::endl;
                auto solver = std::static_pointer_cast<ChIterativeSolverLS>(sys.GetSolver());
                solver->SetMaxIterations(200);
                solver->SetTolerance(1e-10);
                solver->EnableDiagonalPreconditioner(true);
                break;
            }
        }
    }
    sys.GetSolver()->SetVerbose(verbose_solver);

    sys.SetTimestepperType(integrator_type);
    switch (integrator_type) {
        case ChTimestepper::Type::HHT: {
            auto integrator = std::static_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
            integrator->SetAlpha(-0.2);
            integrator->SetMaxIters(50);
            integrator->SetAbsTolerances(1e-4, 1e2);
            integrator->SetStepControl(false);
            integrator->SetModifiedNewton(false);
            break;
        }

        case ChTimestepper::Type::EULER_IMPLICIT: {
            auto integrator = std::static_pointer_cast<ChTimestepperEulerImplicit>(sys.GetTimestepper());
            integrator->SetMaxIters(50);
            integrator->SetAbsTolerances(1e-4, 1e2);
            break;
        }
    }
    sys.GetTimestepper()->SetVerbose(verbose_integrator);

    // Create tracked vehicle subsystems
    auto chassis = chrono_types::make_shared<Chassis>();
    chassis->Initialize(&sys, ChCoordsys<>(), 0);
    chassis->SetFixed(true);

    // Create the track assembly
    std::shared_ptr<Assembly> track;
    if (shoe_type == TrackShoeType::SINGLE_PIN)
        track = chrono_types::make_shared<AssemblySP>(num_shoes, RSDA);
    else
        track = chrono_types::make_shared<AssemblyDP>(num_shoes, RSDA);
    track->SetFixed(fixed_shoe);
    track->Initialize(chassis);
    auto radius = track->GetRadius();

    // Create the Irrlicht application
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->SetWindowSize(800, 600);
    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->SetWindowTitle("Track test");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector3d(0, 1, radius), ChVector3d(0, 0, radius));
    vis->AddTypicalLights();
    vis->AttachSystem(&sys);

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    ////sys.EnableSolverMatrixWrite(true, out_dir);

    // Simulation loop
    int frame = 0;
    while (vis->Run()) {
        std::cout << sys.GetChTime() << " ========================================" << std::endl;
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        sys.DoStepDynamics(step_size);
        frame++;

        ////if (frame > 2)
        ////    break;
    }
}
