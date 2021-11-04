// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Milad Rakhsha, Arman Pazouki, Wei Hu
// =============================================================================
//
// Implementation of FSI system that includes all subclasses for proximity and
// force calculation, and time integration.
//
// =============================================================================

#ifndef CH_SYSTEMFSI_H_
#define CH_SYSTEMFSI_H_

#include "chrono/ChConfig.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_fsi/physics/ChBce.cuh"
#include "chrono_fsi/physics/ChFluidDynamics.cuh"
#include "chrono_fsi/physics/ChFsiGeneral.h"
#include "chrono_fsi/ChFsiInterface.h"
#include "chrono_fsi/ChFsiDefines.h"
#include "chrono_fsi/utils/ChUtilsPrintSph.cuh"
#include "chrono_fsi/utils/ChUtilsJSON.h"

namespace chrono {

// Forward declaration
namespace fea {
class ChNodeFEAxyzD;
class ChMesh;
class ChElementCableANCF;
class ChElementShellANCF_3423;
}  // namespace fea

namespace fsi {

class ChSystemFsi_impl;

/// @addtogroup fsi_physics
/// @{

/// @brief Physical system for fluid-solid interaction problems.
///
/// This class is used to represent fluid-solid interaction problems consisting of
/// fluid dynamics and multibody system. Each of the two underlying physics is
/// an independent object owned and instantiated by this class. Additionally,
/// the fsi system owns other objects to handle the interface between the two
/// systems, boundary condition enforcing markers, and data.
class CH_FSI_API ChSystemFsi {
  public:
    /// Constructor for FSI system.
    /// This class constructor instantiates all the member objects. Wherever relevant, the
    /// instantiation is handled by sending a pointer to other objects or data.
    /// Therefore, the sub-classes have pointers to the same data.
    ChSystemFsi(ChSystem& other_physicalSystem, 
                CHFSI_TIME_INTEGRATOR time_integrator = CHFSI_TIME_INTEGRATOR::ExplicitSPH);

    /// Destructor for the FSI system.
    virtual ~ChSystemFsi();

    /// Function to integrate the fsi system in time.
    /// It uses a Runge-Kutta 2nd order algorithm to update both the fluid and multibody
    /// system dynamics. The midpoint data of MBS is needed for fluid dynamics update.
    virtual void DoStepDynamics_FSI();

    /// Function to integrate the multibody system dynamics based on Runge-Kutta
    /// 2nd-order integration scheme.
    virtual void DoStepDynamics_ChronoRK2();

    /// Function to initialize the midpoint device data of the fluid system by
    /// copying from the full step
    virtual void CopyDeviceDataToHalfStep();

    /// Fill out the dependent data based on the independent one. For instance,
    /// it copies the position of the rigid bodies from the multibody dynamics system
    /// to arrays in fsi system since they are needed for internal use.
    virtual void FinalizeData();

    /// Get a pointer to the data manager.
    std::shared_ptr<ChSystemFsi_impl> GetFsiData() { return fsiSystem; }

    /// Set the linear system solver for implicit methods
    void SetFluidSystemLinearSolver(ChFsiLinearSolver::SolverType other_solverType) {
        fluidDynamics->GetForceSystem()->SetLinearSolver(other_solverType);
    }

    /// Set the SPH method to be used for fluid dynamics
    void SetFluidDynamics(fluid_dynamics params_type = fluid_dynamics::I2SPH);

    /// Get a pointer to the parameters used to set up the simulation.
    std::shared_ptr<SimParams> GetSimParams() { return paramsH; }

    /// Get a reference to the fsi bodies.
    /// fsi bodies are the ones seen by the fluid dynamics system.
    std::vector<std::shared_ptr<ChBody>>& GetFsiBodies() { return fsiBodies; }

    /// Get a reference to the fsi ChElementCableANCF.
    /// fsi ChElementCableANCF are the ones seen by the fluid dynamics system.
    std::vector<std::shared_ptr<fea::ChElementCableANCF>>& GetFsiCables() { return fsiCables; }

    /// Get a reference to the fsi ChElementShellANCF_3423.
    /// fsi ChElementShellANCF_3423 are the ones seen by the fluid dynamics system.
    std::vector<std::shared_ptr<fea::ChElementShellANCF_3423>>& GetFsiShells() { return fsiShells; }

    /// Get a reference to the fsi ChNodeFEAxyzD.
    /// fsi ChNodeFEAxyzD are the ones seen by the fluid dynamics system.
    std::vector<std::shared_ptr<fea::ChNodeFEAxyzD>>& GetFsiNodes() { return fsiNodes; }

    /// Adds FSI body to the FsiSystem
    void AddFsiBody(std::shared_ptr<ChBody> mbody) { fsiBodies.push_back(mbody); }

    /// Finzalizes data by calling FinalizeData function and finalize fluid and bce
    /// and also finalizes the fluid and bce objects if the system have fluid.
    virtual void Finalize();

    /// Finalizes the construction of cable elements in the FSI system.
    void SetCableElementsNodes(std::vector<std::vector<int>> elementsNodes) {
        CableElementsNodes = elementsNodes;
        size_t test = fsiSystem->fsiGeneralData->CableElementsNodes.size();
        std::cout << "numObjects.numFlexNodes" << test << std::endl;
    }

    /// Finalizes the construction of cable elements in the FSI system.
    void SetShellElementsNodes(std::vector<std::vector<int>> elementsNodes) {
        ShellElementsNodes = elementsNodes;
        size_t test = fsiSystem->fsiGeneralData->ShellElementsNodes.size();
        std::cout << "numObjects.numFlexNodes" << test << std::endl;
    }

    /// Sets the FSI mesh for flexible elements.
    void SetFsiMesh(std::shared_ptr<fea::ChMesh> other_fsi_mesh) {
        fsi_mesh = other_fsi_mesh;
        fsiInterface->SetFsiMesh(other_fsi_mesh);
    }

    /// Sets the FSI system output mode.
    void SetParticleOutputMode(CHFSI_OUTPUT_MODE mode) { file_write_mode = mode; }

    /// Write FSI system particle output
    void WriteParticleFile(const std::string& outfilename) const;

    /// Function to save the SPH particle information into files,
    /// when called, this function creates three files to write fluid,
    /// boundary and BCE markers data into file
    void PrintParticleToFile(const std::string& out_dir) const;

    /// Add SPH particle's information into the FSI system
    void AddSphMarker(const ChVector<>& points,
                      const ChVector<>& properties,
                      const double h,
                      const double particle_type,
                      const ChVector<>& velocity = ChVector<>(),
                      const ChVector<>& tauXxYyZz = ChVector<>(),
                      const ChVector<>& tauXyXzYz = ChVector<>());

    /// Add reference array for SPH particles
    void AddRefArray(const int start, const int numPart, const int typeA, const int typeB);

    /// Add BCE particle for a box
    void AddBceBox(std::shared_ptr<SimParams> paramsH,
                   std::shared_ptr<ChBody> body,
                   const ChVector<>& relPos,
                   const ChQuaternion<>& relRot,
                   const ChVector<>& size,
                   int plane = 12);

    /// Add BCE particle for a cylinder
    void AddBceCylinder(std::shared_ptr<SimParams> paramsH,
                        std::shared_ptr<ChBody> body,
                        ChVector<> relPos,
                        ChQuaternion<> relRot,
                        double radius,
                        double height,
                        double kernel_h,
                        bool cartesian = true);

    /// Add BCE particle from a file
    void AddBceFile(std::shared_ptr<fsi::SimParams> paramsH,
                    std::shared_ptr<ChBody> body,
                    std::string dataPath,
                    ChVector<> collisionShapeRelativePos,
                    ChQuaternion<> collisionShapeRelativeRot,
                    double scale,
                    bool isSolid = true);                  
    
    /// Add BCE particle from mesh
    void AddBceFromMesh(std::shared_ptr<SimParams> paramsH,
                        std::shared_ptr<fea::ChMesh> my_mesh,
                        const std::vector<std::vector<int>>& NodeNeighborElement,
                        const std::vector<std::vector<int>>& _1D_elementsNodes,
                        const std::vector<std::vector<int>>& _2D_elementsNodes,
                        bool add1DElem,
                        bool add2DElem,
                        bool multiLayer,
                        bool removeMiddleLayer,
                        int SIDE,
                        int SIZE2D);

    /// Set FSI parameters from a JSON file
    void SetSimParameter(const std::string& inputJson,
                         std::shared_ptr<SimParams> paramsH,
                         const ChVector<>& box_size);

    /// Set Periodic boundary condition for fluid
    void SetBoundaries(const ChVector<>& cMin,
                       const ChVector<>& cMax,
                       std::shared_ptr<SimParams> paramsH);
                       
    /// Set prescribed initial pressure for gravity field
    void SetInitPressure(std::shared_ptr<SimParams> paramsH,
                         const double fzDim);

    /// Gets the FSI mesh for flexible elements.
    std::shared_ptr<fea::ChMesh> GetFsiMesh() { return fsi_mesh; }

    /// Return the SPH kernel length of kernel function.
    float GetKernelLength() const;

    /// Set subdomains so that we find neighbor particles faster
    void SetSubDomain(std::shared_ptr<SimParams> paramsH);

    /// Set output directory for FSI data
    void SetFsiOutputDir(std::shared_ptr<fsi::SimParams> paramsH,
                          std::string& demo_dir,
                          std::string out_dir,
                          std::string inputJson);

    /// Return the SPH particle position
    std::vector<ChVector<>> GetParticlePosOrProperties();

    /// Return the SPH particle velocity
    std::vector<ChVector<>> GetParticleVel();

  protected:
    std::shared_ptr<ChSystemFsi_impl> fsiSystem; 

  private:
    /// Integrate the chrono system based on an explicit Euler scheme.
    int DoStepChronoSystem(Real dT, double mTime);
    /// Set the type of the fluid dynamics
    void SetFluidIntegratorType(fluid_dynamics params_type);

    CHFSI_OUTPUT_MODE file_write_mode;  ///< FSI particle output type::CSV | ChPF | NONE, default is NONE

    std::vector<std::shared_ptr<ChBody>> fsiBodies;  ///< Vector of a pointers to fsi bodies. fsi bodies
                                                     /// are those that interact with fluid
    std::vector<std::shared_ptr<fea::ChElementCableANCF>> fsiCables;  ///< Vector of ChElementCableANCF pointers
    std::vector<std::shared_ptr<fea::ChElementShellANCF_3423>>
        fsiShells;                                              ///< Vector of ChElementShellANCF_3423 pointers
    std::vector<std::shared_ptr<fea::ChNodeFEAxyzD>> fsiNodes;  ///< Vector of ChNodeFEAxyzD nodes
    std::shared_ptr<fea::ChMesh> fsi_mesh;                      ///< ChMesh Pointer

    std::vector<std::vector<int>> ShellElementsNodes;  ///< Indices of nodes of each Element
    std::vector<std::vector<int>> CableElementsNodes;  ///< Indices of nodes of each Element
    std::shared_ptr<ChFluidDynamics> fluidDynamics;    ///< pointer to the fluid system
    CHFSI_TIME_INTEGRATOR fluidIntegrator;             ///< IISPH by default
    std::shared_ptr<ChFsiInterface> fsiInterface;      ///< pointer to the fsi interface system
    std::shared_ptr<ChBce> bceWorker;                  ///< pointer to the bce workers
    std::shared_ptr<SimParams> paramsH;                ///< pointer to the simulation parameters
    std::shared_ptr<NumberOfObjects> numObjectsH;      ///< number of objects, fluid, bce, and boundary markers
    chrono::ChSystem& mphysicalSystem;                 ///< Reference to the multi-body system

    double mTime;  ///< current real time of the simulation
};

/// @} fsi_physics

}  // end namespace fsi
}  // end namespace chrono

#endif
