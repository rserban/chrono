// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2022 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Radu Serban
// =============================================================================
//
// Output data writer for Polaris on SPH terrain system
//
// =============================================================================

#pragma once

#include <array>
#include <fstream>

#include <thrust/copy.h>
#include <thrust/gather.h>
#include <thrust/for_each.h>
#include <thrust/iterator/counting_iterator.h>
#include <thrust/functional.h>
#include <thrust/execution_policy.h>

#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono/utils/ChFilters.h"
#include "chrono_fsi/ChSystemFsi.h"

/// Base class for a data writer.
///
/// The data writer is reset with a frequency 'major_FPS'.  At each reset time, new sampling volumes are created under
/// each interacting object and the particle indices in each sampling volume are cached.  Following a reset, the states
/// of all tagged particles are saved at 'num_minor' subsequent frames, separated by 'minor_FPS'.
///
/// For example, assuming a step_size of 1 ms, a major and minor frequencies of 20 and 200, respectively, and
/// num_minor=5, output will be generated at the following simulation frames: 0,5,10,15,20, 50,55,60,65,70,
/// 100,105,110,115,120, etc. The sampling boxes and tracked particles are re-evaluated at frames: 0, 50, 100, etc.
///
/// Each output frame is uniquely identified with the index of the "major" frame M and the index of the "minor" frame m.
/// At each output frame, the following files are generated:
/// - soil_M_m_w0.csv, soil_M_m_w1.csv, soil_M_m_w2.csv, soil_M_m_w3.csv
///   which contain the state (position and optionally velocity) of a particle in the sampling volume below the
///   corresponding wheel.  Wheels are counted front-to-back and left-to-right; i.e., in the order FL, FR, RL, RR.
/// - mbs_M_m.csv
///   with information specific to a particular derived class.
///
/// Positions and orientations are relative to the global frame.
/// Rotations are provided as a quaternion (w,x,y,z).
/// Linear and angular velocities are relative to and expressed in the global frame.
/// Terrain forces and moments are assumed applied at the interacting object's center and are given in the global frame.
class DataWriter {
  public:
    enum class ParticleOutput { ALL, POSITIONS, NONE};

    virtual ~DataWriter();

    /// Enable/disable output data running-average filtering of velocities (default: true).
    void UseFilteredVelData(bool val, double window);

    /// Enable/disable output data running-average filtering of accelerations (default: true).
    void UseFilteredAccData(bool val, double window);

    /// Enable/disable terminal messages (default: true).
    void SetVerbose(bool verbose) { m_verbose = verbose; }

    /// Set output level for SPH particles (default: ALL).
    /// By default, output contains particle positions, velocities, and forces.
    void SetParticleOutput(ParticleOutput val) { m_particle_output = val; }

    /// Enable/disable individual output files for MBS (default: true).
    /// If false, only the global MBS output file is written.
    void SetMBSOutput(bool val) { m_mbs_output = val; }

    /// Set the location (relative to tire bottom point) and dimension (x and y) of the soil sampling domain.
    void SetSamplingVolume(const chrono::ChVector3d& offset, const chrono::ChVector2<>& size);

    /// Initialize the data writer, specifying the output directory and output frequency parameters.
    void Initialize(const std::string& dir, double major_FPS, double minor_FPS, int num_minor, double step_size);

    /// Run the data writer at the current simulation frame.
    /// This function must be called at each simulation frame; output occurs only at those frames that are consistent
    /// with the given output frequencies.
    void Process(int sim_frame, double time);

  protected:
    /// Construct an output data writer for the specified FSI system.
    DataWriter(chrono::fsi::ChSystemFsi& sysFSI, int num_sample_boxes);

    /// Specify the number of output data channels from the multibody system.
    virtual int GetNumChannelsMBS() const = 0;

    /// Specify the output velocity channels.
    virtual const std::vector<int> GetVelChannelsMBS() const = 0;

    /// Specify the output acceleration channels.
    virtual const std::vector<int> GetAccChannelsMBS() const = 0;

    /// Collect current values from all MBS output channels.
    virtual void CollectDataMBS() = 0;

    /// Write the MBS data at the current time.
    virtual void WriteDataMBS(const std::string& filename) = 0;

    /// Get the current position and orientation of the sampling box under the specified interacting object.
    virtual chrono::ChFrame<> GetSampleBoxFrame(int box_id) const = 0;

    int m_num_sample_boxes;
    chrono::ChVector3d m_box_size;
    chrono::ChVector3d m_box_offset;
    double m_box_x;
    double m_box_z;

    std::vector<double> m_mbs_outputs;

  private:
    void Reset();

    void Write();

    void WriteDataParticles(const thrust::device_vector<int>& indices_D, const std::string& filename);

    chrono::fsi::ChSystemFsi& m_sysFSI;
    std::array<thrust::device_vector<int>, 4> m_indices;

    std::string m_dir;
    int m_major_skip;
    int m_minor_skip;
    int m_out_frames;
    int m_major_frame;
    int m_minor_frame;
    int m_last_major;

    bool m_filter_vel;
    bool m_filter_acc;
    double m_filter_window_vel;
    double m_filter_window_acc;
    std::vector<std::shared_ptr<chrono::utils::ChRunningAverage>> m_filters_vel;
    std::vector<std::shared_ptr<chrono::utils::ChRunningAverage>> m_filters_acc;

    std::ofstream m_mbs_stream;

    ParticleOutput m_particle_output;
    bool m_mbs_output;
    bool m_verbose;
};

// --------------------------------------------------------------------------------------------------------------------

/// Data writer for a vehicle over SPH terrain.
///
/// It is assumed that the vehicle has exactly 4 wheels. Data is saved for a sampling box under each wheel.
///
/// At a "major" frame M and "minor" frame m, the mbs_M_m.csv output file contains 9 lines:
/// - line 1: vehicle position, orientation, linear velocity, angular velocity (3+4+3+3 values)
/// - line 2-5: wheel position, orientation, linear velocity, angular velocity (3+4+3+3 values per line)
/// - line 6-9: tire force and moment (3+3 values per line)
/// 
/// Positions and orientations are relative to the global frame.
/// Rotations are provided as a quaternion (w,x,y,z).
/// Linear and angular velocities are relative to and expressed in the global frame.
/// Tire forces and moments are assumed applied at the wheel/tire center and are given in the global frame.
class DataWriterVehicle : public DataWriter {
  public:
    DataWriterVehicle(chrono::fsi::ChSystemFsi& sysFSI, std::shared_ptr<chrono::vehicle::WheeledVehicle> vehicle);
    ~DataWriterVehicle() {}

  private:
    virtual int GetNumChannelsMBS() const override { return (7 + 6) + 4 * (7 + 6 + 3 + 3); }
    virtual const std::vector<int> GetVelChannelsMBS() const { return m_vel_channels; }
    virtual const std::vector<int> GetAccChannelsMBS() const { return m_acc_channels; }
    virtual void CollectDataMBS() override;
    virtual void WriteDataMBS(const std::string& filename) override;
    virtual chrono::ChFrame<> GetSampleBoxFrame(int box_id) const override;

    std::shared_ptr<chrono::vehicle::WheeledVehicle> m_vehicle;
    std::array<std::shared_ptr<chrono::vehicle::ChWheel>, 4> m_wheels;

    std::vector<int> m_vel_channels;
    std::vector<int> m_acc_channels;
};

// --------------------------------------------------------------------------------------------------------------------

/// Data writer for a single rigid object interacting with SPH terrain.
///
/// At a "major" frame M and "minor" frame m, the mbs_M_m.csv output file contains 2 lines:
/// - line 1: object position, orientation, linear velocity, angular velocity (3+4+3+3 values)
/// - line 2: terrain force and moment (3+3 values)
///
/// Positions and orientations are relative to the global frame.
/// Rotations are provided as a quaternion (w,x,y,z).
/// Linear and angular velocities are relative to and expressed in the global frame.
/// Terrain forces and moments are assumed applied at the object center and are given in the global frame.
class DataWriterObject : public DataWriter {
  public:
    DataWriterObject(chrono::fsi::ChSystemFsi& sysFSI,
                     std::shared_ptr<chrono::ChBody> body,
                     const chrono::ChVector3d& body_size);
    ~DataWriterObject() {}

  private:
    virtual int GetNumChannelsMBS() const override { return (7 + 6 + 3 + 3); }
    virtual const std::vector<int> GetVelChannelsMBS() const { return m_vel_channels; }
    virtual const std::vector<int> GetAccChannelsMBS() const { return m_acc_channels; }
    virtual void CollectDataMBS() override;
    virtual void WriteDataMBS(const std::string& filename) override;
    virtual chrono::ChFrame<> GetSampleBoxFrame(int box_id) const override;

    std::shared_ptr<chrono::ChBody> m_body;
    chrono::ChVector3d m_body_size;

    std::vector<int> m_vel_channels;
    std::vector<int> m_acc_channels;
};
