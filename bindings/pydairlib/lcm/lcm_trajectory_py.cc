#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "lcm/lcm_trajectory.h"

namespace py = pybind11;

namespace dairlib {
namespace pydairlib {

PYBIND11_MODULE(lcm_trajectory, m) {
  m.doc() = "Binding functions for saving/loading trajectories";

  py::class_<lcmt_metadata>(m, "lcmt_metadata")
      .def_readwrite("datetime", &lcmt_metadata::datetime)
      .def_readwrite("name", &lcmt_metadata::name)
      .def_readwrite("description", &lcmt_metadata::description)
      .def_readwrite("git_commit_hash", &lcmt_metadata::git_commit_hash);

  py::class_<LcmTrajectory::Trajectory>(m, "Trajectory")
      .def(py::init<>())
      .def_readwrite("traj_name", &LcmTrajectory::Trajectory::traj_name)
      .def_readwrite("time_vector", &LcmTrajectory::Trajectory::time_vector)
      .def_readwrite("datapoints", &LcmTrajectory::Trajectory::datapoints)
      .def_readwrite("datatypes", &LcmTrajectory::Trajectory::datatypes);

  py::class_<lcmt_saved_traj>(m, "lcmt_saved_traj")
      .def(py::init<>())
      .def_readwrite("metadata", &lcmt_saved_traj::metadata)
      .def_readwrite("num_trajectories", &lcmt_saved_traj::num_trajectories)
      .def_readwrite("trajectories", &lcmt_saved_traj::trajectories)
      .def_readwrite("trajectory_names", &lcmt_saved_traj::trajectory_names);

  py::class_<LcmTrajectory>(m, "LcmTrajectory")
      .def(py::init<>())
      .def(py::init<const std::string&>())
      .def(py::init<const lcmt_saved_traj&>())
      .def("LoadFromFile", &LcmTrajectory::LoadFromFile,
           py::arg("trajectory_name"))
      .def("WriteToFile", &LcmTrajectory::WriteToFile,
           py::arg("trajectory_name"))
      .def("GetTrajectoryNames", &LcmTrajectory::GetTrajectoryNames)
      .def("GetMetadata", &LcmTrajectory::GetMetadata)
      .def("GenerateLcmObject", &LcmTrajectory::GenerateLcmObject)
      .def("GetTrajectory", &LcmTrajectory::GetTrajectory,
           py::arg("trajectory_name"));

}
}  // namespace pydairlib
}  // namespace dairlib
