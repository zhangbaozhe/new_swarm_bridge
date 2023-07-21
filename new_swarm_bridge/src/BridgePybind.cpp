#include "BridgeServerPythonInterface.h"
#include "BridgeClientPythonInterface.h"

namespace py = pybind11;

PYBIND11_MODULE(Bridge, m) {
  py::class_<ServerDataPackagePythonInterface>(m, "ServerDataPackage")
      .def(py::init())
      .def("get_size", &ServerDataPackagePythonInterface::getSize)
      .def("get_data", &ServerDataPackagePythonInterface::getData)
      .def("release", &ServerDataPackagePythonInterface::release)
      ;

  py::class_<BridgeServerPythonInterface>(m, "BridgeServer")
      .def(py::init())
      .def("init", &BridgeServerPythonInterface::init)
      .def("run", &BridgeServerPythonInterface::run)
      .def("stop", &BridgeServerPythonInterface::stop)
      .def("release", &BridgeServerPythonInterface::release)
      .def("get_latest_data_packages", &BridgeServerPythonInterface::getLatestDataPackages)
      ;

  py::class_<ClientDataPackagePythonInterface>(m, "ClientDataPackage")
      .def(py::init())
      .def("set_size", &ClientDataPackagePythonInterface::setSize)
      .def("set_data", &ClientDataPackagePythonInterface::setData)
      ;
  
  py::class_<BridgeClientPythonInterface>(m, "BridgeClient")
      .def(py::init())
      .def("init", &BridgeClientPythonInterface::init)
      .def("run", &BridgeClientPythonInterface::run)
      .def("stop", &BridgeClientPythonInterface::stop)
      .def("set_data_packages", &BridgeClientPythonInterface::setDataPackages)
      ;
}