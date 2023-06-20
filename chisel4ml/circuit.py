# Copyright 2022 Computer Systems Department, Jozef Stefan Insitute
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#  https://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import logging
import os
import shutil
from pathlib import Path

import numpy as np

import chisel4ml.lbir.lbir_pb2 as lbir
import chisel4ml.lbir.services_pb2 as services
from chisel4ml import transforms
from chisel4ml.chisel4ml_server import start_server_once

log = logging.getLogger(__name__)


class Circuit:
    """Container class for the generated circuits. Objects of type Circuit hold a
    reference to the circuit implemnentation in the chisel4ml server memory. It also
    provides a python interface to that server via gRPC (via __call__ or predict).
    """

    def __init__(self, circuitId: int, input_quantizer, input_qtensor: lbir.QTensor):
        assert circuitId >= 0, (
            "Invalid circuitId provided. This parameter should be positive, but is"
            f" {circuitId}."
        )
        self.circuitId = circuitId
        self.input_quantizer = input_quantizer
        self.input_qtensor = input_qtensor
        self._server = start_server_once()

    def __call__(self, np_arr, sim_timeout_sec=100):
        "Simulate the circuit, timeout in seconds."
        qtensors = transforms.numpy_transforms.numpy_to_qtensor(
            np_arr, self.input_quantizer, self.input_qtensor
        )
        run_sim_params = services.RunSimulationParams(
            circuitId=self.circuitId, inputs=qtensors
        )
        run_sim_return = self._server.send_grpc_msg(
            run_sim_params, timeout=sim_timeout_sec
        )
        results = []
        for res in run_sim_return.values:
            results.append(res.values)
        return np.array(results)

    def predict(self, np_arr):
        return self.__call__(np_arr)

    def package(self, directory=None, name="ProcessingPipeline"):
        if directory is None:
            raise ValueError("Directory parameter missing.")
        temp_dir = self._server.temp_dir
        temp_circuit_dir = os.path.join(temp_dir, f"circuit{self.circuitId}")
        try:
            temp_circuit_file = Path(temp_circuit_dir).glob("*.sv").__next__()
        except StopIteration:
            raise Exception("Can only package if Verilator selected as backend.")
        dest_file = directory
        os.makedirs(Path(directory).absolute(), exist_ok=True)
        if os.path.isdir(directory):
            dest_file = os.path.join(directory, f"{name}.sv")
        shutil.copyfile(temp_circuit_file, dest_file)
