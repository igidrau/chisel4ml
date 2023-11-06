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

import qkeras
from tensorflow_model_optimization.python.core.sparsity.keras import prune

from chisel4ml.optimizations import qkeras_opt_list
from chisel4ml.preprocess.fft_layer import FFTLayer
from chisel4ml.preprocess.lmfe_layer import LMFELayer
from chisel4ml.qkeras_extensions import QDepthwiseConv2DPermuted

log = logging.getLogger(__name__)


def qkeras_model(model, skip_list=[]):
    "Applys optimizations to the model."
    new_model = prune.strip_pruning(model)
    new_model = qkeras.utils.clone_model(
        new_model,
        custom_objects={
            "FFTLayer": FFTLayer,
            "LMFELayer": LMFELayer,
            "QDepthwiseConv2DPermuted": QDepthwiseConv2DPermuted,
        },
    )
    new_model = qkeras.unfold_model(new_model)

    for opt in qkeras_opt_list:
        if opt.__class__.__name__ in skip_list:
            continue
        left = 0
        right = opt.num_layers
        while right < len(new_model.layers):
            assert right > left
            if opt.is_applicable(new_model.layers[left:right]):
                new_model = opt(new_model, new_model.layers[left:right])
            else:
                left = left + 1
                right = right + 1

    return new_model
