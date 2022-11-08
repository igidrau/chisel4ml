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

import qkeras
from tensorflow.keras.activations import linear

from chisel4ml.transforms.qkeras_util import _qkeras_base_transform_no_inp
from chisel4ml.transforms.qkeras_transforms import QKerasTransform
from chisel4ml.transforms import register_qkeras_transform
import chisel4ml.lbir.lbir_pb2 as lbir


@register_qkeras_transform
class QKerasLbirQDenseFuse(QKerasTransform):
    """ Takes the sequeunce: LbirLayer, QDense (with QActivation). And outputs a Sequence of two lbir layers."""
    num_layers = 2
    order = 4

    def _call_impl(self, layers):
        lbir_layer = _qkeras_base_transform_no_inp(layers[1])
        lbir_layer.input.CopyFrom(layers[0].output)
        return [layers[0], lbir_layer]
        
    def is_applicable(self, layers) -> bool:
        return (isinstance(layers[0], lbir.Layer) and
                isinstance(layers[1], qkeras.QDense))
