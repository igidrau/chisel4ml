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

from tensorflow.keras.layers import Layer as KerasLayer
from tensorflow.keras.activations import softmax
from tensorflow.keras.activations import linear
import qkeras
import numpy as np
import math

import chisel4ml.lbir.lbir_pb2 as lbir
from chisel4ml.transforms import register_qkeras_transform

from typing import Sequence
import inspect
import logging
log = logging.getLogger(__name__)


# We use a defaultdict here because quantizers can also be left unspecified in qkeras.
# TODO: Remove this by adding a pass that forces an quantization on those fields.
_qkeras_to_lbir_quantizer_dict = {}  # type: ignore
_qkeras_to_lbir_quantizer_dict[qkeras.quantized_bits] = lbir.Datatype.UNIFORM
_qkeras_to_lbir_quantizer_dict[qkeras.binary] = lbir.Datatype.BINARY
_qkeras_to_lbir_quantizer_dict[qkeras.quantized_relu] = lbir.Datatype.UNIFORM


def _qkeras_to_lbir_activation_transform(keras_act):
    """ Keras allows both objects and functions as activations. This wrapper function deals with this. """
    _qkeras_to_lbir_activation_dict = {
        qkeras.binary: lbir.Activation.BINARY_SIGN,
        qkeras.quantized_relu: lbir.Activation.RELU,
        softmax: lbir.Activation.NO_ACTIVATION,
        linear: lbir.Activation.NO_ACTIVATION,
    }
    if inspect.isfunction(keras_act):
        return _qkeras_to_lbir_activation_dict[keras_act]
    else:
        return _qkeras_to_lbir_activation_dict[keras_act.__class__]


def _qkeras_get_shape(keras_layer: KerasLayer, tensor: str) -> Sequence[int]:
    if (tensor == 'input' or
            tensor == 'output'):
        # We throw away the batch dimension
        return keras_layer.__getattribute__(tensor + '_shape')[1:]  # type: ignore
    elif tensor == 'kernel':
        return keras_layer.kernel.shape.as_list()  # type: ignore
    elif tensor == 'bias':
        return keras_layer.bias.shape.as_list()  # type: ignore
    else:
        raise ValueError(f'Function {__name__} invalid tensor string: {tensor}.')


def _qkeras_transform_tensor(keras_layer: KerasLayer, tensor: str) -> lbir.QTensor:
    """
        Populates a lbir QTensor. Unfortunately neither keras, nor qkeras hold all the necesssary info in a single
        field, so we must use some awkward methods to obtain them. The variable tensor should be one of the strings
        specified in the assertion (all lower case).
    """
    assert (tensor == 'kernel' or
            tensor == 'bias' or
            tensor == 'input')
    qkeras_quantizer = keras_layer.__getattribute__(tensor + '_quantizer_internal')
    qtensor = lbir.QTensor()
    if qkeras_quantizer is not None:
        qtensor.dtype.quantization = _qkeras_to_lbir_quantizer_dict[qkeras_quantizer.__class__]
    elif tensor == 'bias':
        qtensor.dtype.quantization = lbir.Datatype.UNIFORM
    else:
        raise ValueError(f'QKeras layer {keras_layer} {tensor} quantizer is not specified. Both input and kernel '
                         f'quantizers must be specified to allow a purely integer based implementation in hardware.')

    # We run this so that scale wont be a place holder tensor
    _ = keras_layer.kernel_quantizer_internal(keras_layer.kernel)

    if hasattr(qkeras_quantizer, 'bits'):
        qtensor.dtype.bitwidth = qkeras_quantizer.bits
    else:
        qtensor.dtype.bitwidth = 32

    if (isinstance(qkeras_quantizer, qkeras.quantizers.binary) or
       isinstance(qkeras_quantizer, qkeras.quantizers.ternary)):
        qtensor.dtype.scale[:] = [int(math.log2(qkeras.get_weight_scale(qkeras_quantizer)))] * \
                                    keras_layer.kernel.shape[1]
        qtensor.dtype.signed = True
    elif isinstance(qkeras_quantizer, qkeras.quantizers.quantized_bits):
        qtensor.dtype.scale[:] = np.log2(np.array(qkeras_quantizer.scale)).astype(int).flatten().tolist()
        qtensor.dtype.signed = qkeras_quantizer.keep_negative
    else:
        qtensor.dtype.scale[:] = [1]  # TODO for bias without quantizers, but is dangerous
    qtensor.shape[:] = _qkeras_get_shape(keras_layer, tensor)
    assert len(qtensor.shape) > 0
    if (tensor == 'kernel'):
        quant_internals = keras_layer.__getattribute__('kernel_quantizer_internal')
        qtensor.values[:] = (quant_internals(keras_layer.kernel) /
                             keras_layer.kernel_quantizer_internal.scale).numpy().flatten().tolist()
    elif tensor == 'bias':
        # Bias can be folded into the activation in some cases (BNN), so we allow using float biases as well.
        # We negate bias values because we redefine them as a threshold value. I.e. w*x +b > 0 == w*x > thresh (=-b)
        thresh_arr = -keras_layer.bias.numpy()
        qtensor.values[:] = (thresh_arr).flatten().tolist()
    return qtensor


def _qkeras_base_transform(keras_layer: KerasLayer) -> lbir.Layer:
    """ The base tranform transform transforms the parts of the layer that are common to all layers. """
    lbir_layer = lbir.Layer()
    lbir_layer.use_bias = keras_layer.use_bias
    lbir_layer.biases.CopyFrom(_qkeras_transform_tensor(keras_layer, 'bias'))
    lbir_layer.weights.CopyFrom(_qkeras_transform_tensor(keras_layer, 'kernel'))
    lbir_layer.input.CopyFrom(_qkeras_transform_tensor(keras_layer, 'input'))
    lbir_layer.activation.fn = _qkeras_to_lbir_activation_transform(keras_layer.activation)
    lbir_layer.activation.bitwidth = 1  # TODO currently only supporting BINARY_SIGN act
    output = lbir.QTensor()
    dtype = lbir.Datatype()
    if isinstance(keras_layer.activation, qkeras.quantizers.binary):
        dtype.quantization = lbir.Datatype.BINARY
        dtype.bitwidth = 1
        dtype.signed = True
    elif isinstance(keras_layer.activation, qkeras.quantizers.quantized_relu):
        dtype.quantization = lbir.Datatype.UNIFORM
        dtype.bitwidth = keras_layer.activation.bits
        dtype.signed = False
    elif (keras_layer.activation.__name__ == 'softmax' or
          keras_layer.activation.__name__ == 'linear'):
        dtype.quantization = lbir.Datatype.UNIFORM
        dtype.bitwidth = 8  # TODO: Allow overwrite
        dtype.signed = True
    else:
        raise NotImplementedError
    dtype.scale[:] = [1]
    dtype.offset[:] = [0]
    output.dtype.CopyFrom(dtype)
    output.shape[:] = keras_layer.output_shape[1:]
    lbir_layer.output.CopyFrom(output)

    return lbir_layer
