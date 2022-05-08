import qkeras
import tensorflow as tf

from tensorflow.keras.layers import Layer as KerasLayer
from tensorflow.keras.layers import BatchNormalization
from tensorflow.math import sqrt
from tensorflow.math import multiply as mul
from tensorflow.math import truediv as div

from typing import Sequence

from chisel4ml.optimizations.qkeras_optimization import QKerasOptimization
from chisel4ml.optimizations import register_qkeras_optimization


@register_qkeras_optimization
class QKerasBNQDenseBinaryFuse(QKerasOptimization):
    """
        Fuses the BatchNorm and QDense layer with a binary quantizer. For more information read the paper
        on Binarized Neural networks by Umuroglu et al.: https://arxiv.org/pdf/1612.07119.pdf.
    """
    num_layers = 3
    priority = 2

    def _call_impl(self, layers: Sequence[KerasLayer]) -> Sequence[KerasLayer]:
        fan_in = layers[0].kernel.shape.as_list()[0]
        mm = layers[1].moving_mean
        mv = layers[1].moving_variance
        beta = layers[1].beta
        gamma = layers[1].gamma
        b = layers[0].bias
        thresh = (mm - b) - div(mul(sqrt(mv), beta), gamma)
        layers[0].bias = tf.math.floor((fan_in + thresh) / 2)
        layers[1].c4ml_remove_layer = True
        return layers

    def is_applicable(self, layers: Sequence[KerasLayer]) -> bool:
        return (type(layers[0]) is qkeras.QDense and
                type(layers[1]) is BatchNormalization and
                type(layers[2]) is qkeras.QActivation and
                type(layers[2].activation) is qkeras.quantizers.binary)
