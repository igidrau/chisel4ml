from chisel4ml.optimizations.qkeras_bn_qdense_binary_fuse import (
    QKerasBNQDenseBinaryFuse,
)
from chisel4ml import optimize
from math import isclose

import tensorflow as tf
from tensorflow.keras.datasets import mnist
import numpy as np
import qkeras


def test_bn_qdense_binary_fuse_opt(bnn_qdense_bn_sign_act):
    """Tests the fusing of binary qdense layer with a batch normalization layer."""
    org_model = bnn_qdense_bn_sign_act
    new_model = qkeras.utils.clone_model(bnn_qdense_bn_sign_act)
    new_model.compile(optimizer="adam", loss="squared_hinge")
    l0 = new_model.layers[0]
    l1 = new_model.layers[1]
    l2 = new_model.layers[2]
    opt = QKerasBNQDenseBinaryFuse()
    assert opt.is_applicable([l0, l1, l2])
    new_model = opt(new_model, [l0, l1, l2])
    assert not isinstance(new_model.layers[1], tf.keras.layers.BatchNormalization)
    for i0 in [+1.0, -1.0]:
        for i1 in [+1.0, -1.0]:
            org_res = org_model.predict(np.array([i0, i1]).reshape(1, 2))
            opt_res = new_model.predict(np.array([i0, i1]).reshape(1, 2))
            assert tf.reduce_all(tf.math.equal(org_res, opt_res)), (
                "There seems to be a problem with the BatchNorm fuse operation for"
                f" binary kerenels. The original model predicted {org_res} but the"
                f" optimized version predicted {opt_res} for inputs [{i0},{i1}].The"
                f" original parameters are qdense layer:{org_model.layers[0].weights},"
                f" {org_model.layers[0].bias} and for batchnorm:"
                f" {org_model.layers[1].get_weights()} and the optimized weights are:"
                f" {opt_res.layers[0].weights}, {opt_res.layers[0].bias}."
            )


def test_bnn_mnist_model_opt(bnn_mnist_model):
    """The optimization of this model should yield a model that produces the same
    results.
    """
    opt_model = optimize.qkeras_model(bnn_mnist_model)
    (_, _), (x_test, y_test) = mnist.load_data()
    image_vector_size = 28 * 28
    x_test = x_test.reshape(x_test.shape[0], image_vector_size)
    x_test = x_test.astype("float32")

    for i in range(0, 10):
        org_res = bnn_mnist_model.predict(x_test[i].reshape(1, 784))
        opt_res = opt_model.predict(x_test[i].reshape(1, 784))
        assert tf.reduce_all(tf.math.equal(org_res, opt_res)), (
            f"The original model predicted the result: {org_res}, where as the"
            f" optimized model predicted: {opt_res}.The results differed on mnist test"
            f" image index: {i}."
        )


def test_sint_mnist_qdense_relu_opt(sint_mnist_qdense_relu):
    """Tests if the model performs (approximatly) as well after optimization, as before
    optimization.
    """
    (_, _), (x_test, y_test) = mnist.load_data()
    image_vector_size = 28 * 28
    x_test = x_test.reshape(x_test.shape[0], image_vector_size)
    x_test = x_test.astype("float32")
    y_test = tf.one_hot(y_test, 10)
    (_, acc) = sint_mnist_qdense_relu.evaluate(x_test, y_test, verbose=0)
    opt_model = optimize.qkeras_model(sint_mnist_qdense_relu)
    opt_model.compile(
        optimizer="adam", loss="categorical_crossentropy", metrics=["accuracy"]
    )
    (_, acc_opt) = opt_model.evaluate(x_test, y_test, verbose=0)
    assert isclose(acc, acc_opt, abs_tol=0.03), (
        "The prediction of the optimized model should be with in 3 percent of the"
        " original model. Numerical instability can account for such small"
        " differences, bigger differences are likely some other failure. The original"
        f" sint_mnist_qdense_relu model had accuracy of {acc} and the optimized"
        f" {acc_opt}."
    )


def test_sint_mnist_qdense_relu_pruned_opt(sint_mnist_qdense_relu_pruned):
    """Tests if the pruned model performs (approximatly) as well after optimization,
    as before optimization.
    """
    (_, _), (x_test, y_test) = mnist.load_data()
    image_vector_size = 28 * 28
    x_test = x_test.reshape(x_test.shape[0], image_vector_size)
    x_test = x_test.astype("float32")
    y_test = tf.one_hot(y_test, 10)
    (_, acc) = sint_mnist_qdense_relu_pruned.evaluate(x_test, y_test, verbose=0)
    opt_model = optimize.qkeras_model(sint_mnist_qdense_relu_pruned)
    opt_model.compile(
        optimizer="adam", loss="categorical_crossentropy", metrics=["accuracy"]
    )
    (_, acc_opt) = opt_model.evaluate(x_test, y_test, verbose=0)
    assert isclose(acc, acc_opt, abs_tol=0.05), (
        "The prediction of the optimized model should be with in 5 percent of the"
        " original model. Numerical instability can account for such small"
        " differences, bigger differences are likely some other failure. The original"
        f" sint_mnist_qdense_relu_pruned model had accuracy of {acc} and the optimized"
        f" {acc_opt}."
    )
