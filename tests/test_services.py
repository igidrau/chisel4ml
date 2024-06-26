import os

import numpy as np
from pytest_cases import get_current_cases
from pytest_cases import parametrize_with_cases

from chisel4ml import generate
from chisel4ml import optimize
from chisel4ml.utils import get_submodel
from tests.conftest import TEST_MODELS_LIST

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
MODEL_DIR = os.path.join(SCRIPT_DIR, "models")


@parametrize_with_cases("model_data_info", cases=TEST_MODELS_LIST, has_tag="trainable")
def test_trainable_simulation(request, model_data_info):
    model, data, training_info = model_data_info
    filename = get_current_cases(request)["model_data_info"][0]
    if request.config.getoption("--retrain"):
        print(f"Retraining model {filename}.")
        model.fit(
            data["X_train"],
            data["y_train"],
            batch_size=training_info["batch_size"],
            epochs=training_info["epochs"],
            callbacks=training_info["callbacks"],
            verbose=True,
        )
        opt_model = optimize.qkeras_model(model)
        opt_model.compile(
            optimizer=model.optimizer.from_config(model.optimizer.get_config()),
            loss=model.loss,
            metrics=["accuracy"],
        )
        opt_model.fit(
            data["X_train"],
            data["y_train"],
            batch_size=training_info["batch_size"],
            epochs=training_info["epochs"],
            callbacks=training_info["callbacks"],
            verbose=True,
        )
        if request.config.getoption("--save-retrained"):
            print(f"Saving model {filename} to {filename}.h5.")
            opt_model.save_weights(os.path.join(MODEL_DIR, f"{filename}.h5"))
    else:
        model_weights = os.path.join(MODEL_DIR, f"{filename}.h5")
        print(f"Loading weights from: {model_weights}.")
        opt_model = optimize.qkeras_model(model)
        opt_model.load_weights(model_weights)
    circuit = generate.circuit(
        opt_model,
        use_verilator=request.config.getoption("--use-verilator"),
        gen_waveform=request.config.getoption("--gen-waveform"),
        gen_timeout_sec=request.config.getoption("--generation-timeout"),
        num_layers=request.config.getoption("--num-layers"),
        debug=request.config.getoption("--debug-trans"),
    )
    assert circuit is not None
    for data in data["X_test"]:
        sw_res = opt_model.predict(np.expand_dims(data, axis=0))
        if isinstance(sw_res, tuple):
            sw_res = sw_res[0]
        hw_res = circuit(data)
        assert np.array_equal(sw_res.flatten(), hw_res.flatten())
    circuit.delete_from_server()


@parametrize_with_cases(
    "model_data_info", cases=TEST_MODELS_LIST, has_tag="trainable-gen"
)
def test_trainable_gen_simulation(request, model_data_info):
    model, data, training_info = model_data_info
    filename = get_current_cases(request)["model_data_info"][0]
    if request.config.getoption("--retrain"):
        print(f"Retraining model {filename}.")
        model.fit(
            x=data["train_set"]
            .batch(training_info["batch_size"], drop_remainder=True)
            .repeat(training_info["epochs"]),
            validation_data=data["val_set"]
            .batch(training_info["batch_size"], drop_remainder=True)
            .repeat(training_info["epochs"]),
            batch_size=training_info["batch_size"],
            steps_per_epoch=int(
                training_info["train_len"] / training_info["batch_size"]
            ),
            validation_steps=int(
                training_info["val_len"] / training_info["batch_size"]
            ),
            epochs=training_info["epochs"],
            callbacks=training_info["callbacks"],
            verbose=True,
        )
        opt_model = optimize.qkeras_model(model)
        opt_model.compile(
            optimizer=model.optimizer.from_config(model.optimizer.get_config()),
            loss=model.loss,
            metrics=["accuracy"],
        )
        opt_model.fit(
            x=data["train_set"]
            .batch(training_info["batch_size"], drop_remainder=True)
            .repeat(training_info["epochs"]),
            validation_data=data["val_set"]
            .batch(training_info["batch_size"], drop_remainder=True)
            .repeat(training_info["epochs"]),
            batch_size=training_info["batch_size"],
            steps_per_epoch=int(
                training_info["train_len"] / training_info["batch_size"]
            ),
            validation_steps=int(
                training_info["val_len"] / training_info["batch_size"]
            ),
            epochs=training_info["epochs"],
            callbacks=training_info["callbacks"],
            verbose=True,
        )
        if request.config.getoption("--save-retrained"):
            print(f"Saving model {filename} to {filename}.h5.")
            opt_model.save_weights(os.path.join(MODEL_DIR, f"{filename}.h5"))
    else:
        model_weights = os.path.join(MODEL_DIR, f"{filename}.h5")
        print(f"Loading weights from: {model_weights}.")
        opt_model = optimize.qkeras_model(model)
        opt_model.load_weights(model_weights)
    circuit = generate.circuit(
        opt_model,
        use_verilator=request.config.getoption("--use-verilator"),
        gen_waveform=request.config.getoption("--gen-waveform"),
        gen_timeout_sec=request.config.getoption("--generation-timeout"),
        num_layers=request.config.getoption("--num-layers"),
        debug=request.config.getoption("--debug-trans"),
    )
    assert circuit is not None
    for x, _ in data["test_set"]:
        sw_res = opt_model.predict(np.expand_dims(x, axis=0))[0]
        hw_res = circuit(x)
        assert np.array_equal(sw_res.flatten(), hw_res.flatten())
    circuit.delete_from_server()


@parametrize_with_cases("model_data", cases=TEST_MODELS_LIST, has_tag="non-trainable")
def test_simulation(request, model_data):
    (
        model,
        data,
    ) = model_data
    opt_model = optimize.qkeras_model(model)
    circuit = generate.circuit(
        opt_model,
        use_verilator=request.config.getoption("--use-verilator"),
        gen_waveform=request.config.getoption("--gen-waveform"),
        gen_timeout_sec=request.config.getoption("--generation-timeout"),
        num_layers=request.config.getoption("--num-layers"),
        debug=request.config.getoption("--debug-trans"),
    )
    if request.config.getoption("--num-layers") is not None:
        opt_model = get_submodel(opt_model, request.config.getoption("--num-layers"))
    assert circuit is not None
    for x in data:
        sw_res = opt_model.predict(np.expand_dims(x, axis=0))
        hw_res = circuit(x)
        assert np.array_equal(sw_res.flatten(), hw_res.flatten())
    circuit.delete_from_server()
