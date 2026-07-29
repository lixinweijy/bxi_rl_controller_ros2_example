from pathlib import Path
import os

import numpy as np
import onnxruntime as ort


TRACE_TAG = "BXI_MOTION_RKNN_V1"


class ModelTensorInfo:
    def __init__(self, name, shape):
        self.name = name
        self.shape = shape


class RknnPolicySession:
    def __init__(self, onnx_path, rknn_path, output_names=None):
        try:
            from rknnlite.api import RKNNLite
        except Exception as exc:
            raise RuntimeError("rknn-toolkit-lite2 is unavailable") from exc

        metadata = ort.InferenceSession(
            str(onnx_path),
            providers=["CPUExecutionProvider"],
        )
        self._inputs = [
            ModelTensorInfo(item.name, item.shape)
            for item in metadata.get_inputs()
        ]
        metadata_outputs = [
            ModelTensorInfo(item.name, item.shape)
            for item in metadata.get_outputs()
        ]
        if output_names:
            output_name_set = set(output_names)
            self._outputs = [
                item for item in metadata_outputs if item.name in output_name_set
            ]
        else:
            self._outputs = metadata_outputs
        self._input_names = [item.name for item in self._inputs]
        self._output_names = [item.name for item in self._outputs]

        rknn = RKNNLite(verbose=False)
        ret = rknn.load_rknn(str(rknn_path))
        if ret != 0:
            rknn.release()
            raise RuntimeError(f"load_rknn failed: ret={ret}")
        ret = rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_AUTO)
        if ret != 0:
            rknn.release()
            raise RuntimeError(f"init_runtime failed: ret={ret}")
        self._rknn = rknn

    def get_inputs(self):
        return self._inputs

    def get_outputs(self):
        return self._outputs

    def run(self, output_names, feed):
        inputs = [
            np.asarray(feed[name], dtype=np.float32)
            for name in self._input_names
        ]
        outputs = self._rknn.inference(inputs=inputs)
        if outputs is None:
            raise RuntimeError("RKNN inference returned no outputs")
        if output_names is None:
            return outputs

        selected = []
        for name in output_names:
            selected.append(outputs[self._output_names.index(name)])
        return selected

    def release(self):
        self._rknn.release()

    def __del__(self):
        try:
            self.release()
        except Exception:
            pass


def create_motion_session(
    onnx_path,
    providers,
    sess_options,
    *,
    rknn_outputs=None,
    label="motion",
):
    onnx_path = Path(onnx_path)
    rknn_path = onnx_path.with_suffix(".rknn")
    if os.environ.get("BXI_MOTION_RKNN", "1").lower() not in ("0", "false", "off"):
        if rknn_path.exists():
            try:
                session = RknnPolicySession(onnx_path, rknn_path, rknn_outputs)
                print(
                    f"{TRACE_TAG} enabled label={label} "
                    f"onnx={onnx_path} rknn={rknn_path}"
                )
                return session
            except Exception as exc:
                print(
                    f"{TRACE_TAG} fallback label={label} "
                    f"rknn={rknn_path} error={exc}"
                )
        else:
            print(f"{TRACE_TAG} missing label={label} rknn={rknn_path}")
    else:
        print(f"{TRACE_TAG} disabled label={label}")

    session = ort.InferenceSession(
        str(onnx_path),
        providers=providers,
        sess_options=sess_options,
    )
    print(f"{TRACE_TAG} onnxruntime label={label} onnx={onnx_path}")
    return session
