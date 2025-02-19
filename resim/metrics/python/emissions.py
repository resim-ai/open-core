import json
from pathlib import Path
from typing import Any, Optional, Union
import numpy.typing as npt
from resim.metrics.python.metrics_utils import Timestamp

def is_jsonable(x):
    try:
        json.dumps(x)
        return True
    except (TypeError, OverflowError):
        return False

def emit(
    topic_name: str,
    data: dict[str, Any], 
    *,
    timestamp: Optional[Union[int, Timestamp]] = None, 
    timestamps: Optional[Union[list[int], list[Timestamp], npt.NDArray]] = None, 
    outfile: Path = Path("/tmp/resim/outputs/emissions.ndjson")
):
    try:
        # only allow one of timestamp or timestamps to be set
        if timestamp is not None and timestamps is not None:
            raise ValueError("Only one of timestamp or timestamps can be set")
        
        if timestamp is None and timestamps is not None:
            # assert all data values are mapped to lists of the same length
            assert all([isinstance(v, list) for v in data.values()]), "All data values must be lists"
            assert all([len(timestamps) == len(series) for series in data.values()]), "All series must be the same length as the timestamps"
            for i, ts in enumerate(timestamps):
                scalar_data = {k: v[i] for k, v in data.items()}
                emit(topic_name, scalar_data, timestamp=ts)
            return

        # build the single point emission dictionary
        emission = {
            "$metadata": {
                "topic": topic_name,
            },
            "$data": data,
        }
        if timestamp is not None:
            if isinstance(timestamp, Timestamp):
                emission["$metadata"]["timestamp"] = timestamp.to_int()
            else:
                emission["$metadata"]["timestamp"] = timestamp
    
        # write the emission to the output file
        with open(outfile, "a") as f:
            f.write(json.dumps(emission) + "\n")
    except Exception as e:
        raise RuntimeError(f"Error emitting topic {topic_name}") from e
