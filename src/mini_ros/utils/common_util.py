import bisect
import datetime
from typing import Union

import numpy


class CommonUtil:

    @classmethod
    def timestamp_to_nanosec(
        cls, ts: Union[datetime.datetime, numpy.ndarray, float]
    ) -> int:
        if isinstance(ts, datetime.datetime):
            return int(ts.timestamp() * 1e9)
        elif isinstance(ts, numpy.ndarray):
            if isinstance(ts[0], datetime.datetime):
                return int(ts[0].timestamp() * 1e9)
            elif isinstance(ts[0], float):
                return int(ts[0] * 1e9)
            else:
                raise ValueError(f"Invalid timestamp type: {type(ts[0])}")
        elif isinstance(ts, float):
            return int(ts * 1e9)
        else:
            raise ValueError(f"Invalid timestamp type: {type(ts)}")

    @classmethod
    def find_largest_smaller_timestamp(cls, target, timestamps):
        index = bisect.bisect_left(timestamps, target)

        if index > 0:
            return timestamps[index - 1]
        else:
            return None
