from typing import Any, Dict, Optional

import numpy
from starlette.requests import Request
from starlette.responses import JSONResponse

from loguru import logger
from mini_ros.utils.time_util import TimeUtil


class WebUtil:

    @classmethod
    def get_proper_msg(cls, msg: Any):
        """
        Args:
            msg: a maybe-an-exception message

        Returns:
            A string representation of the message
        """
        return (
            msg
            if not isinstance(msg, Exception)
            else (
                "[EXCEPTION]{}: {}".format(msg.__class__, msg.args)
                if len(msg.args) == 0
                else msg.args[0]
            )
        )

    @classmethod
    def make_response(cls, content: Any, is_good: bool = True, status_code: int = 200):
        return JSONResponse(
            {
                "status": 0 if is_good else 1,
                "timestamp": TimeUtil.now().isoformat(),
                "content": cls.get_printable_json(content),
            },
            status_code=status_code,
        )

    @classmethod
    def make_success(cls, content: Any, status_code: int = 200):
        return WebUtil.make_response(
            content=content, is_good=True, status_code=status_code
        )

    @classmethod
    def make_error(cls, error: Any, status_code: int = 403):
        return WebUtil.make_response(
            cls.get_proper_msg(error), is_good=False, status_code=status_code
        )

    @classmethod
    def get_printable_json(cls, json: Optional[Dict[str, Any]]):

        if json is None:
            return None
        elif isinstance(json, (int, float, str, bool)):
            return str(json)

        # if the actual type is anything that does not have .items(), try

        # Recursively resolve the json object, for the ones that are not serializable, use str() on them
        for key, value in json.items():
            if isinstance(value, dict):
                json[key] = cls.get_printable_json(value)
            elif isinstance(value, numpy.ndarray):
                json[key] = value.tolist()
            elif value is None:
                json[key] = None
            elif not isinstance(value, (int, float, str, bool, list, tuple, dict)):
                json[key] = str(value)
        return json

    @classmethod
    async def get_params(cls, request: Request):

        has_body = (
            request.method
            in [
                "POST",
                "PUT",
                "DELETE",
                "PATCH",
            ]
            and request.headers.get("content-type") == "application/json"
        )

        if has_body:
            try:
                return await request.json()
            except Exception as e:
                logger.warning(f"Error parsing JSON body: {e}")
                return {}
        else:
            return dict(request.query_params)

    @classmethod
    def ensure_valid_request(cls, params: Dict[str, Any], *required_args):
        missing_args = [
            arg
            for arg in required_args
            if arg not in params.keys() or params[arg] is None
        ]

        if len(missing_args) > 0:
            raise Exception(f"required args missing: {missing_args}")
