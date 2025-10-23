import asyncio
from doctest import FAIL_FAST
import json
import multiprocessing
from typing import Any, Callable, Coroutine, Type, Dict
from dataclasses import dataclass, field

import aiomultiprocess
import fastapi
import uvicorn
from fastapi import FastAPI
from fastapi.responses import HTMLResponse
from fastapi.websockets import WebSocketDisconnect
from mini_ros.common.state import CommanderState
from starlette.middleware.cors import CORSMiddleware
from starlette.requests import Request
from starlette.websockets import WebSocket, WebSocketState
from mini_ros.utils.web_util import WebUtil
from mini_ros.utils.async_util import AsyncUtil
from mini_ros.utils.lang_util import LangUtil
from mini_ros.utils.time_util import TimeUtil
from mini_ros.utils.rate_limiter import RateLimiterAsync
from mini_ros.utils.config import LazyConfig
from fastapi.staticfiles import StaticFiles
from loguru import logger


@dataclass
class ProxyFunc:
    func: Callable
    args: Dict[str, Type] = field(default_factory=dict)


def bind_apis(app: FastAPI):
    path = LangUtil.get_abs_path("frontend/dist")

    if not path.exists():
        logger.error("Webui dist directory not found, did you run `pnpm build`?")
        return

    logger.info("Launching webui at", path)

    app.mount("/", StaticFiles(directory=path, html=True), name="rdc-web")


class CommanderWebWrapper:
    """
    Registers a web driver for the commander node.
    Reflect commander's method to the web API.
    """

    def __init__(self, commander):
        self.app = FastAPI()
        self.commander = commander
        self.task_bind: asyncio.Task = None

    async def on_ws_error(self, ws: WebSocket, e: Exception):

        is_connected = ws.client_state == WebSocketState.CONNECTED

        if is_connected:
            await ws.send_json({"error": str(e)})
            await ws.close()

        if isinstance(e, fastapi.WebSocketDisconnect):
            logger.verbose("WebSocket disconnected")
        else:
            logger.error(e)

    def spawn_admin_api(self, method_name: str, proxy_func: ProxyFunc):
        @self.app.post(f"/admin/{method_name}")
        @self.app.get(f"/admin/{method_name}")
        async def api_call(request: Request):
            try:

                required_params = list(proxy_func.args.keys())

                params = await WebUtil.get_params(request)
                WebUtil.ensure_valid_request(params, *required_params)
                result = await proxy_func.func(**params)

                return WebUtil.make_success(content=result)
            except Exception as e:
                logger.error(f"Error calling {method_name}", e)
                return WebUtil.make_error(e)

        return api_call

    def spawn_commander_api(
        self,
        method_name: str,
        proxy_func: ProxyFunc,
    ):
        @self.app.post(f"/{method_name}")
        @self.app.get(f"/{method_name}")
        async def api_call(request: Request):
            try:

                required_params = list(proxy_func.args.keys())

                WebUtil.ensure_valid_request(request, *required_params)

                params = await WebUtil.get_params(request)

                result = await proxy_func.func(**params)
                return WebUtil.make_success(content=result)
            except Exception as e:
                logger.error(f"Error calling {method_name}", e)

                # TODO: Return a proper error message
                return WebUtil.make_error(e)

        return api_call

    def class_name(self) -> str:
        return "WebDriver"

    def initialize(self) -> FastAPI:
        """
        Register all the APIs to the app instance
        """

        fast_api_app = self.app

        for name, proxy_func in (
            self.commander.get_api_mapping().items()
        ):
            self.spawn_commander_api(name, proxy_func)

        @fast_api_app.get(f"/commander_state")
        async def get_commander_state(request: Request):

            states = await self.commander.get_states()

            return WebUtil.make_success(
                content=WebUtil.get_printable_json(
                    {
                        "commander": states["commander_state"] != CommanderState.DEAD,
                        "robot": states["is_connection_bad"],
                    }
                )
            )

        @fast_api_app.get(f"/get_color")
        async def get_color(request: Request):
            """
            This is to make Material UI happy.
            """

            return WebUtil.make_success(content={"color": "#FFFFFF"})

        @fast_api_app.get(f"/version")
        async def get_version(request: Request):
            return WebUtil.make_success(content={"version": LazyConfig.get_version()})

        @fast_api_app.websocket(f"/ws/health")
        async def ws_health(websocket: WebSocket):
            await websocket.accept()

            rate_limiter = RateLimiterAsync(60)

            prev = None

            try:
                while True:
                    await rate_limiter.wait_for_tick("WebDriver_ws_health")
                    parsed = (
                        await self.commander.get_health_state()
                    )

                    # Only send on diff
                    if prev is None or (
                        WebUtil.get_printable_json(prev)
                        != WebUtil.get_printable_json(parsed)
                    ):
                        prev = parsed
                        if websocket.client_state == WebSocketState.CONNECTED:
                            await websocket.send_json(
                                WebUtil.get_printable_json(parsed)
                            )
                    await rate_limiter.unset_busy("WebDriver_ws_health")
            except Exception as e:
                await self.on_ws_error(websocket, e)

        @fast_api_app.websocket(f"/states_simple")
        async def ws_get_commander_state_simple(websocket: WebSocket):

            await websocket.accept()

            rate_limiter = RateLimiterAsync(50)

            logger.info(
                self.class_name(),
                f"Client connected to {websocket.base_url} websocket from {websocket.client.host}",
            )

            prev = None

            try:
                while True:
                    await rate_limiter.wait_for_tick("WebDriver_ws_get_commander_state")

                    payload = WebUtil.get_printable_json(
                        await self.commander.get_states()
                    )

                    # Too noisy
                    unwanted_key = [
                        "robot_data",
                        "gello_state",
                        "last_server_comm_time",
                    ]

                    for key in unwanted_key:
                        payload.pop(key, None)

                    if prev is None or (prev != payload):
                        prev = payload
                        if websocket.client_state == WebSocketState.CONNECTED:
                            await websocket.send_json(payload)

                    await rate_limiter.unset_busy("WebDriver_ws_get_commander_state")
            except Exception as e:
                await self.on_ws_error(websocket, e)

        @fast_api_app.websocket(f"/states")
        async def ws_get_commander_state(websocket: WebSocket):

            await websocket.accept()

            rate_limiter = RateLimiterAsync(50)

            logger.info(
                self.class_name(),
                f"Client connected to {websocket.base_url} websocket from {websocket.client.host}",
            )

            try:
                while True:
                    await rate_limiter.wait_for_tick("WebDriver_ws_get_commander_state_simple")

                    payload = WebUtil.get_printable_json(
                        await self.commander.get_states()
                    )
                    if websocket.client_state == WebSocketState.CONNECTED:
                        await websocket.send_json(payload)

                    await rate_limiter.unset_busy("WebDriver_ws_get_commander_state_simple")
            except Exception as e:
                await self.on_ws_error(websocket, e)

        @fast_api_app.get("/gello/states")
        @fast_api_app.post("/gello/states")
        async def get_gello_states(request: Request):
            return WebUtil.make_success(
                content=WebUtil.get_printable_json(
                    (await self.commander.get_states())[
                        "gello_state"
                    ]
                )
            )

        @fast_api_app.get("/counter")
        async def get_counter(request: Request):
            return WebUtil.make_success(
                content={
                    "counter": await self.commander.get_counter()
                }
            )

        @fast_api_app.delete("/counter")
        async def reset_counter(request: Request):
            await self.commander.reset_counter()
            return WebUtil.make_success(content={"counter": 0})

        return fast_api_app

    def _bind(self):
        try:

            web_port = LangUtil.maybe(LazyConfig.get_param("commander/webui/port"), 8000)
            web_host = LangUtil.maybe(
                LazyConfig.get_param("commander/webui/host"), "0.0.0.0"
            )

            is_serve_frontend = LazyConfig.get_param("misc/serve_frontend")

            if is_serve_frontend == None:
                is_serve_frontend = True

            if is_serve_frontend:
                web_port = LangUtil.maybe(
                    LazyConfig.get_param("commander/webui/port"), 5006
                )
                web_host = LangUtil.maybe(
                    LazyConfig.get_param("commander/webui/host"), "0.0.0.0"
                )

            config = uvicorn.Config(self.app, host=web_host, port=web_port, access_log=False)

            server = uvicorn.Server(config)

            self.task_bind = AsyncUtil.detach_coroutine(server.serve())
            logger.info(f"Web driver bound to {web_host}:{web_port}")

        except asyncio.CancelledError:
            pass
        except Exception as e:
            logger.error(f"Error binding web driver: {e}")
            raise e

    async def stop(self):
        try:
            if self.task_bind is not None:
                logger.info("Stopping web driver")
                self.task_bind.cancel()

            await TimeUtil.wait_until_condition(
                condition=lambda: not self.is_alive(), timeout_ms=5000
            )

        except Exception as e:
            logger.error(f"Error stopping web driver: {e}")
            # Logger.error(f"Error stopping web driver: {e}")
            # raise e

    def is_alive(self) -> bool:
        return self.task_bind is not None and not self.task_bind.done()

    async def start(self):

        # CORS setup
        # TODO: Make the CORS policy safer and configurable

        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],  #
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )

        @self.app.middleware("http")
        async def log_requests(request: Request, call_next):
            client_ip = request.client.host
            route = request.url.path

            if route != "/get_color":
                logger.info(
                    self.class_name(), f"HTTP Request from {client_ip} to {route}"
                )

            response = await call_next(request)
            return response

        # TODO: Make this configurable
        self._bind()
