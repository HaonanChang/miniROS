import uvicorn
import uvloop
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import RedirectResponse
from fastapi.staticfiles import StaticFiles

from mini_ros.utils.config import LazyConfig
from mini_ros.utils.lang_util import LangUtil
from mini_ros.utils.cli_util import CliUtil
from loguru import logger


def bind_apis(app: FastAPI):
    path = LangUtil.get_abs_path("frontend/dist")

    if not path.exists():
        raise Exception("Webui dist directory not found, did you run `pnpm build`?")

    logger.verbose("Launching webui at", path)

    app.mount("/", StaticFiles(directory=path, html=True), name="rdc-web")


def main():

    app = FastAPI()

    bind_apis(app)

    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],  #
        allow_credentials=False,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    host = LangUtil.maybe(LazyConfig.get_param("commander/webui/host"), "0.0.0.0")
    port = LangUtil.maybe(LazyConfig.get_param("commander/webui/port"), 5006)

    uvicorn.run(
        app,
        host=host,
        port=port,
    )


if __name__ == "__main__":

    args = CliUtil.get_cli_args()

    if args["config"] is not None:
        LazyConfig.set_config_path(args["config"])

    main()
