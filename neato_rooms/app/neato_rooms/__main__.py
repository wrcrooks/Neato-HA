import logging

import uvicorn

from .api import create_app
from .config import Config


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    cfg = Config.load()
    logging.getLogger(__name__).info("backend=%s openneato=%s data=%s", cfg.robot, cfg.openneato_url, cfg.data_dir)
    uvicorn.run(create_app(cfg), host=cfg.host, port=cfg.port, log_level="info")


if __name__ == "__main__":
    main()
