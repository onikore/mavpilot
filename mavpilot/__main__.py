"""python -m mavpilot entrypoint."""

import asyncio
import contextlib
import sys

from .cli import main

if __name__ == "__main__":
    with contextlib.suppress(KeyboardInterrupt):
        asyncio.run(main())
    sys.exit(0)
