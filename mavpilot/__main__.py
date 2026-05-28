"""python -m mavpilot entrypoint."""
import asyncio
import sys

from .cli import main

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
    sys.exit(0)
