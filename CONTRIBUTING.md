# Contributing

Keep hardware reads and motor actuation in separate modules. Changes to the
default transport must preserve the `PING`/`READ`/`SYNC_READ` allow-list and add
tests proving no write-class packet can be built.

Before opening a pull request:

```bash
python -m pytest
python -m ruff check src tests
```

Do not commit serial numbers, absolute home-directory paths, calibration
artifacts, run logs, videos, hidden evaluation states, or model checkpoints.
