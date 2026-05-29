# mavpilot documentation

The API reference is **auto-generated from the source docstrings** with
[pdoc](https://pdoc.dev/) — there is no hand-maintained API doc to drift out of
sync with the code.

## Build locally

```bash
pip install -e ".[dev]"      # installs pdoc
scripts/build_docs.sh        # → docs/api/index.html
# or live-preview with hot reload:
python -m pdoc --docformat google mavpilot
```

The generated HTML lands in `docs/api/` (git-ignored — it's a build artifact).

## Published docs

On every push to `main` and every `v*` tag, the
[`Docs`](../.github/workflows/docs.yml) workflow rebuilds the reference and
deploys it to **GitHub Pages**: https://onikore.github.io/mavpilot/

> First-time setup: in the repo, **Settings → Pages → Build and deployment →
> Source = GitHub Actions**. Until that's enabled, the deploy step will be the
> only thing that fails; the build step still validates that docs generate.

## Narrative docs

Usage, the coordinate system, safety behaviour, and the architecture overview
live in the top-level [README.md](../README.md)
(and [README.ru.md](../README.ru.md) in Russian).
