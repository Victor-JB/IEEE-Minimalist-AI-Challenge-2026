# luxoiii applications

`luxoiii/` is the single product application. Its Python package is `luxoiii`; the manual desk and
full-model preview are subcommands of the same package.

| Command (from `apps/luxoiii`) | Purpose |
|---|---|
| `python -m luxoiii --headless --stream 8080` | Camera, AI and behavior; HUD at `/`, manual page at `/desk` |
| `python -m luxoiii.desk --port 8081` | Standalone manual desk |
| `python -m luxoiii.preview --port 8082` | Read-only full-model animation, including X-ray case |
| `python -m luxoiii --fake-arm --stream 8080` | Camera and AI with simulated motion; no physical arm connection |

The main app uses the MCU firmware in `arm/`; it does not use App Lab to flash firmware on every launch.
The main app and standalone desk must not own the camera and arm simultaneously. Their systemd units
conflict; the integrated `/desk` page transfers ownership within one process.

Lamp clip authoring tools previously under `apps/stepper_desk` now live in `tools/animation`.
Historical branch paths remain unchanged in dated notes and source provenance.
