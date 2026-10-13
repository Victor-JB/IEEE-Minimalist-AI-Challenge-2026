"""WEB PAGE: the one page on :8080 (HUD + desk controls + Auto/Manual switch). Edit here."""

# Mode at boot. "manual": the arm stays still until someone presses Auto on the page (what the
# luxoIII setup chose: nothing moves by itself after a power-up). "auto": Lumen runs right away.
START_MODE = "manual"
VIDEO_QUALITY = 70                       # JPEG quality of the page's camera feed (0-100)
