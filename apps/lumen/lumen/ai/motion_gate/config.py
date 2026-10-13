"""MOTION GATE: the free "model", a frame difference. Nothing moving -> no neural network runs.
Edit here."""

MOTION_THRESHOLD = 6.0                   # mean abs diff (0-255) on the gray thumbnail
THUMBNAIL = (80, 60)                     # frames are shrunk to this before differencing
