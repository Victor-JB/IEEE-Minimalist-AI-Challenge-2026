"""Lumen Desk: a browser control page for the arm, served from the board.

    python -m lumen.desk                 # http://luxo3:8081/  (stop Lumen first: one arm owner)

What the Stepper Desk Mac app did, on the closed-loop firmware, plus the camera and head tilt:
live joints and sensors, enable/lock, automatic driver power per move, hold-to-jog, pose targets, taught poses, clips,
the measured-vs-commanded plot, recordings and replay. arm.py owns the arm, video.py the camera,
server.py the HTTP side, page.html the page.
"""
