"""AI (shared): when raw detections become events, and where the models look. Edit here.

Each model has its own config next to it: ai/face/config.py, ai/hands/config.py,
ai/book/config.py, ai/motion_gate/config.py. Which models run in which state, and how often,
is the state table in state/machine.py (STATES), since it is part of each state's definition.
"""

# --- Face events (ai/__init__.py) ---------------------------------------------------------------
FACE_SEEN_K, FACE_SEEN_N = 1, 1          # IDLE -> NOTICE on the first face
FACE_CONFIRM_K, FACE_CONFIRM_N = 3, 5    # NOTICE -> STANDBY
FACE_LOST_S = 4.0                        # STANDBY -> LOST USER

# --- Presence ---------------------------------------------------------------------------------
NO_MOTION_SLEEP_S = 300.0                # IDLE -> SLEEP
NOBODY_S = 180.0                         # ILLUMINATE -> GOODBYE: no face, hand or motion

# --- Crops, as multiples of the box they grow from: (left, top, right, bottom) margins ----------
HAND_ROI_FROM_FACE = (1.5, 0.5, 1.5, 3.0)   # gestures happen beside and below the face
HAND_ROI_FROM_BOOK = (0.5, 0.5, 0.5, 0.5)   # over the lit book
BOOK_ROI_FROM_BOOK = (0.5, 0.5, 0.5, 0.5)   # track the book near where it was

# --- Scheduler (ai/scheduler.py) ----------------------------------------------------------------
SLACK_S = 0.01   # a frame this much early still counts: at 2 fps in SLEEP the frames are ~0.5 s apart
