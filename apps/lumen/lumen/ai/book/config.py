"""BOOK model: book detector (see ai/book/detector.py; the model itself is still being chosen,
benchmarks on the lumen-book branch). Edit here."""

# --- When detections become events (ai/__init__.py) ----------------------------------------------
BOOK_FOUND_CONSECUTIVE = 2               # BOOK_FOUND after this many hits in a row
BOOK_LOST_S = 2.0                        # BOOK_LOST needs the book gone this long...
BOOK_LOST_MISSES = 3                     # ...and this many misses in a row
BOOK_CONFIRM_HZ = 3.0                    # after a missed check, re-check this fast
BOOK_PER_POSE = 2                        # SEARCH: book inferences per settled sweep pause
