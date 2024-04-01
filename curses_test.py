import curses

# Initialize curses
stdscr = curses.initscr()
curses.cbreak()
stdscr.keypad(True)

try:
    while True:
        # Get the key pressed
        key = stdscr.getch()

        # Check if it's one of the arrow keys
        if key == curses.KEY_UP:
            print('Up key pressed')
        elif key == curses.KEY_DOWN:
            print('Down key pressed')
        elif key == curses.KEY_LEFT:
            print('Left key pressed')
        elif key == curses.KEY_RIGHT:
            print('Right key pressed')

finally:
    # Clean up and restore terminal settings
    curses.nocbreak()
    stdscr.keypad(False)
    curses.echo()
    curses.endwin()