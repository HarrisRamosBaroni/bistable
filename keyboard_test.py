import keyboard

def on_key_event(event):
    if event.name == 'w':
        print('W key pressed')
    elif event.name == 'a':
        print('A key pressed')
    elif event.name == 's':
        print('S key pressed')
    elif event.name == 'd':
        print('D key pressed')

keyboard.on_press(on_key_event)

keyboard.wait('esc')  # Wait for the 'esc' key to be pressed before exiting
