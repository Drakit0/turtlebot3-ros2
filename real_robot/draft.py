from sshkeyboard import listen_keyboard

def on_press(key):
    print(f"Key pressed: {type(key)}")

listen_keyboard(on_press)