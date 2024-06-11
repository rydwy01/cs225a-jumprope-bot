from pynput import keyboard
import redis

# Initialize Redis connection
redisKey = "keyboard"
redis_client = redis.Redis(host='localhost', port=6379, db=0)

def on_press(key):
    try:
        # Capture alphanumeric keys
        input = key.char
        print(key.char)
        if input == "8" or input == "9" or input == "0" or input == "1":
            print(key)
            check = redis_client.set(redisKey, f'{input}')
            print(check)

    except AttributeError:
        pass
    #     # Capture special keys (e.g., space, enter, etc.)
    #     redis_client.set(redisKey, f'{key}')

def on_release(key):
    # Stop listener if the escape key is pressed
    if key == keyboard.Key.esc:
        return False

# Setup the listener
with keyboard.Listener(
        on_press=on_press,
        on_release=on_release) as listener:
    listener.join()