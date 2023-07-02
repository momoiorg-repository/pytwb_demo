from pytwb.lib_main import initialize
import app

if __name__ == "__main__":
    initialize("/root/pytwb_ws", "pytwb_demo")
    app.app_main("sim")
