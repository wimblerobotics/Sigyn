Import("env")
import os
import sys

def check_upload_port(source, target, env):
    upload_port = env.get("UPLOAD_PORT")
    # If upload_port is set and is an absolute path (starts with /), check if it exists
    if upload_port and upload_port.startswith("/") and not os.path.exists(upload_port):
        sys.stderr.write(f"\n\033[91mError: Upload port '{upload_port}' does not exist.\033[0m\n")
        sys.stderr.write(f"\033[93mPlease ensure the device is connected and the path is correct.\033[0m\n")
        # Exit with error to stop the upload process
        sys.exit(1)

# Register the callback for the "upload" target
env.AddPreAction("upload", check_upload_port)
