#!/bin/bash
# run_drivers.sh

# Exit immediately if any command fails
set -e

# Path to the virtual environment
VENV_PYTHON="/home/user/venv/bin/python3"

# Run the first driver script
(trap 'kill 0' SIGINT;
sudo "$VENV_PYTHON" /home/user/inspire_hand_sdk/example/Headless_driver_485_l.py & 


# Run the second driver script
sudo "$VENV_PYTHON" /home/user/inspire_hand_sdk/example/Headless_driver_485_r.py)


wait
