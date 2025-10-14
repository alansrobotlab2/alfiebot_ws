# alfiebot_ws

### ROS2 Humble workspace for Alfiebot

Some setup instructions (there are probably more, but here's what I have for now.
 

    sudo apt-get install portaudio19-dev python3-pyaudio python3-venv ros-humble-foxglove-bridge ros-humble-depthai-ros tmux
     
    pip3 install pyusb spidev piper-tts pyalsaaudio onnx-asr silero-vad qdarktheme

    ros2 run micro_ros_setup create_agent_ws.sh
    ros2 run micro_ros_setup build_agent.sh

  

seeedstudio respeaker usb setup

    echo 'SUBSYSTEM=="usb", MODE="0666"' | sudo tee /etc/udev/rules.d/99-usb-all.rules
    sudo udevadm control --reload-rules
    sudo udevadm trigger

  you'll need jetson-containers installed:
  https://github.com/dusty-nv/jetson-containers

then copy over alfiebot_ws/src/alfie_llm/scripts/convert.sh over to ~/repos/jetson-containers/data and run it to generate your mlc-llm of choice like:

    convert.sh "Qwen/Qwen3-4B" "q4f16_ft"



access to USB serial

    sudo usermod -aG dialout $USER

then reboot.



To Do:




