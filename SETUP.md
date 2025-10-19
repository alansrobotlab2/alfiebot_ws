https://wiki.seeedstudio.com/recomputer_jetson_mini_getting_started

#### let's get everything updated
```
sudo apt update
sudo apt install -y git git-lfs nano net-tools curl btop cmake
sudo apt dist-upgrade -y
```

#### set to 2.4ghz wifi by default?
```
sudo nmcli connection modify "ShoppingCart" 802-11-wireless.band bg  
```

#### disable gdm desktop
```
sudo systemctl disable gdm
sudo systemctl stop gdm
```


#### increase swap to 60gb
```
# needed to compile tensorrt_llm
# updates swap to ~ 61Gb
sudo sed -i 's|/ 2 /|* 4 /|g' /etc/systemd/nvzramconfig.sh
```


#### Install jtop
https://github.com/rbonghi/jetson_stats
```
sudo apt install python3-pip
sudo pip3 install -U jetson-stats
```


#### Disable suspend
```
sudo systemctl mask suspend.target
gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-timeout 0  
gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-battery-timeout 0
```


#### eza
```
sudo apt update  
sudo apt install -y gpg
sudo mkdir -p /etc/apt/keyrings
wget -qO- https://raw.githubusercontent.com/eza-community/eza/main/deb.asc | sudo gpg --dearmor -o /etc/apt/keyrings/gierens.gpg
echo "deb [signed-by=/etc/apt/keyrings/gierens.gpg] http://deb.gierens.de stable main" | sudo tee /etc/apt/sources.list.d/gierens.list
sudo chmod 644 /etc/apt/keyrings/gierens.gpg /etc/apt/sources.list.d/gierens.list
sudo apt update
sudo apt install -y eza
```


#### fastfetch
```
sudo add-apt-repository ppa:zhangsongcui3371/fastfetch
sudo apt update
sudo apt install fastfetch
```

#### oh my zsh
https://ohmyz.sh/#install
https://github.com/zsh-users/zsh-autosuggestions/blob/master/INSTALL.md
https://github.com/zsh-users/zsh-syntax-highlighting/blob/master/INSTALL.md
https://github.com/zdharma-continuum/fast-syntax-highlighting?tab=readme-ov-file#installation

```
sudo apt install zsh
chsh -s /bin/zsh
exec zsh

rm ~/.zshrc

sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"
git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions
git clone https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting
git clone https://github.com/zdharma-continuum/fast-syntax-highlighting.git \
  ${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}/plugins/fast-syntax-highlighting
# update theme to gnzh
sed -i 's|robbyrussell|gnzh|g' ~/.zshrc
# enable the plugins
sed -i '/^plugins=(git)$/c\plugins=(\n    git\n    zsh-autosuggestions\n    zsh-syntax-highlighting\n    fast-syntax-highlighting\n)' ~/.zshrc
# add alias for eza
echo "alias ls='eza -a --icons=always'" >> ~/.zshrc

# enable arg autocomplete for zsh
echo 'eval "$(register-python-argcomplete3 ros2)"' >> ~/.zshrc
echo 'eval "$(register-python-argcomplete3 colcon)"' >> ~/.zshrc
```


#### Upgrade Jetpack compute to 6.2
https://docs.nvidia.com/jetson/jetpack/install-setup/index.html#upgradable-compute-stack

```
sudo apt remove -y nvidia-cuda-dev

# let's clean out the apt lists details
sudo apt clean                       # drop any stale partials
sudo rm -rf /var/lib/apt/lists/*

sudo echo "deb https://repo.download.nvidia.com/jetson/common r36.4 main" | sudo tee -a /etc/apt/sources.list.d/nvidia-l4t-apt-source.list

sudo echo "deb https://repo.download.nvidia.com/jetson/t234 r36.4 main" | sudo tee -a /etc/apt/sources.list.d/nvidia-l4t-apt-source.list

sudo apt update

sudo apt install -y nvidia-jetpack nvidia-cuda-dev nvidia-cudnn

# now take out those r36.4 lines so you don't bunk anything else up
sudo sed -i '/deb https:\/\/repo.download.nvidia.com\/jetson\/common r36.4 main/d' /etc/apt/sources.list.d/nvidia-l4t-apt-source.list
sudo sed -i '/deb https:\/\/repo.download.nvidia.com\/jetson\/t234 r36.4 main/d' /etc/apt/sources.list.d/nvidia-l4t-apt-source.list

# let's clean out the apt lists details again
sudo apt clean                       # drop any stale partials
sudo rm -rf /var/lib/apt/lists/*

# pin these packages 
# as the version number format changed
# and they'll get downgraded during apt upgrade
sudo apt-mark hold nvidia-cuda-dev
sudo apt-mark hold nvidia-cudnn
apt-mark showhold

cat /etc/apt/sources.list.d/nvidia-l4t-apt-source.list 
```


#### ROS2 Humble (22.04)
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
```
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update

sudo apt upgrade

sudo apt install ros-humble-desktop

sudo apt install ros-dev-tools
```

#### jetson-containers
https://github.com/dusty-nv/jetson-containers
```
# install the container tools
cd ~
git clone https://github.com/dusty-nv/jetson-containers
bash jetson-containers/install.sh
```


#### set up alfiebot_ws
```
cd ~
git clone https://github.com/alansrobotlab2/alfiebot_ws.git

echo '' >> ~/.zshrc
echo '# sourcing for ros2 humble and alfiebot_ws packages' >> ~/.zshrc
echo 'source /opt/ros/humble/setup.zsh' >> ~/.zshrc
echo 'source ~/alfiebot_ws/install/setup.zsh' >> ~/.zshrc

source ~/.zshrc

cd ~/alfiebot_ws
colcon build
source ~/.zshrc

# some more installs
sudo apt-get install portaudio19-dev python3-pyaudio python3-venv ros-humble-foxglove-bridge ros-humble-depthai-ros tmux
 
pip3 install pyusb spidev piper-tts pyalsaaudio onnx onnx-asr silero-vad pyqtdarktheme pyserial

sudo rosdep init
rosdep update

rm -rf ~/alfiebot_ws/src/micro_ros_setup
rm -rf ~/alfiebot_ws/src/uros

cd ~/alfiebot_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Update dependencies using rosdep
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

# Install pip
sudo apt-get install python3-pip

# Build micro-ROS tools and source them
colcon build

ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
```

#### seeedstudio respeaker usb setup
```
echo 'SUBSYSTEM=="usb", MODE="0666"' | sudo tee /etc/udev/rules.d/99-usb-all.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

#### set up access to usbserial for the Waveshare General Driver Board
```
sudo usermod -aG dialout $USER
```

#### convert your llm model of choice to mlc_llm
```
cp ~/alfiebot_ws/src/alfie_llm/scripts/convert.sh ~/jetson-containers/data/

jetson-containers run \
  dustynv/mlc:0.20.0-r36.4.0
```
then...
```
cd /data
./convert.sh "Qwen/Qwen3-1.7B" "q4f16_1-MLC"
```

#### set up for parakeet asr
```
# Downgrade to NumPy 1.x (compatible with the pre-built wheel) 
pip3 install "numpy<2" huggingface_hub

# Remove the wrong version
pip3 uninstall onnxruntime onnxruntime-gpu -y

# Download and install the Python 3.10 wheel for JetPack 6
wget https://nvidia.box.com/shared/static/48dtuob7meiw6ebgfsfqakc9vse62sg4.whl -O onnxruntime_gpu-1.18.0-cp310-cp310-linux_aarch64.whl
pip3 install onnxruntime_gpu-1.18.0-cp310-cp310-linux_aarch64.whl

# Test it
python3 -c "import onnxruntime as ort; print(ort.get_available_providers())"
```

You should now see:
```
['TensorrtExecutionProvider', 'CUDAExecutionProvider', 'CPUExecutionProvider']
```
## Everything below is not currently necessary

#### docker
https://www.digitalocean.com/community/tutorials/how-to-install-and-use-docker-on-ubuntu-22-04
```
sudo apt update
sudo apt install apt-transport-https ca-certificates curl software-properties-common
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt update
apt-cache policy docker-ce
sudo apt-get install -y docker-ce=5:27.5\* docker-ce-cli=5:27.5\* --allow-downgrades
sudo systemctl enable docker
sudo systemctl start docker
sudo systemctl status docker
sudo usermod -aG docker $USER 
```

#### uv
https://docs.astral.sh/uv/getting-started/installation/
```
curl -LsSf https://astral.sh/uv/install.sh | sh
```


#### distrobox
https://nathanaelgandhi.com/computing/linux/2023/03/16/Setting-Up-Distrobox-on-Ubuntu-22.04.html
```
sudo add-apt-repository ppa:michel-slm/distrobox
sudo apt update
sudo apt install distrobox -y
```

sudo nano /etc/containers/registries.conf
add
```
unqualified-search-registries = ["docker.io", "quay.io", "ghcr.io", "nvcr.io"]
```

