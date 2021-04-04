# Arduino CLI install ([how to use](https://arduino.github.io/arduino-cli/latest/getting-started/))
This is a way for us to compile and upload our arduino code from the terminal.
```bash
cd ~/
echo "export PATH=\$PATH:~/bin" >> ~/.bashrc
source ~/.bashrc
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

arduino-cli config init --additional-urls https://github.com/stm32duino/BoardManagerFiles/raw/master/STM32/package_stm_index.json
arduino-cli core update-index
arduino-cli core install arduino:avr
arduino-cli core install STM32:stm32
```

# STMCubeProgrammer install (optional)
This is needed to upload code to the STM32s.

Download STM32CubePrg-Lin from [here](https://www.st.com/en/development-tools/stm32cubeprog.html). Note that to download you only need to input your email so that they can send you the download in an email. Make sure to choose `Save file` when prompted.

When it downloads, we can go ahead and install it. When you get the install window just power through without changing any defaults.
```bash
#install java
sudo apt install openjdk-8-jre-headless
sudo apt install openjfx=8u161-b12-1ubuntu2 libopenjfx-jni=8u161-b12-1ubuntu2 libopenjfx-java=8u161-b12-1ubuntu2
sudo apt-mark hold openjfx libopenjfx-jni libopenjfx-java

#install STMCubeProgrammer
cd ~/Downloads
mkdir stm32prog
unzip *stm32cube*.zip -d stm32prog/
./stm32prog/Setup*.linux

echo "export PATH="\$PATH:$HOME/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin" >> ~/.bashrc
```

# Other
Add autocompletion to arduino-cli
```bash
arduino-cli completion bash > arduino-cli.sh
sudo mv arduino-cli.sh /etc/bash_completion.d/
```

Compile and upload (`-u`) the code including the libraries in `firmware/libraries`. This must be run in the sketch folder
```bash
arduino-cli compile -b STM32:stm32:GenF4:pnum=Generic_F401RE,upload_method=dfuMethod,xserial=generic,usb=CDCgen,xusb=FS,opt=osstd,rtlib=nano --libraries ../../libraries --clean -u
```
Add `--clean` if things are being wack, as messing with adding libraries might need this.

Update ros_lib (if new custom messages are made)
```bash
cd ~/NUice/firmware/lib
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
```