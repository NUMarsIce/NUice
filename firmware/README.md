# Arduino CLI install [how to use](https://arduino.github.io/arduino-cli/latest/getting-started/)
```bash
echo "export PATH=\$PATH:~/bin" >> ~/.bashrc
source ~/.bashrc
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

arduino-cli config init --additional-urls https://github.com/stm32duino/BoardManagerFiles/raw/master/STM32/package_stm_index.json
arduino-cli core update-index
arduino-cli core install arduino:avr
arduino-cli core install STM32:stm32
```

# STM programmer
Download STM32CubePrg-Lin from [here](https://www.st.com/en/development-tools/stm32cubeprog.html). Note that to download you only need to input your email so that they can send you the download in an email. Make sure to choose `Save file` when prompted.

When it downloads, we can go ahead and install it.
```bash
cd ~/Downloads
mkdir stm32prog
unzip *stm32cubeprg*.zip -d stm32prog/

echo "export PATH="\$PATH:$HOME/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin" >> ~/.bashrc
```

# other
```bash
arduino-cli sketch new sample_sketch

arduino-cli completion bash > arduino-cli.sh
sudo mv arduino-cli.sh /etc/bash_completion.d/

arduino-cli compile -b STM32:stm32:GenF4:pnum=Generic_F401RE,upload_method=dfuMethod,xserial=generic,usb=CDCgen,xusb=FS,opt=osstd,rtlib=nano -u
```