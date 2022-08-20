# recommended for opencv
sudo apt install -y libopenjp2-7 libilmbase-dev libopenexr-dev libgstreamer1.0-dev ffmpeg
# required to build library stepper motor
sudo apt install -y cmake
# library to interact with raspberry pi pins
wget http://www.airspayce.com/mikem/bcm2835/bcm2835-1.70.tar.gz 
tar zxvf bcm2835-1.70.tar.gz
cd bcm2835-1.70
./configure
# fPIC is needed for python TMCStepper library
make CFLAGS="-g -O2 -fPIC"
sudo make install
cd ..
rm -r bcm2835-1.70
rm bcm2835-1*
# fomu-flash to flash fpga
git clone https://github.com/hstarmans/fomu-flash
cd fomu-flash
make
sudo make install
cd ..
sudo rm -r fomu-flash
# stepper motor drivers, poetry cannot clone with recurse submodules
git clone --recurse-submodules https://github.com/hstarmans/TMCStepper
cd TMCStepper
poetry run python3 setup.py install
cd ..
sudo rm -r TMCStepper
sudo apt install ghostscript 
