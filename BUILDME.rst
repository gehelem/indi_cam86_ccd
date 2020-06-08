First, libftdi is the tricky one
Try to follow this :
http://developer.intra2net.com/mailarchive/html/libftdi/2013/msg00014.html
(until step 4)

Then :
ssh to your pi and install compilation tools :

.. code-block:: shell

  sudo apt-get install build-essential gcc cmake git

install indilib dev prerequisites :

.. code-block:: shell

  sudo apt-get install cdbs libcfitsio-dev libnova-dev libusb-1.0-0-dev libjpeg-dev libusb-dev libtiff5-dev libftdi-dev fxload libkrb5-dev libcurl4-gnutls-dev libraw-dev libgphoto2-dev libgsl0-dev dkms libboost-regex-dev libgps-dev libdc1394-22-dev

install indilib dev :

.. code-block:: shell

  sudo apt-get install libindi-dev
  
  
Clone cam86 driver somewhere on your machine (here ~/tmp)

.. code-block:: shell

  cd
  mkdir tmp
  cd tmp
  git clone https://github.com/gehelem/indi_cam86_ccd.git
  cd indi_cam86_ccd

Then switch to my lastest code :

.. code-block:: shell

  git checkout workinprogress

Then compilation steps starts :

.. code-block:: shell

  mkdir build
  cd build
  cmake ..
  make

You have to manually isnstall udev rules file, i have to fix that :

.. code-block:: shell

  sudo cp ../99-cam86.rules /etc/udev/rules.d/
  
And restart udev :

.. code-block:: shell

  sudo service udev restart

plug your cam86

From here you can run the driver like that :

.. code-block:: shell

  indiserver -v -m 100 ./indi_cam86_ccd

you can also install the driver into your environemnt :

.. code-block:: shell

  sudo make install
  
with that one, you can use the driver like any other one.
