AD-R1M Production Guide
=======================

Parts sourcing
--------------

TODO: BOM, off-the-shelf components, 3D printed components

Assembly
--------

TODO: mechanical, electrical assembly

.. tab:: Raspberry Pi 5

   .. todo::

      mechanical, electrical assembly for Raspberry Pi 5 variant

.. tab:: Nvidia AGX Orin

   .. todo::

      mechanical, electrical assembly for Nvidia AGX Orin variant

Flashing & Linux setup
----------------------

Download linux image and flash it to the device.

.. tab:: Raspberry Pi 5

        Download, write SD card, be happy

.. tab:: Nvidia AGX Orin

        TODO???

Check hostname, optionally change::

	$ hostname
	ad-r1m
	$ sudo sed -i 's/ad-r1m/{newhostname}/g' /etc/hosts
	$ sudo sed -i 's/ad-r1m/{newhostname}/g' /etc/hostname
	$ sudo hostname {newhostname}

Check IP address::

	$ ip a
	TODO: output, highlight

Check peripherals: CAN (TODO), libiio (TODO), ToF (TODO), UART (TODO)

Check docker container tag::

	$ docker ps -a
	TODO: output

Pull and instantiate latest docker image::

	$ docker pull idkorg/idktag
	$ docker run --flags-todo idkorg/idktag

TODO: switch to docker compose?

TODO: create a script to run all of these automatically and present them on the screen, for easy startup troubleshooting?
