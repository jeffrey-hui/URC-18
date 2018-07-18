=====================
Controlling the rover
=====================

Now that you've started the rover, you need to control it. The first thing you need to do is start the watchdog client.

.. note::
   The watchdog automatically stops the rover if the basestation is not connected for more than 1 second.

To start the watchdog client, use ``$ rosrun rover wclient.py``
   
