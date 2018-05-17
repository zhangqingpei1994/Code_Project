* To run examples you may copy the files from "lib" folder to usr/lib

* To build examples from command line:
 For "simple" example:
   change directory to examples/eclipse/simple and type:
   sudo g++ main.cpp -I../../../include -L../../../lib -lOMD -o simple
 For "advanced" example:
   change directory to examples/eclipse/advanced and type
   sudo g++ main.cpp -I../../../include -L../../../lib -lOMD -o advanced

* To run examples:
  sudo ./simple
  sudo ./advanced 

Superuser rights might be necessary to access the DAQ through ttyACM0.

The most current version and the documentation of the API can be downloaded from http://www.optoforce.com/support.
