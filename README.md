bcc_lib
=======

Blank Canvas Cape user-space driver

This project provides a user-space application for configuring the
Blank Canvas Cape (BCC), a Beaglebone cape which provides a Xilinx
Spartan 3A FPGA connected to the Beaglebone I/O connectors. Find out
more details about the BCC here:

http://ebrombaugh.studionebula.com/embedded/bcc/index.html

In addition to the application, an underlying interface library is
also provided which facilitates creation of additional applications
for interfacing to the FPGA via the SPI interface.

bcc_tool
========

This is the application. Usage is as follows:

bcc_tool [-f freq][-p][-r addr][-w addr data][-v][-V] [BITSTREAM]

BITSTREAM is a file containing the FPGA bitstream to download to the
FPGA or configuration flash. With no other options the application will
quietly configure the FPGA with the contents of BITSTREAM.

-p program the SPI Flash configuration memory with BITSTREAM.

   This option redirects the contents of BITSTREAM into the on-board
   configuration flash memory so that it is immediately available on
   power-up.
   
   Note that in order to route the SPI signals to the flash memory it
   is necessary to first configure the FPGA with a special bitstream
   that provides the proper connections. This bitstream is provided at
   the above website, along with full source code for creating it.
   
-r read SPI control port at addr

   This option allows a quick check of SPI communications with the
   SPI slave port in the FPGA by reading from the slave port at the
   provided address. This can also be used to check status and results
   from any functions attached to the SPI slave.

-w write SPI control port at addr w/ data
  
   This option allows a quick check of SPI communications with the
   SPI slave port in the FPGA by writing data to a specific address.
   This can also be used to control & configure any functions that are
   attached to the SPI slave.
   
-v enables verbose progress messages

   This option provides detailed progress information on the process of
   communicating with the FPGA. Use this to debug any problems with the
   application.
   
-V prints the tool version

