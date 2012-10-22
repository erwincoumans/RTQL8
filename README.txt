=====INSTALLATION FOR UNIX AND MAC OSX=====

1. Setup CMake
Download CMake packages from http://www.cmake.org/.

2. Build third party library, ticpp, in /thirdparty/ticpp
Follow the instructions provided by ticpp. Build libticpp.a and copy
it into /thirdparty/lib.

3. Build RTQL8 libraries and example apps
Go to /bin and type the following commands:
cmake ..
make

4. Test example apps
If you successfully built RTQL8, eight apps should be generated in
/bin (viewer, forwardSim, cubes, motionAnalysis, pdController,
balance, ik, and hybrid). You can run any of these apps to test if
RTQL8 is installed correctly.

That's it!

=====INSTALLATION DOXYGEN DOCUMENTS (OPTIONAL)=====

1. Setup Doxygen
Download and install Doxygen from http://www.stack.nl/~dimitri/doxygen/.

2. Create html and latex Doxygen documents for RTQL8
Go to /bin and type the following commands:
cmake ..
make docs

The documents are generated in /docs/html and /docs/latex

=====LEARNING HOW TO USE RTQL8=====

RTQL8 includes a comprehensive tutorial (RTQL8-tutorial.pdf ) to help
you start. For someone who is also interested in the dynamics of
multibody systems and collision routines, please read
dynamics-tutorial.pdf and lcp-tutorial.pdf.

rtql8-tutorial.pdf: This tutorial goes over the implementation of
every example apps (except for viewer) provided in RTQL8 package. It
is the most efficient way to learn how to write an app using
RTQL8. 

dynamics-tutorial.pdf: For those who want to understand the dynamic
libraries of RTQL8, this tutorial provides all the mathematics details
you need.

lcp-tutorial.pdf: This tutorial goes over the details of collision
handling using LCP formulation.
