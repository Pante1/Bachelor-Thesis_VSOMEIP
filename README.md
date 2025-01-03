# Bachelor-Thesis_VSOMEIP

vSomeIP by COVESA:
This project utilizes the vSomeIP (Scalable Open Middleware for Embedded Protocols) framework, developed and maintained by the COVESA (Connected Vehicle Systems Alliance) community. vSomeIP is an open-source middleware solution designed for service-oriented communication in distributed embedded systems, particularly in the automotive domain. It is licensed under the Mozilla Public License 2.0 (MPL-2.0). For more information about vSomeIP and its documentation, visit the official GitHub repository: https://github.com/COVESA/vsomeip.

Two single-board computers establish a server-client connection utilizing the vSomeIP framework to facilitate data transfer. The server accesses a shared memory where another process stores information about detected car brands. This relevant data is retrieved by the server and transmitted to the client via vSomeIP's service-oriented communication protocol, ensuring efficient and reliable data exchange within the embedded system.
