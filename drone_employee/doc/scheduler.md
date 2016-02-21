# Simple scheduling for Drone Employee routes

The case is making periodicaly routes with autonomous 
decentralized administrative registration and releasing of route.

## Scheduler

For this case try to use UNIX Cron daemon and `run_route.js` script.
Sample **crontab** file below:

    0 * * * * node /path/to/run_route.js 0x7f43839b8e4c39d811da781ab191e885ca2b9508 '[[65.12, -39.17, 10], [65.11, -39.17, 10]]'

* **run_route.js** script takes two arguments: *DroneEmployee* contract address and waypoint list in format [latitude, longitude, altitude];
* the first five words is a crontab time format, in sample they says 'run every hour', detailed description available [there](https://en.wikipedia.org/wiki/Cron).

## Enviroment

**run_route.js** script require running *geth* or *eth* with enabled RPC port `8545`.

The scheduller communicates with standart DroneEmployee contract by ethereum node and can be run separately or on robot computer.


