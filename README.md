# MOC-SOC_FlightComputer

Written by Paul Wehling with assistance from Lilia Bouayed, based on code originally written by Jordan Bartlett and Emma Krieg.
For use with MOC-SOC 3U Cubesat 1.2, built by Paul Wehling and Donald Rowell, based on original by Jordan Bartlett and Emma Krieg.
Last updated August 10, 2020

Commands to be used with this code:
QUERY: Reports essential status information, has no actual effect on MOC-SOC. Useful for testing radio connection.
TEMP: Reports current temperature in celsius.
GPS: Reports last GPS data, as well as the time of last lock that passed GPS filter.
GEIGER: Reports geiger counter hits, both total and since last logging cycle. NOTE: WILL REFRESH GEIGER COUNTER CYCLE HITS
VOLT: Reports voltage and current information from solar panels.
DEPLOY: Attempts to deploy solar panel mounts. It is inadvisable to trigger before PPOD has opened.
PREPDEPLOY: Removes need for MOC-SOC to recieve go-ahead from PPOD to deploy.
UNPREPDEPLOY: Undoes PREPDEPLOY command. Will not stop deployment if PPOD is broacasting a deployed signal.
START: Overrides remove-before-flight pin, and begins data collection and flight timer.
ACGPS: Overrides GPS filter for next GPS data intake. Only use in event of mission-critical failure of GPS filter.
RSGPS: Spams calls to update GPS. May have a better chance of achieving a lock than normal operation.
TOGGLERADIOMODE: Switches smart telemetry on or off.
FENCE: Allows changing of GPS fences. Ex: FENCEN-200 would set the north fence to -200, or off.
SETDEPLOY: Toggles deployment booleans, G for GPS, A for altitude, T for timer, D for descent, P for forced PPOD deploy.
SETT: Sets the time till deployment of the solar panel mounts in minutes. Ex: SETT35 will tell MOC-SOC to deploy in 35 minutes.
SETC: Sets data and radio cycle time in seconds. Ex: SETC60 will tell MOC-SOC to cycle every minute.
+: Adds time to time till deployment timer in minutes. Ex: +30 will add half an hour until MOC-SOC deploys.
-: Subtracts time to time till deployment timer in minutes. Ex: -30 will subtract half an hour until MOC-SOC deploys.
P: Forwards message to connected XBEE with ID POD or PPOD. Ex: P+15 will send +15 command across XBEE network.


Future Features:
LOWPOWER command: turns off radio
READPOWER command: returns current voltage produced by batteries and estimation of service life left.
First line for data .csv sheet specifing what data is in what columns
COMMS compatability
RESET command: reruns sdSetup(), radioAndGpsSetup, and sensorsSetup() if data is bad
HARDRESET command: restarts entire cubesat
Pressure sensor
