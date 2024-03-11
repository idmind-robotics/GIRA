# GIRA

#"lib" 
Contains helper functions. This can be left alone as is.

 "src"/"srcAux" : These folders should contain the same files. These contain mostly classes related to hardware peripherals, namely interfacing with the buzzer, power sensors, SIM module and IMU. The "hw.py" file handles all these components and contains the reading and updating of values associated with them, aswell as control logic. 
		   The main program uses the files contained in the "src" folder. At boot, the contents of the folders should be compared. If srcAux is different, copy them into src (This portion is implemented on the first version, check there) . This can be used to implement remote updates using SIM808 module (save the new files into srcAux, then reboot, and they will go into src).

 "testing" : Contains scripts that can be ran as standalone to test the current sensors and the IMU independetly of the rest of the components. 

 "boot.py" : The code here is the first to run when the system is powered on or resets. Reprograming through SrcAux/Src should be implemented here. 

 "main.py" : Contains the main functioning loop of the program. Starts by checking if the battery is connected to the IDMINDCONFIGIRA program and retrieving initial configuration parameters from "bike_id" and "msgFreq" files.
              Initializes different peripherals and communications. Runs communication and hardware routines periodically.

 "bike_id.txt" : Saves information about the bike's identification (RFID & Registry) as a JSON: {'BIKERFID': '1111X', 'BIKEREGISTRY': '1111X'}

 "msgFreq.txt" : Saves information about how often to send data to the API as an int in seconds.

"IDMIND_GIRA_CONFIG.rar" : Zip file containing the CLI application developed for EMEL to interface with batteries for updating the bikes identification, test and debug different components.
