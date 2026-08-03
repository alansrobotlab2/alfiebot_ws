uint8 firmwaremajor     
uint8 firmwaresub  
uint8 servomajor    
uint8 servosub       
uint8 servoid       
uint8 baudrate
uint8 returndelay 
uint8 responsestatuslevel
uint16 minanglelimit
uint16 maxanglelimit
uint8 maxtemplimit
# Fields are in register-address order. 0x0E is the MAXIMUM input voltage and
# 0x0F the minimum, per the ST3215 vendor map - they used to be named the other
# way round here, which showed the two limits swapped.
uint8 maxinputvoltage
uint8 mininputvoltage
uint16 maxtorque
uint8 phase
uint8 unloadingcondition
uint8 ledalarmcondition
uint8 pcoefficient
uint8 dcoefficient
uint8 icoefficient
uint16 minstartupforce
uint8 clockwiseinsensitivearea
uint8 counterclockwiseinsensitiveregion
uint16 protectioncurrent
uint8 angularresolution
int16 positioncorrection
uint8 operationmode
uint8 protectivetorque
uint8 protectiontime
uint8 overloadtorque
uint8 speedclosedlooppcoefficient
uint8 overcurrentprotectiontime
uint8 velocityclosedloopicoefficient
uint8 torqueswitch       
uint8 acceleration       
int16 targetlocation    
uint16 runningtime       
uint16 runningspeed      
uint16 torquelimit        
uint8 lockmark
int16 currentlocation
int16 currentspeed    
uint16 currentload       
uint8 currentvoltage     
uint8 currenttemperature 
uint8 asyncwriteflag     
uint8 servostatus        
uint8 mobilesign         
uint16 currentcurrent  

# Outcome fields, filled in by the module firmware's register service.
#
# writebyteresult carries the write outcome for ALL write operations (byte,
# word, lock, unlock), mirroring the SERVO_WRITE_* codes in the firmware config:
#   0 none (a plain read)   1 ok                2 address not writable
#   3 EEPROM locked         4 torque enabled    5 servo did not acknowledge
#   6 wrote but the read-back disagrees         7 value does not fit the register
uint8 writebyteresult
# Width actually written, in bytes: 0 (no write), 1 or 2.
uint8 writewordresult
# 1 when memorymap holds a fresh, complete read. Every operation ends with a
# re-read, so this is set for successful writes too.
uint8 readmemoryresult