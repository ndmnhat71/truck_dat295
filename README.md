# truck_dat295
This is DAT295 project.

1. How to implement:
- Control running/stop by toggle the variable self.lock_stop of auto_master class
- Need a Scubcriber (or just a function) to manage this: 
    + If the current possition is in range A from a certain point (VTL):
        . stop the truck
        . wait for a period of time / until some object move out or the area
          depend on how many objects can be track by GulliView)
        . release the lock
    + End if
- How to test?

