# IO Assignments

## Digital IO
* DIO 0: Practice Robot Jumper
* DIO 1: Drive RF Absolute Encoder
* DIO 2: Drive LF Absolute Encoder
* DIO 3: Drive LR Absolute Encoder
* DIO 4: Drive RR Absolute Encoder
* ~~DIO 7: Shoulder CTRE Mag Encoder~~

## Analog IO

## PWM
* PWM 0: Blinky Lights
* PWM 1: Blinken
* PWM 2: Blinken

## Motor Controllers

# PDB assignments

# Vision

# Driver Controller

* L/R X/Y: drive

* A/SWA: reset NavX
* LB/SWF: turn off vision
* X/SWD: X mode
* LT/SWE: intake
* RT/SWH: shoot
* B/SWC: outtake barf

---

# Things to do different next year

* Angular stuff should use Rotation2d, eliminates this year's Mars Orbiter degrees/radians screwup.
* All results from Vision should be in one container (so all data comes from same data read).
* Use ErrorMessages.requireNonNullParam in constructors.

---

# Recalibrating shooter elevation

* Request 32.5 degrees from Dashboard (this is the angle that is good for a shot when the vision says the distance to target is 10.7 feet).
* Use level to check angle (lay it across the tops of the bottom churro). It should read 32.5 degrees.
* If the angle is significantly off, then:
    * Fiddle the dashboard until you have 32.5 degrees actual on the level.
    * Note what the dashboard says (in our example, was 28 degrees)
    * Subtract what the dashboard says from 32.5. This is the "adjustment amount". In our case, 32.5 - 28 = 4.5.
    * Find the shooterElevationHomePosition variable in ShooterSubsystem. Add the "adjustment amount" to it's value, and put the new value into the code. In our example, shooterElevationHomePosition was 64, we added 4.5 to it, and changed it to 69 (should have been 68.5, but we were doing math in our head).
    * Deploy code. Requesting 32.5 should give you 32.5 on the level.

---

# Adding swap space

* Partition thumb drive

```
parted /dev/sda
(parted) rm 1
(parted) rm 2
(parted) mkpart
Partition type?  primary/extended? primary
File system type?  [ext2]? linux-swap
Start? 0%
End? 1g
(parted) print
Model:  USB  SanDisk 3.2Gen1 (scsi)
Disk /dev/sda: 30.8GB
Sector size (logical/physical): 512B/512B
Partition Table: msdos
Disk Flags:

Number  Start   End     Size   Type     File system     Flags
 1      1049kB  1000MB  999MB  primary  linux-swap(v1)  lba
(parted) mkpart
Partition type?  primary/extended? primary
File system type?  [ext2]? ext
parted: invalid token: ext
File system type?  [ext2]? 1g
parted: invalid token: 1g
File system type?  [ext2]? ext4
Start? 1g
End? -1s
(parted) print
Model:  USB  SanDisk 3.2Gen1 (scsi)
Disk /dev/sda: 30.8GB
Sector size (logical/physical): 512B/512B
Partition Table: msdos
Disk Flags:

Number  Start   End     Size    Type     File system     Flags
 1      1049kB  1000MB  999MB   primary  linux-swap(v1)
 2      1000MB  30.8GB  29.8GB  primary

(parted) quit
admin@roboRIO-3620-FRC:~# mkswap /dev/sda1
mkswap: /dev/sda1: warning: wiping old swap signature.
Setting up swapspace version 1, size = 953 MiB (999288832 bytes)
no label, UUID=b5a0ef7e-8fea-4608-bdef-4b08d9e45106
admin@roboRIO-3620-FRC:~# mkfs -t ext4 /dev/sda2
mke2fs 1.43.8 (1-Jan-2018)
Creating filesystem with 7271424 4k blocks and 1818624 inodes
Filesystem UUID: 6e817af1-45f2-4afb-a7dd-a7586e895880
Superblock backups stored on blocks:
        32768, 98304, 163840, 229376, 294912, 819200, 884736, 1605632, 2654208,
        4096000

Allocating group tables: done
Writing inode tables: done
Creating journal (32768 blocks): done
Writing superblocks and filesystem accounting information: done
admin@roboRIO-3620-FRC:~# cd /etc/init.d
admin@roboRIO-3620-FRC:/etc/init.d# cat > addswap.sh
#!/bin/sh
### BEGIN INIT INFO
# Provides:          swap
# Required-Start:    udev
# Required-Stop:
# Default-Start:     S
# Default-Stop:
# Short-Description: Add swap space
# Description:
### END INIT INFO

[ -x /sbin/swapon ] && swapon --summary
echo "Adding Swap"
[ -x /sbin/swapon ] && swapon --all --ifexists --verbose
[ -x /sbin/swapon ] && swapon --summary

: exit 0
admin@roboRIO-3620-FRC:/etc/init.d# chmod a+x addswap.sh
```
   

* Enable the addswap.sh: `update-rc.d -v addswap.sh defaults`
* Add this to fstab
```
/dev/sda1            none                 swap       sw                    0  0
```
* Issue these commands
```
admin@roboRIO-3620-FRC:/etc/init.d# swapon -s
admin@roboRIO-3620-FRC:/etc/init.d# vi /etc/fstab
admin@roboRIO-3620-FRC:/etc/init.d# swapon -a
admin@roboRIO-3620-FRC:/etc/init.d# swapon -s
Filename                                Type            Size    Used    Priority
/dev/sda1                               partition       975868  0       -2
admin@roboRIO-3620-FRC:/etc/init.d#
```
* Reboot roboRIO and verify swap space is present.
```
login as: lvuser
Pre-authentication banner message from server:
| NI Linux Real-Time (run mode)
|
| Log in with your NI-Auth credentials.
|
End of banner message from server
lvuser@roboRIO-3620-FRC:~# swapon -s
Filename                                Type            Size    Used    Priority
/dev/sda1                               partition       975868  0       -2
```