# Team 95 2025 season on-robot network notes



| IP                  | Description     |
|---------------------|-----------------|
| 10.0.95.1           | VH-109 radio    |
| 10.0.95.2  | RoboRIO         |
| 10.0.95.4  | FMS Field radio |
| 10.0.95.5  | reasonable static IP for Driver Station - if static, DS *must* use 255.0.0.0 subnet mask|
| 10.0.95.10 - .19 | IP Cameras, other devices |
| 10.0.95.20 - .199 | Field Network DHCP server range |
| 10.0.95.200 - .219 | VH-109 DHCP server range|

## Static IPs used so far

| IP         | Device             | Position            |
|------------|--------------------|---------------------|
| 10.0.95.13 | LimeLight 2+       | None                |
| 10.0.95.14 | 1st LimeLight 3G   | Lower, scoring side |
| 10.0.95.15 | 2nd LimeLight 3G   | Upper, loading side |
| 10.0.95.16 | Spare LimeLight 3G | Focus iffy, still reads apriltags.  Has L1 pipeline in Pipeline 1, L4 pipeline in pipeline 4    |

LimeLight 2+ and the two 3G's are on Firmware 2025.0 and are configured with AprilTag localization pipelines.  We did get a couple Google Coral TPUs to try Machine Learning game piece recognition as well, but have not yet found a use for them.

## ToDo
* Save the pipelines / settings of the two on-robot 3Gs for backup.
* pre-configure the spare 3G as much as possible to make swaping it into use as easy as possible
