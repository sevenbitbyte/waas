waas_config - Readme
===

waas_config is a utility for performing pixel mapping with an LED array.


Requirements
---

1. [Qt5](http://qt-project.org/downloads)
2. [OLA](http://www.opendmx.net/index.php/Download_%26_Install_OLA)


Assumptions
---
* OLA is properly configured to talk to your LED array(probably using Streaming ACN or ArtNet plugin)
* Your display is color mapped to R,G,B order
* The display you are mapping consists of columns of vertical straight runs of LEDs

What Do?
---
When first started waas_config will attempt to connect to your local OLA server. If successful it will animate a fading white light at DMX address 1.0(universe 1, channel 0). Using the "Forward" button the cursor's position will be incremented by three DMX channels. The "Back" button decrements 3 channels. Reset jumps the cursor back to DMX 1.0. 


Mapping a run
---
* Click "Start Run" to make the current cursor position the start of a column of pixels.
* Move the cursor to the end of the column. The lights between start and the current cursor location will turn yellow.
* Set the correct column number
* Set the correct direction
* Click "End Run" to make hte current cursor position as the end of the column of pixels.
* The animation shown in the "Test Image" tab will be displayed

<b>Notes</b>
* Start is expected to come before end address, expect the unexpected
* Creating runs in an already assigned column will override the original run... (TODO: Probably a memory leak some where in there too)

Saving
---
At any point you can save the pixel map from the "File -> Save" menu. The format is JSON, describes what DMX addresses are to be assigned to a give image column. Includes a flag to reverse the DMX range.


Todo (priority order)
---
* Implement DmxRange
* Clean up memory space
* Pixel map loading
* Visualize cursor, existing runs, and selected DMX window
* Widgetize
* Preserve animation aspect ratio during window resize
